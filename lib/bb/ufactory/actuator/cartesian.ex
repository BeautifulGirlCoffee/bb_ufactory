# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.Cartesian do
  @moduledoc """
  Cartesian-space position actuator for xArm arms.

  Sends end-effector pose commands (x, y, z, roll, pitch, yaw) directly to
  the arm using register 0x15 (`MOVE_LINE`). The arm's own controller solves
  inverse kinematics internally, so no IK computation is needed here.

  Unlike `BB.Ufactory.Actuator.Joint`, this actuator **bypasses the ETS batch
  loop** — frames are forwarded immediately to the controller via
  `BB.Process.call/3` and dispatched to the hardware on the same call.

  ## Command Interface

  The actuator declares `BB.Ufactory.Message.Command.CartesianMove` via
  `c:BB.Actuator.command_payloads/1`, so pose commands travel through BB's
  gated command pipeline (armed check, refusals reaching the caller):

      alias BB.Ufactory.Message.Command.CartesianMove

      msg = BB.Message.new!(CartesianMove, :tcp,
        x: 300.0, y: 0.0, z: 400.0, roll: 3.14159, pitch: 0.0, yaw: 0.0)

      # Synchronous — learn whether the arm accepted the command:
      {:ok, :accepted} = BB.call(MyRobot, :tcp, {:command, msg})

      # Fire-and-forget:
      BB.cast(MyRobot, :tcp, {:command, msg})

  - `x`, `y`, `z` — position in **millimetres**
  - `roll`, `pitch`, `yaw` — orientation in **radians**

  `speed` and `acceleration` default to the values configured in
  `options_schema` and can be overridden per-command in the payload.

  ### Legacy cast interface

  The pre-0.2 raw cast is still accepted for backwards compatibility, but it
  bypasses the armed check the command pipeline provides — prefer
  `CartesianMove`:

      BB.Process.cast(robot, :cartesian, {:move_cartesian, {x, y, z, roll, pitch, yaw}})
      BB.Process.cast(robot, :cartesian, {:move_cartesian, pose, speed, accel})
  """

  use BB.Actuator,
    options_schema: [
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller in the robot's registry"
      ],
      speed: [
        type: :number,
        default: 100.0,
        doc: "End-effector linear speed in mm/s (default: 100.0)"
      ],
      acceleration: [
        type: :number,
        default: 2000.0,
        doc: "End-effector linear acceleration in mm/s² (default: 2000.0)"
      ]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Ufactory.Message.Command.CartesianMove
  alias BB.Ufactory.Protocol

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    speed = Keyword.get(opts, :speed, 100.0)
    acceleration = Keyword.get(opts, :acceleration, 2000.0)

    ets =
      case BB.Process.call(bb.robot, controller, :get_ets) do
        ref when is_reference(ref) or is_atom(ref) -> ref
        _ -> nil
      end

    {:ok,
     %{
       bb: bb,
       controller: controller,
       speed: speed * 1.0,
       acceleration: acceleration * 1.0,
       ets: ets
     }}
  end

  # ── disarm/1 — controller handles hardware stop ──────────────────────────────

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  # ── Accepted command payloads ────────────────────────────────────────────────

  # MOVE_LINE has no scoped stop: firmware state 4 halts the whole arm and
  # clears every queued command, which would silently fight the 100 Hz joint
  # streaming loop. Stopping the arm is the safety system's job (disarm), so
  # only the pose command is declared here.
  @impl BB.Actuator
  def command_payloads(_opts), do: [CartesianMove]

  # ── handle_command/2 — gated command pipeline ────────────────────────────────

  @impl BB.Actuator
  def handle_command(%Message{payload: %CartesianMove{} = cmd}, state) do
    # Both operands are already floats: the payload schema types speed and
    # acceleration as :float, and init/1 coerces the configured defaults.
    pose = {cmd.x, cmd.y, cmd.z, cmd.roll, cmd.pitch, cmd.yaw}
    speed = cmd.speed || state.speed
    accel = cmd.acceleration || state.acceleration

    case send_cartesian(pose, speed, accel, state) do
      :ok -> {:noreply, state}
      {:error, reason} -> {:reply, {:error, reason}, state}
    end
  end

  def handle_command(%Message{payload: payload}, state) do
    {:reply, {:error, {:unsupported_command, payload.__struct__}}, state}
  end

  # ── Legacy direct cast: {:move_cartesian, pose} ─────────────────────────────

  @impl BB.Actuator
  def handle_cast({:move_cartesian, {_x, _y, _z, _roll, _pitch, _yaw} = pose}, state) do
    send_cartesian(pose, state.speed, state.acceleration, state)
    {:noreply, state}
  end

  def handle_cast({:move_cartesian, pose, speed, accel}, state) do
    send_cartesian(pose, speed * 1.0, accel * 1.0, state)
    {:noreply, state}
  end

  def handle_cast(_request, state), do: {:noreply, state}

  # ── Private helpers ──────────────────────────────────────────────────────────

  defp send_cartesian(pose, speed, accel, state) do
    frame = Protocol.cmd_move_cartesian(0, pose, speed, accel)

    case BB.Process.call(state.bb.robot, state.controller, {:send_command, frame}) do
      :ok ->
        publish_begin_motion(pose, speed, state)
        :ok

      {:error, reason} = error ->
        Logger.warning("[BB.Ufactory.Actuator.Cartesian] send_command failed: #{inspect(reason)}")
        error
    end
  end

  # BeginMotion for a Cartesian move is expressed as scalar travel distance:
  # initial_position 0.0 → target_position <distance in METRES> (BB's contract
  # for prismatic-style positions is metres, not millimetres). The arrival
  # estimate uses the speed this command was actually sent with, which may be
  # a per-command override rather than the configured default.
  defp publish_begin_motion({x, y, z, _roll, _pitch, _yaw}, speed, state) do
    actuator_name = List.last(state.bb.path)
    {cx, cy, cz} = current_position(state)
    dx = x - cx
    dy = y - cy
    dz = z - cz
    distance_mm = :math.sqrt(dx * dx + dy * dy + dz * dz)
    travel_ms = round(distance_mm / max(speed, 0.001) * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_ms

    case Message.new(BeginMotion, actuator_name,
           initial_position: 0.0,
           target_position: distance_mm / 1000.0,
           expected_arrival: expected_arrival,
           command_type: :position
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:actuator | state.bb.path], msg)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.Cartesian] Failed to build BeginMotion: #{inspect(reason)}"
        )
    end
  end

  defp current_position(%{ets: ets}) when not is_nil(ets) do
    case :ets.lookup(ets, :arm) do
      [{:arm, _state, _mode, {x, y, z, _roll, _pitch, _yaw}}] -> {x, y, z}
      _ -> {0.0, 0.0, 0.0}
    end
  rescue
    ArgumentError -> {0.0, 0.0, 0.0}
  end

  defp current_position(_state), do: {0.0, 0.0, 0.0}
end
