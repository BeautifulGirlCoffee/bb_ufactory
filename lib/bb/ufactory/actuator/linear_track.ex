# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.LinearTrack do
  @moduledoc """
  Linear track position actuator for xArm arms.

  Controls the UFactory linear track via the xArm RS485 RTU proxy. The track
  position is expressed in **millimetres** and converted internally to the
  hardware's native int32 encoding (`round(mm * 2000)`).

  Unlike joint positions which are batched at 100 Hz via ETS, linear track
  commands are forwarded immediately to the controller via
  `BB.Process.call/3`.

  ## Lifecycle

  The track motor is **not** enabled during `init/1`. Instead, the actuator
  subscribes to state machine transitions and enables the track when the
  robot transitions to `:armed`. This avoids sending RS485 commands before
  the arm controller is fully initialized (mode 0, state 0), which can
  trigger spurious servo motor faults on the controller bus.

  ## Protocol Note

  The linear track uses big-endian int32 encoding for position — the only
  place in the UFactory protocol where position is not little-endian fp32.
  This is handled transparently by `BB.Ufactory.Protocol.cmd_linear_track_move/3`,
  which returns `{pos_frame, spd_frame}`. Both frames are sent sequentially:
  speed first, then position.

  ## Command Interface

  Receives standard `%BB.Message.Actuator.Command.Position{}` commands, where
  `position` is the target position in millimetres.
  """

  use BB.Actuator,
    options_schema: [
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller in the robot's registry"
      ],
      speed: [
        type: :pos_integer,
        default: 200,
        doc: "Linear track speed in mm/s (default: 200)"
      ]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.StateMachine.Transition
  alias BB.Ufactory.Protocol

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    speed = Keyword.get(opts, :speed, 200)

    BB.subscribe(bb.robot, [:state_machine])

    {:ok, %{bb: bb, controller: controller, speed: speed}}
  end

  # ── disarm/1 — disable track motor when arm disarms ──────────────────────────

  @impl BB.Actuator
  def disarm(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    frame = Protocol.cmd_linear_track_enable(0, false)

    try do
      BB.Process.call(bb.robot, controller, {:send_command, frame})
    catch
      _, _ -> :ok
    end

    :ok
  end

  # ── handle_cast position commands ────────────────────────────────────────────

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{position: pos_mm}}}, state) do
    state = apply_track_position(pos_mm, state)
    {:noreply, state}
  end

  def handle_cast(_request, state), do: {:noreply, state}

  # ── handle_info — state machine transitions ─────────────────────────────────

  @impl BB.Actuator
  def handle_info(
        {:bb, [:state_machine], %Message{payload: %Transition{to: :armed}}},
        state
      ) do
    enable_track(state.bb.robot, state.controller)
    {:noreply, state}
  end

  # ── handle_info — pubsub delivery ──────────────────────────────────────────

  @impl BB.Actuator
  def handle_info(
        {:bb, [:actuator | _path], %Message{payload: %Command.Position{position: pos_mm}}},
        state
      ) do
    state = apply_track_position(pos_mm, state)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # ── Private helpers ──────────────────────────────────────────────────────────

  defp apply_track_position(pos_mm, state) do
    initial_position = read_track_position(state.bb.robot, state.controller)
    {pos_frame, spd_frame} = Protocol.cmd_linear_track_move(0, pos_mm, state.speed)

    # Speed must be set before position so the arm uses the new speed for this move.
    with :ok <- BB.Process.call(state.bb.robot, state.controller, {:send_command, spd_frame}),
         :ok <- BB.Process.call(state.bb.robot, state.controller, {:send_command, pos_frame}) do
      publish_begin_motion(pos_mm, initial_position, state)
    else
      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.LinearTrack] send_command failed: #{inspect(reason)}"
        )
    end

    state
  end

  defp enable_track(robot, controller) do
    frame = Protocol.cmd_linear_track_enable(0, true)

    case BB.Process.call(robot, controller, {:send_command, frame}) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.LinearTrack] track enable failed: #{inspect(reason)}"
        )
    end
  end

  defp read_track_position(robot, controller) do
    frame = Protocol.cmd_linear_track_read_position(0)

    case BB.Process.call(robot, controller, {:send_and_recv, frame}) do
      {:ok, {_reg, 0x00, params}, _rest} ->
        case Protocol.parse_linear_track_position(params) do
          {:ok, pos_mm} -> pos_mm
          _ -> 0.0
        end

      _ ->
        0.0
    end
  end

  defp publish_begin_motion(pos_mm, initial_position, state) do
    actuator_name = List.last(state.bb.path)
    travel_distance = abs(pos_mm - initial_position)
    travel_ms = round(travel_distance / max(state.speed, 1) * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_ms

    case Message.new(BeginMotion, actuator_name,
           initial_position: initial_position * 1.0,
           target_position: pos_mm * 1.0,
           expected_arrival: expected_arrival,
           command_type: :position
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:actuator | state.bb.path], msg)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.LinearTrack] Failed to build BeginMotion: #{inspect(reason)}"
        )
    end
  end
end
