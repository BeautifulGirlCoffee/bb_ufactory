# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.Gripper do
  @moduledoc """
  Gripper G2 position actuator for xArm arms.

  Controls the UFactory Gripper G2 via the xArm RS485 RTU proxy (register
  0x7C). Position is expressed in **pulse units** (0–840). The G2 range is
  0–840, capped at 840 in `BB.Ufactory.Protocol.cmd_gripper_position/2`.

  ## Lifecycle

  The gripper is **not** enabled during `init/1`. Instead, the actuator
  registers its `cmd_gripper_enable(true)` and `cmd_gripper_speed` frames
  with the controller, which sends them at the end of its own arm sequence
  on every `:armed` transition. This guarantees no RS485 command reaches the
  bus before the arm controller is fully initialized (mode 0, state 0) —
  ordering a separate state-machine subscription could not provide, since
  pubsub dispatch order across subscribers is unspecified.

  ## Command Interface

  Receives standard `%BB.Message.Actuator.Command.Position{}` commands, where
  `position` is the target position in pulse units (0.0–840.0). Non-integer
  values are rounded to the nearest integer.

  ## Disarm

  On disarm, the actuator sends `cmd_gripper_enable(false)` through the
  controller via `BB.Process.call/3`, avoiding the need for `host`/`port`
  in the actuator's init opts.
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
        default: 1500,
        doc: "Gripper speed in pulse units per second (default: 1500)"
      ]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Ufactory.Protocol

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    speed = Keyword.get(opts, :speed, 1500)

    # Commands may be delivered over pubsub (BB.Actuator.set_position/4) —
    # nothing else subscribes this process to its own command topic.
    BB.subscribe(bb.robot, [:actuator | bb.path])

    register_arm_frames(bb.robot, controller, speed)

    {:ok, %{bb: bb, controller: controller, speed: speed}}
  end

  # ── disarm/1 — disable gripper via controller ───────────────────────────────

  @impl BB.Actuator
  def disarm(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    frame = Protocol.cmd_gripper_enable(0, false)

    try do
      BB.Process.call(bb.robot, controller, {:send_command, frame})
    catch
      _, _ -> :ok
    end

    :ok
  end

  # ── handle_cast position commands ────────────────────────────────────────────

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{position: pos}}}, state) do
    state = apply_gripper_position(pos, state)
    {:noreply, state}
  end

  def handle_cast(_request, state), do: {:noreply, state}

  # ── handle_info — pubsub delivery ──────────────────────────────────────────

  @impl BB.Actuator
  def handle_info(
        {:bb, [:actuator | _path], %Message{payload: %Command.Position{position: pos}}},
        state
      ) do
    state = apply_gripper_position(pos, state)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # ── Private helpers ──────────────────────────────────────────────────────────

  # Registers the enable + speed frames with the controller, which sends them
  # at the end of its arm sequence (or immediately if already armed).
  defp register_arm_frames(robot, controller, speed) do
    frames = [Protocol.cmd_gripper_enable(0, true), Protocol.cmd_gripper_speed(0, speed)]

    case BB.Process.call(robot, controller, {:register_arm_frames, :gripper, frames}) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.Gripper] arm-frame registration failed: #{inspect(reason)}"
        )
    end
  end

  defp apply_gripper_position(pos, state) do
    pos_int = round(pos) |> max(0) |> min(840)

    frame = Protocol.cmd_gripper_position(0, pos_int)

    case BB.Process.call(state.bb.robot, state.controller, {:send_command, frame}) do
      :ok ->
        publish_begin_motion(pos_int, state)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.Gripper] gripper_position(#{pos_int}) failed: #{inspect(reason)}"
        )
    end

    state
  end

  defp publish_begin_motion(pos_int, state) do
    actuator_name = List.last(state.bb.path)
    # Gripper moves at `speed` pulse units/s; estimate travel assuming worst case from 0.
    travel_ms = round(pos_int / max(state.speed, 1) * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_ms

    case Message.new(BeginMotion, actuator_name,
           initial_position: 0.0,
           target_position: pos_int * 1.0,
           expected_arrival: expected_arrival,
           command_type: :position
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:actuator | state.bb.path], msg)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.Gripper] Failed to build BeginMotion: #{inspect(reason)}"
        )
    end
  end
end
