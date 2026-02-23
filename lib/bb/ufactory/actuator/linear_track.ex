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
  alias BB.Ufactory.Protocol

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    speed = Keyword.get(opts, :speed, 200)

    {:ok, %{bb: bb, controller: controller, speed: speed}}
  end

  # ── disarm/1 — track coasts when arm disarms ─────────────────────────────────

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  # ── handle_cast position commands ────────────────────────────────────────────

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{position: pos_mm}}}, state) do
    state = apply_track_position(pos_mm, state)
    {:noreply, state}
  end

  def handle_cast(_request, state), do: {:noreply, state}

  # ── handle_info pubsub delivery ──────────────────────────────────────────────

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
    {pos_frame, spd_frame} = Protocol.cmd_linear_track_move(0, pos_mm, state.speed)

    # Speed must be set before position so the arm uses the new speed for this move.
    with :ok <- BB.Process.call(state.bb.robot, state.controller, {:send_command, spd_frame}),
         :ok <- BB.Process.call(state.bb.robot, state.controller, {:send_command, pos_frame}) do
      publish_begin_motion(pos_mm, state)
    else
      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.LinearTrack] send_command failed: #{inspect(reason)}"
        )
    end

    state
  end

  defp publish_begin_motion(pos_mm, state) do
    actuator_name = List.last(state.bb.path)
    travel_ms = round(abs(pos_mm) / max(state.speed, 1) * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_ms

    case Message.new(BeginMotion, actuator_name,
           initial_position: 0.0,
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
