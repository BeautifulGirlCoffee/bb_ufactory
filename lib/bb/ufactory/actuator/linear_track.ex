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
  registers its enable frame with the controller, which sends it at the end
  of its own arm sequence on every `:armed` transition. This guarantees no
  RS485 command reaches the bus before the arm controller is fully
  initialized (mode 0, state 0) — ordering a separate state-machine
  subscription could not provide, since pubsub dispatch order across
  subscribers is unspecified.

  ## Protocol Note

  The linear track uses big-endian int32 encoding for position — the only
  place in the UFactory protocol where position is not little-endian fp32.
  This is handled transparently by `BB.Ufactory.Protocol.cmd_linear_track_move/3`,
  which returns `{pos_frame, spd_frame}`. Both frames are sent sequentially:
  speed first, then position.

  ## Command Interface

  All transports converge on `c:BB.Actuator.handle_command/2`. Accepted
  commands:

  - `Command.Position` — target position in millimetres, clamped to
    `[0, stroke_mm]`.
  - `Command.Stop` — reads the track's current position over RS485 and
    re-targets it, braking the carriage in place. Refused when the position
    read fails (commanding an assumed position could move the track).

  A synchronous caller (`BB.Actuator.set_position/4`) receives
  `{:error, reason}` when the controller cannot deliver the frames.
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
      ],
      stroke_mm: [
        type: :pos_integer,
        default: 700,
        doc:
          "Track stroke length in millimetres; position commands are clamped to " <>
            "[0, stroke_mm]. UFactory tracks ship in 700 mm and 1000 mm variants " <>
            "(default: 700)"
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
    stroke_mm = Keyword.get(opts, :stroke_mm, 700)

    register_arm_frames(bb.robot, controller)

    {:ok, %{bb: bb, controller: controller, speed: speed, stroke_mm: stroke_mm}}
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

  # ── Accepted command payloads ────────────────────────────────────────────────

  @impl BB.Actuator
  def command_payloads(_opts), do: [Command.Position, Command.Stop]

  # ── handle_command/2 — all transports converge here ─────────────────────────

  @impl BB.Actuator
  def handle_command(%Message{payload: %Command.Position{position: pos_mm}}, state) do
    case apply_track_position(pos_mm, state) do
      :ok -> {:noreply, state}
      {:error, reason} -> {:reply, {:error, reason}, state}
    end
  end

  # Braking the carriage means re-targeting its current position — the servo
  # tracks the most recent target, so the write preempts the move in flight.
  # Refused when the position read fails: commanding an assumed position
  # (e.g. 0.0) would MOVE the track rather than stop it.
  def handle_command(%Message{payload: %Command.Stop{}}, state) do
    case read_track_position(state.bb.robot, state.controller) do
      {:ok, pos_mm} ->
        case apply_track_position(pos_mm, state) do
          :ok -> {:noreply, state}
          {:error, reason} -> {:reply, {:error, reason}, state}
        end

      :error ->
        {:reply, {:error, :position_unknown}, state}
    end
  end

  def handle_command(%Message{payload: payload}, state) do
    {:reply, {:error, {:unsupported_command, payload.__struct__}}, state}
  end

  # ── Private helpers ──────────────────────────────────────────────────────────

  defp apply_track_position(pos_mm, state) do
    clamped = pos_mm |> max(0.0) |> min(state.stroke_mm)

    if clamped != pos_mm do
      Logger.debug(
        "[BB.Ufactory.Actuator.LinearTrack] position #{pos_mm} clamped to #{clamped} " <>
          "(stroke #{state.stroke_mm} mm)"
      )
    end

    pos_mm = clamped

    initial_position =
      case read_track_position(state.bb.robot, state.controller) do
        {:ok, pos} -> pos
        :error -> 0.0
      end

    {pos_frame, spd_frame} = Protocol.cmd_linear_track_move(0, pos_mm, state.speed)

    # Speed must be set before position so the arm uses the new speed for this move.
    with :ok <- BB.Process.call(state.bb.robot, state.controller, {:send_command, spd_frame}),
         :ok <- BB.Process.call(state.bb.robot, state.controller, {:send_command, pos_frame}) do
      publish_begin_motion(pos_mm, initial_position, state)
      :ok
    else
      {:error, reason} = error ->
        Logger.warning(
          "[BB.Ufactory.Actuator.LinearTrack] send_command failed: #{inspect(reason)}"
        )

        error
    end
  end

  # Registers the enable frame with the controller, which sends it at the end
  # of its arm sequence (or immediately if already armed).
  defp register_arm_frames(robot, controller) do
    frames = [Protocol.cmd_linear_track_enable(0, true)]

    case BB.Process.call(robot, controller, {:register_arm_frames, :linear_track, frames}) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.LinearTrack] arm-frame registration failed: #{inspect(reason)}"
        )
    end
  end

  defp read_track_position(robot, controller) do
    frame = Protocol.cmd_linear_track_read_position(0)

    case BB.Process.call(robot, controller, {:send_and_recv, frame}) do
      {:ok, {_reg, 0x00, params}, _rest} ->
        case Protocol.parse_linear_track_position(params) do
          {:ok, pos_mm} -> {:ok, pos_mm}
          _ -> :error
        end

      _ ->
        :error
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
