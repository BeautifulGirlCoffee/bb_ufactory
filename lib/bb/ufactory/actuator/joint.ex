# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.Joint do
  @moduledoc """
  Joint-space position actuator for xArm joints.

  One instance runs per joint. On receiving a `%BB.Message.Actuator.Command.Position{}`
  command, the actuator clamps the angle to joint limits and writes the target
  `set_position` into the controller's ETS table. The controller's 100 Hz loop
  reads all pending `set_position` values and batches them into a single
  `cmd_move_joints` frame.

  Commands are accepted via two delivery paths:

  - **Pubsub** (`BB.Actuator.set_position/4`): arrives as
    `handle_info({:bb, [:actuator | path], msg}, state)`.
  - **Direct cast** (`BB.Actuator.set_position!/4`): arrives as
    `handle_cast({:command, msg}, state)`.

  Both paths share the same clamping and ETS write logic.

  ## ETS Write

  The actuator reads the current ETS row first to preserve `current_position`
  and `current_torque` written by the controller's report socket handler, then
  writes only the `set_position` field.

  ## BeginMotion

  A `BB.Message.Actuator.BeginMotion` message is published to
  `[:actuator | bb.path]` after each position command so that the open-loop
  position estimator can track expected arrival.
  """

  use BB.Actuator,
    options_schema: [
      joint: [
        type: {:in, 1..7},
        required: true,
        doc: "1-based joint index (1 = base joint)"
      ],
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller in the robot's registry"
      ]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Actuator
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    joint = Keyword.fetch!(opts, :joint)
    controller = Keyword.fetch!(opts, :controller)

    ets = BB.Process.call(bb.robot, controller, :get_ets)
    model_config = BB.Process.call(bb.robot, controller, :get_model_config)

    limits = Enum.at(model_config.limits, joint - 1)
    max_speed = model_config.max_speed_rads

    state = %{
      bb: bb,
      joint: joint,
      controller: controller,
      ets: ets,
      limits: limits,
      max_speed: max_speed
    }

    {:ok, state}
  end

  # ── disarm/1 — controller handles hardware stop ──────────────────────────────

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  # ── Pubsub delivery ──────────────────────────────────────────────────────────

  @impl BB.Actuator
  def handle_info(
        {:bb, [:actuator | _path], %Message{payload: %Command.Position{} = cmd}},
        state
      ) do
    state = apply_position_command(cmd, state)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # ── Direct cast delivery ─────────────────────────────────────────────────────

  @impl BB.Actuator
  def handle_cast({:command, %Message{payload: %Command.Position{} = cmd}}, state) do
    state = apply_position_command(cmd, state)
    {:noreply, state}
  end

  def handle_cast(_request, state), do: {:noreply, state}

  # ── Private helpers ──────────────────────────────────────────────────────────

  defp apply_position_command(%Command.Position{position: position} = cmd, state) do
    {lower, upper} = state.limits
    clamped = position |> max(lower) |> min(upper)

    if clamped != position do
      Logger.debug(
        "[BB.Ufactory.Actuator.Joint] J#{state.joint} position #{position} clamped to #{clamped}"
      )
    end

    cur_pos = write_set_position(state.ets, state.joint, clamped)
    publish_begin_motion(cmd, clamped, cur_pos, state)
    state
  end

  # Reads the current ETS row to preserve current_position and current_torque,
  # then writes only the set_position field. Returns current_position (may be nil).
  defp write_set_position(ets, joint, set_pos) do
    {cur_pos, cur_torq} =
      case :ets.lookup(ets, joint) do
        [{^joint, cp, ct, _sp}] -> {cp, ct}
        [] -> {nil, nil}
      end

    :ets.insert(ets, {joint, cur_pos, cur_torq, set_pos})
    cur_pos
  end

  defp publish_begin_motion(%Command.Position{} = cmd, target, cur_pos, state) do
    initial = cur_pos || target
    travel = abs(target - initial)
    # Estimate travel time (ms); clamp denominator to avoid division by zero.
    travel_ms = round(travel / max(state.max_speed, 0.001) * 1000)
    expected_arrival = System.monotonic_time(:millisecond) + travel_ms

    actuator_name = List.last(state.bb.path)

    extra = if cmd.command_id, do: [command_id: cmd.command_id], else: []

    case Message.new(
           BeginMotion,
           actuator_name,
           [
             initial_position: initial * 1.0,
             target_position: target * 1.0,
             expected_arrival: expected_arrival,
             command_type: :position
           ] ++ extra
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:actuator | state.bb.path], msg)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Actuator.Joint] J#{state.joint} failed to build BeginMotion: #{inspect(reason)}"
        )
    end
  end
end
