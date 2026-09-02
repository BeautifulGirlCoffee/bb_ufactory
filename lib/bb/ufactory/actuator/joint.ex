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

  All command transports (`BB.Actuator.set_position/4` over pubsub, direct
  cast, synchronous call) converge on `c:BB.Actuator.handle_command/2` —
  `BB.Actuator.Server` subscribes to the command topic, checks that the robot
  is armed, and gates payload types before the driver sees them.

  ## Accepted commands

  - `Command.Position` — clamp to joint limits, write `set_position` to ETS.
  - `Command.Stop` — brake this joint by latching its current reported
    position as the target. The 100 Hz loop keeps commanding that position, so
    the firmware decelerates the joint and it stays put. If no report frame
    has arrived yet, the pending target is cleared instead — nothing has been
    dispatched, so clearing it cancels the motion.
  - `Command.Hold` — same latch as `Stop`; in position mode, holding the
    current angle actively resists external force. Refused when the current
    position is unknown (no report frame yet).

  Per-joint "passive" stop does not exist on this hardware (motor enable is
  arm-wide), so both `Stop` and `Hold` brake at the current position. Making
  the whole arm safe is the controller's `disarm/1`.

  ## Capabilities

  Declares `:position_feedback` and `:effort_feedback`: the controller
  publishes `BB.Message.Sensor.JointState` (angles + torques from every
  ~100 Hz report frame) on `[:sensor, controller_name]`, so joints driven by
  this actuator do not need a separate position sensor.

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

    if joint > model_config.joints do
      # Fail at init instead of crash-looping with a MatchError on the first
      # position command (Enum.at below would return nil limits).
      {:stop, {:invalid_joint, joint, model_config.joints}}
    else
      model_limits = Enum.at(model_config.limits, joint - 1)
      limits = intersect_topology_limits(bb, model_limits)
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
  end

  # ── disarm/1 — controller handles hardware stop ──────────────────────────────

  @impl BB.Actuator
  def disarm(_opts), do: :ok

  # ── Capabilities and accepted command payloads ──────────────────────────────

  # The controller reads position and torque back from every report frame and
  # publishes them as JointState, so the joints this actuator drives need no
  # separate position sensor. Velocity is not present in report frames.
  @impl BB.Actuator
  def capabilities(_opts), do: [:position_feedback, :effort_feedback]

  @impl BB.Actuator
  def command_payloads(_opts), do: [Command.Position, Command.Stop, Command.Hold]

  # ── handle_command/2 — all transports converge here ─────────────────────────

  @impl BB.Actuator
  def handle_command(%Message{payload: %Command.Position{} = cmd}, state) do
    {:noreply, apply_position_command(cmd, state)}
  end

  # Both :immediate and :decelerate stop modes brake at the current position —
  # the firmware's own planner always decelerates smoothly, so the distinction
  # has no hardware expression here.
  def handle_command(%Message{payload: %Command.Stop{}}, state) do
    case brake_at_current(state) do
      :ok ->
        {:noreply, state}

      :no_feedback ->
        # No report frame yet means the loop has never dispatched a move
        # (it skips ticks while any joint's position is unknown), so
        # clearing the pending target cancels the motion outright.
        clear_set_position(state.ets, state.joint)
        {:noreply, state}
    end
  end

  def handle_command(%Message{payload: %Command.Hold{}}, state) do
    case brake_at_current(state) do
      :ok -> {:noreply, state}
      :no_feedback -> {:reply, {:error, :position_unknown}, state}
    end
  end

  def handle_command(%Message{payload: payload}, state) do
    {:reply, {:error, {:unsupported_command, payload.__struct__}}, state}
  end

  # ── Private helpers ──────────────────────────────────────────────────────────

  # Narrows the factory model limits by the joint limits declared in the
  # robot's topology DSL (`limit do ... end`), so a user who tightens a
  # joint's range in their robot module gets that range enforced by the
  # clamp. Topology limits can only narrow, never widen, the factory range.
  # Falls back to the model limits when the topology gives no usable limit —
  # the `function_exported?` guard covers hand-built robots in tests that
  # never define `robot/0`, and the `with` else covers a missing actuator
  # entry, an unknown joint, or non-numeric limits.
  defp intersect_topology_limits(bb, {model_lower, model_upper} = model_limits) do
    actuator_name = List.last(bb.path)

    with true <- Code.ensure_loaded?(bb.robot) and function_exported?(bb.robot, :robot, 0),
         robot = bb.robot.robot(),
         %{joint: joint_name} <- Map.get(robot.actuators, actuator_name),
         {:ok, %BB.Robot.Joint{limits: %{lower: lower, upper: upper}}} <-
           BB.Robot.get_joint(robot, joint_name),
         true <- is_number(lower) and is_number(upper) do
      {max(model_lower, lower * 1.0), min(model_upper, upper * 1.0)}
    else
      _ -> model_limits
    end
  end

  defp apply_position_command(%Command.Position{position: position} = cmd, state) do
    {lower, upper} = state.limits
    clamped = position |> max(lower) |> min(upper)

    if clamped != position do
      Logger.debug(
        "[BB.Ufactory.Actuator.Joint] J#{state.joint} position #{position} clamped to #{clamped}"
      )
    end

    # The velocity hint travels with the target; cmd_move_joints takes one
    # speed for the whole batch, so the loop uses the slowest pending hint.
    cur_pos = write_set_position(state.ets, state.joint, clamped, cmd.velocity)
    publish_begin_motion(cmd, clamped, cur_pos, state)
    state
  end

  # Latches the joint's current reported position as its target, so the 100 Hz
  # loop brakes the joint there. The velocity hint is cleared — braking should
  # happen at the model's full speed, not at a leisurely pace a previous move
  # requested. Returns :no_feedback when no report frame has populated
  # current_position yet.
  defp brake_at_current(state) do
    case :ets.lookup(state.ets, state.joint) do
      [{_joint, cur_pos, _cur_torq, _sp, _sv}] when is_number(cur_pos) ->
        write_set_position(state.ets, state.joint, cur_pos, nil)
        :ok

      _ ->
        :no_feedback
    end
  end

  # The controller's report handler writes the current_position and
  # current_torque columns of the same rows at ~100 Hz from its own process.
  # Writes here must therefore touch ONLY the set_position/set_velocity
  # columns — a read-whole-row-then-insert could interleave with a report
  # update and clobber a fresh angle (or let the controller resurrect a stale
  # target). :ets.update_element/3 is atomic per row.
  defp clear_set_position(ets, joint) do
    :ets.update_element(ets, joint, [{4, nil}, {5, nil}])
    :ok
  end

  # Writes only the set_position/set_velocity columns. Returns
  # current_position (may be nil); the read is advisory (BeginMotion's
  # initial-position estimate), so a report frame landing between the lookup
  # and the update is harmless.
  defp write_set_position(ets, joint, set_pos, set_vel) do
    cur_pos =
      case :ets.lookup(ets, joint) do
        [{^joint, cp, _ct, _sp, _sv}] -> cp
        [] -> nil
      end

    :ets.update_element(ets, joint, [{4, set_pos}, {5, set_vel}]) ||
      :ets.insert(ets, {joint, nil, nil, set_pos, set_vel})

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
