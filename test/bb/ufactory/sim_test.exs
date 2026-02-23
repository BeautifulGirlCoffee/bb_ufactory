# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.SimTest do
  @moduledoc """
  Kinematic simulation integration tests.

  These tests start `BB.Ufactory.Robots.XArm6` in `simulation: :kinematic`
  mode (no hardware required) and verify that the BB framework wiring is
  correct:

  - `BB.Sim.Actuator` replaces all six joint actuators
  - `BB.Sim.Controller` (via `simulation: :mock`) replaces `BB.Ufactory.Controller`
  - `BB.Sensor.OpenLoopPositionEstimator` is auto-created per actuator
  - Joint limits declared in the DSL are enforced by `BB.Sim.Actuator`
  - `BeginMotion` and `JointState` messages flow through pubsub correctly

  ## Path Hierarchy

  The XArm6 DSL topology builds these paths at startup:

      BB.Supervisor → BB.LinkSupervisor(:base, parent_path=[])
        link_path = [:base]
        → BB.JointSupervisor(:j1, path=[:base])
            joint_path = [:base, :j1]
            actuator :j1_motor         → bb.path [:base, :j1, :j1_motor]
            sensor (estimator)         → bb.path [:base, :j1, :j1_motor_position_estimator]
          → BB.LinkSupervisor(:link1, parent_path=[:base, :j1])
              link_path = [:base, :j1, :link1]
            → BB.JointSupervisor(:j2, path=[:base, :j1, :link1])
                joint_path = [:base, :j1, :link1, :j2]
                actuator :j2_motor     → bb.path [:base, :j1, :link1, :j2, :j2_motor]
                ⋮

  BeginMotion is published to `[:actuator | actuator_path]`.
  JointState is published to `[:sensor | estimator_path]`.
  """

  use ExUnit.Case, async: false

  @moduletag :sim

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Sensor.JointState

  @robot BB.Ufactory.Robots.XArm6
  @pi :math.pi()

  # Actuator atom names (used with set_position!/3 for direct GenServer.cast).
  # set_position!/3 avoids publishing a Command.Position via pubsub, which would
  # crash OpenLoopPositionEstimator (it only handles BeginMotion, not commands).

  # ── Setup / teardown ──────────────────────────────────────────────────────────

  setup do
    {:ok, _pid} = @robot.start_link(simulation: :kinematic)

    on_exit(fn ->
      if Process.whereis(@robot) do
        try do
          Supervisor.stop(@robot, :normal, 2_000)
        catch
          :exit, _ -> :ok
        end
      end
    end)

    :ok
  end

  # ── Lifecycle ─────────────────────────────────────────────────────────────────

  describe "robot lifecycle in kinematic simulation" do
    test "starts with safety state :disarmed" do
      assert BB.Safety.state(@robot) == :disarmed
      refute BB.Safety.armed?(@robot)
    end

    test "can be armed" do
      assert :ok = BB.Safety.arm(@robot)
      assert BB.Safety.armed?(@robot)
    end

    test "can be disarmed after arming" do
      BB.Safety.arm(@robot)
      assert :ok = BB.Safety.disarm(@robot)
      assert BB.Safety.state(@robot) == :disarmed
    end
  end

  # ── BeginMotion via BB.Sim.Actuator ──────────────────────────────────────────

  describe "position commands publish BeginMotion via BB.Sim.Actuator" do
    setup do
      BB.Safety.arm(@robot)
      # Broad subscription: receives any message published to [:actuator, ...]
      BB.subscribe(@robot, [:actuator])
      on_exit(fn -> BB.Safety.disarm(@robot) end)
      :ok
    end

    test "J1 position command publishes BeginMotion with correct target" do
      # set_position! sends directly to the actuator via GenServer.cast (no pubsub
      # Command.Position message), so the OpenLoopPositionEstimator only sees the
      # BeginMotion that the Sim.Actuator publishes after accepting the command.
      BB.Actuator.set_position!(@robot, :j1_motor, 0.5)

      assert_receive {:bb, path, %Message{payload: %BeginMotion{} = bm}}, 1_000
      assert :j1_motor in path
      assert_in_delta bm.target_position, 0.5, 1.0e-4
      assert bm.command_type == :position
    end

    test "BeginMotion reports initial position at limits midpoint (0.0 for symmetric J1)" do
      # BB.Sim.Actuator initialises at the limits midpoint.
      # J1: {-2π, +2π} → midpoint = 0.0
      BB.Actuator.set_position!(@robot, :j1_motor, 1.0)

      assert_receive {:bb, _path, %Message{payload: %BeginMotion{} = bm}}, 1_000
      assert_in_delta bm.initial_position, 0.0, 1.0e-4
    end

    test "BeginMotion expected_arrival is in the future for non-trivial travel distance" do
      # 1.0 rad at π rad/s (J1 velocity limit) ≈ 318 ms
      BB.Actuator.set_position!(@robot, :j1_motor, 1.0)

      assert_receive {:bb, _path, %Message{payload: %BeginMotion{} = bm}}, 1_000
      assert bm.expected_arrival > System.monotonic_time(:millisecond)
    end
  end

  # ── Joint limit clamping ──────────────────────────────────────────────────────

  describe "joint limit clamping by BB.Sim.Actuator" do
    setup do
      BB.Safety.arm(@robot)
      BB.subscribe(@robot, [:actuator])
      on_exit(fn -> BB.Safety.disarm(@robot) end)
      :ok
    end

    test "J2 upper limit (~2.094 rad / 120°) clamps an out-of-range position" do
      # J2 upper ≈ 2.094 rad. Request π (~3.14 rad) — must clamp.
      BB.Actuator.set_position!(@robot, :j2_motor, @pi)

      assert_receive {:bb, path, %Message{payload: %BeginMotion{} = bm}}, 1_000
      assert :j2_motor in path
      assert bm.target_position <= 2.095
    end

    test "J2 lower limit (~-2.059 rad / -118°) clamps an out-of-range negative position" do
      # J2 lower ≈ -2.059 rad. Request -π (~-3.14 rad) — must clamp.
      BB.Actuator.set_position!(@robot, :j2_motor, -@pi)

      assert_receive {:bb, path, %Message{payload: %BeginMotion{} = bm}}, 1_000
      assert :j2_motor in path
      assert bm.target_position >= -2.060
    end

    test "J1 position within limits is not clamped" do
      # J1: {-2π, +2π}. 1.0 rad is well within range.
      BB.Actuator.set_position!(@robot, :j1_motor, 1.0)

      assert_receive {:bb, _path, %Message{payload: %BeginMotion{} = bm}}, 1_000
      assert_in_delta bm.target_position, 1.0, 1.0e-4
    end
  end

  # ── OpenLoopPositionEstimator ─────────────────────────────────────────────────

  describe "OpenLoopPositionEstimator publishes JointState after motion" do
    setup do
      BB.Safety.arm(@robot)
      # Broad subscription: receives any message published to [:sensor, ...]
      BB.subscribe(@robot, [:sensor])
      on_exit(fn -> BB.Safety.disarm(@robot) end)
      :ok
    end

    test "JointState is published for J1 after a position command" do
      # 1.0 rad at π rad/s ≈ 318 ms. Estimator ticks at 50 Hz during motion.
      BB.Actuator.set_position!(@robot, :j1_motor, 1.0)

      assert_receive {:bb, path, %Message{payload: %JointState{} = js}}, 2_000
      assert :j1 in path
      assert js.names == [:j1]
      assert [pos] = js.positions
      assert pos >= -@pi * 2 and pos <= @pi * 2
    end

    test "multiple JointState messages are published during motion" do
      # 1.5 rad at π rad/s ≈ 477 ms. At 50 Hz that is ~24 ticks.
      BB.Actuator.set_position!(@robot, :j1_motor, 1.5)

      positions = collect_joint_state_positions(2_500)
      assert length(positions) >= 2
    end
  end

  # ── Helpers ───────────────────────────────────────────────────────────────────

  # Collects all JointState position values published within `timeout_ms`.
  defp collect_joint_state_positions(timeout_ms) do
    collect_joint_state_positions([], timeout_ms, System.monotonic_time(:millisecond))
  end

  defp collect_joint_state_positions(acc, timeout_ms, start_ms) do
    remaining = timeout_ms - (System.monotonic_time(:millisecond) - start_ms)

    if remaining <= 0 do
      acc
    else
      receive do
        {:bb, _path, %Message{payload: %JointState{positions: positions}}} ->
          collect_joint_state_positions(acc ++ positions, timeout_ms, start_ms)
      after
        remaining -> acc
      end
    end
  end
end
