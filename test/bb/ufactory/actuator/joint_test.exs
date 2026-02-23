# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.JointTest do
  use ExUnit.Case, async: false
  use Mimic

  setup :verify_on_exit!

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Ufactory.Actuator.Joint

  @pi :math.pi()
  @two_pi 2 * :math.pi()

  # xArm6 joint 2 limits: -2.059 to 2.094 rad
  @j2_lower -2.059488
  @j2_upper 2.094395

  # xArm6 model config
  @model_config %{
    joints: 6,
    max_speed_rads: @pi,
    limits: [
      {-@two_pi, @two_pi},
      {@j2_lower, @j2_upper},
      {-3.926990, 0.191986},
      {-@two_pi, @two_pi},
      {-1.692969, @pi},
      {-@two_pi, @two_pi}
    ]
  }

  defp make_ets do
    ets = :ets.new(:joint_test_ets, [:public, :set])
    for i <- 1..6, do: :ets.insert(ets, {i, nil, nil, nil})
    :ets.insert(ets, {:arm, 0, 0, nil})
    ets
  end

  defp make_state(ets, joint \\ 2) do
    %{
      bb: %{robot: TestRobot, path: [:j2, :motor]},
      joint: joint,
      controller: :xarm,
      ets: ets,
      limits: Enum.at(@model_config.limits, joint - 1),
      max_speed: @model_config.max_speed_rads
    }
  end

  defp position_msg(position, opts \\ []) do
    Message.new!(Command.Position, :motor,
      position: position * 1.0,
      command_id: opts[:command_id]
    )
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "fetches ETS ref and model config from controller via BB.Process.call" do
      ets = make_ets()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, :get_ets -> ets end)
      |> expect(:call, fn TestRobot, :xarm, :get_model_config -> @model_config end)

      opts = [bb: %{robot: TestRobot, path: [:j2, :motor]}, joint: 2, controller: :xarm]
      assert {:ok, state} = Joint.init(opts)

      assert state.ets == ets
      assert state.joint == 2
      assert state.controller == :xarm
      assert state.limits == {@j2_lower, @j2_upper}
      assert state.max_speed == @pi
    end

    test "sets limits for the correct joint index (1-based)" do
      ets = make_ets()

      BB.Process
      |> stub(:call, fn
        TestRobot, :xarm, :get_ets -> ets
        TestRobot, :xarm, :get_model_config -> @model_config
      end)

      # Joint 1 should have ±2π limits
      opts = [bb: %{robot: TestRobot, path: [:j1, :motor]}, joint: 1, controller: :xarm]
      assert {:ok, state} = Joint.init(opts)
      assert state.limits == {-@two_pi, @two_pi}
    end
  end

  # ── handle_cast position commands ────────────────────────────────────────────

  describe "handle_cast({:command, %Command.Position{}}, state)" do
    test "writes set_position to ETS for an in-range position" do
      ets = make_ets()
      :ets.insert(ets, {2, 0.5, 0.1, nil})
      state = make_state(ets)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(1.0)
      assert {:noreply, ^state} = Joint.handle_cast({:command, msg}, state)

      assert [{2, 0.5, 0.1, 1.0}] = :ets.lookup(ets, 2)
    end

    test "preserves current_position and current_torque when writing set_position" do
      ets = make_ets()
      :ets.insert(ets, {2, 0.3, 0.05, nil})
      state = make_state(ets)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(1.5)
      Joint.handle_cast({:command, msg}, state)

      [{2, cur_pos, cur_torq, set_pos}] = :ets.lookup(ets, 2)
      assert cur_pos == 0.3
      assert cur_torq == 0.05
      assert set_pos == 1.5
    end

    test "clamps position above upper limit to upper limit" do
      ets = make_ets()
      state = make_state(ets)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(@j2_upper + 1.0)
      Joint.handle_cast({:command, msg}, state)

      [{2, _cur_pos, _cur_torq, set_pos}] = :ets.lookup(ets, 2)
      assert_in_delta set_pos, @j2_upper, 0.0001
    end

    test "clamps position below lower limit to lower limit" do
      ets = make_ets()
      state = make_state(ets)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(@j2_lower - 1.0)
      Joint.handle_cast({:command, msg}, state)

      [{2, _cur_pos, _cur_torq, set_pos}] = :ets.lookup(ets, 2)
      assert_in_delta set_pos, @j2_lower, 0.0001
    end

    test "publishes BeginMotion with correct initial_position, target_position, expected_arrival" do
      ets = make_ets()
      :ets.insert(ets, {2, 0.0, 0.0, nil})
      state = make_state(ets)

      test_pid = self()

      BB
      |> expect(:publish, fn _robot,
                             [:actuator | _path],
                             %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = position_msg(1.0)

      before_ms = System.monotonic_time(:millisecond)
      Joint.handle_cast({:command, msg}, state)
      after_ms = System.monotonic_time(:millisecond)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.initial_position, 0.0, 0.0001
      assert_in_delta bm.target_position, 1.0, 0.0001
      # expected_arrival should be in the future relative to the time of the call
      travel_ms = round(abs(1.0 - 0.0) / @pi * 1000)
      assert bm.expected_arrival >= before_ms + travel_ms
      assert bm.expected_arrival <= after_ms + travel_ms + 5
    end

    test "publishes BeginMotion with initial_position = target when current_position is nil" do
      ets = make_ets()
      state = make_state(ets)

      test_pid = self()

      BB
      |> expect(:publish, fn _robot, _path, %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = position_msg(0.5)
      Joint.handle_cast({:command, msg}, state)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.initial_position, 0.5, 0.0001
      assert_in_delta bm.target_position, 0.5, 0.0001
    end

    test "passes command_id through to BeginMotion" do
      ets = make_ets()
      state = make_state(ets)
      ref = make_ref()
      test_pid = self()

      BB
      |> expect(:publish, fn _robot, _path, %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = position_msg(0.5, command_id: ref)
      Joint.handle_cast({:command, msg}, state)

      assert_receive {:begin_motion, bm}, 500
      assert bm.command_id == ref
    end
  end

  # ── handle_info pubsub delivery ──────────────────────────────────────────────

  describe "handle_info({:bb, [:actuator | path], %Command.Position{}}, state)" do
    test "applies the same position command logic as handle_cast" do
      ets = make_ets()
      :ets.insert(ets, {2, 0.0, 0.0, nil})
      state = make_state(ets)

      test_pid = self()

      BB
      |> expect(:publish, fn _robot, _path, %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = position_msg(1.0)
      assert {:noreply, ^state} = Joint.handle_info({:bb, [:actuator, :j2, :motor], msg}, state)

      [{2, _cur_pos, _cur_torq, set_pos}] = :ets.lookup(ets, 2)
      assert_in_delta set_pos, 1.0, 0.0001
      assert_receive {:begin_motion, _bm}, 500
    end

    test "clamps positions in the pubsub path the same as in the cast path" do
      ets = make_ets()
      state = make_state(ets)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(@j2_upper + 5.0)
      Joint.handle_info({:bb, [:actuator, :j2, :motor], msg}, state)

      [{2, _cur_pos, _cur_torq, set_pos}] = :ets.lookup(ets, 2)
      assert_in_delta set_pos, @j2_upper, 0.0001
    end

    test "ignores unrecognised messages" do
      state = make_state(make_ets())
      assert {:noreply, ^state} = Joint.handle_info(:unexpected, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "returns :ok without doing anything" do
      assert :ok = Joint.disarm([])
    end
  end
end
