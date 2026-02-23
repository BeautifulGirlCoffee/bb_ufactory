# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Sensor.CollisionTest do
  use ExUnit.Case, async: false
  use Mimic

  setup :verify_on_exit!

  alias BB.Message
  alias BB.Ufactory.Message.ArmStatus
  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Sensor.Collision

  defp make_state do
    %{
      bb: %{robot: TestRobot, path: [:collision]},
      controller: :xarm
    }
  end

  defp make_opts(overrides \\ []) do
    [bb: %{robot: TestRobot, path: [:collision]}, controller: :xarm] ++ overrides
  end

  defp arm_status_msg(error_code, extra \\ []) do
    state = Keyword.get(extra, :state, 0)
    mode = Keyword.get(extra, :mode, 0)
    warn_code = Keyword.get(extra, :warn_code, 0)

    Message.new!(ArmStatus, :xarm,
      state: state,
      mode: mode,
      error_code: error_code,
      warn_code: warn_code
    )
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "sends cmd_set_collision_sensitivity when sensitivity option is set" do
      expected_frame = Protocol.cmd_set_collision_sensitivity(0, 3)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      assert {:ok, state} = Collision.init(make_opts(sensitivity: 3))
      assert state.controller == :xarm
      assert state.bb.robot == TestRobot
    end

    test "sends cmd_set_collision_rebound when rebound option is set" do
      expected_frame = Protocol.cmd_set_collision_rebound(0, false)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      assert {:ok, _state} = Collision.init(make_opts(rebound: false))
    end

    test "sends cmd_set_self_collision_check when self_collision_check option is set" do
      expected_frame = Protocol.cmd_set_self_collision_check(0, true)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      assert {:ok, _state} = Collision.init(make_opts(self_collision_check: true))
    end

    test "sends all three config frames when all options are set" do
      sensitivity_frame = Protocol.cmd_set_collision_sensitivity(0, 3)
      rebound_frame = Protocol.cmd_set_collision_rebound(0, false)
      self_check_frame = Protocol.cmd_set_self_collision_check(0, true)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == sensitivity_frame
        :ok
      end)
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == rebound_frame
        :ok
      end)
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == self_check_frame
        :ok
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      opts = make_opts(sensitivity: 3, rebound: false, self_collision_check: true)
      assert {:ok, _state} = Collision.init(opts)
    end

    test "sends no config frames when no collision options are set" do
      BB.Process
      |> stub(:call, fn _robot, _controller, _msg -> flunk("should not call Process.call") end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      # No sensitivity/rebound/self_collision_check opts
      assert {:ok, _state} = Collision.init(make_opts())
    end

    test "subscribes to controller arm_status pubsub path" do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:send_command, _} -> :ok end)

      BB
      |> expect(:subscribe, fn TestRobot, [:sensor, :xarm, :arm_status] -> :ok end)

      assert {:ok, _state} = Collision.init(make_opts(sensitivity: 1))
    end

    test "completes init even when config send fails" do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:send_command, _} -> {:error, :closed} end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      assert {:ok, _state} = Collision.init(make_opts(sensitivity: 3))
    end
  end

  # ── handle_info — ArmStatus collision forwarding ─────────────────────────────

  describe "handle_info({:bb, [:sensor, _, :arm_status], msg}, state)" do
    test "re-publishes ArmStatus with error_code 22 (self-collision)" do
      state = make_state()
      msg = arm_status_msg(22)

      BB
      |> expect(:publish, fn TestRobot, [:sensor, :collision], ^msg -> :ok end)

      assert {:noreply, ^state} =
               Collision.handle_info({:bb, [:sensor, :xarm, :arm_status], msg}, state)
    end

    test "re-publishes ArmStatus with error_code 31 (collision abnormal current)" do
      state = make_state()
      msg = arm_status_msg(31)

      BB
      |> expect(:publish, fn TestRobot, [:sensor, :collision], ^msg -> :ok end)

      assert {:noreply, ^state} =
               Collision.handle_info({:bb, [:sensor, :xarm, :arm_status], msg}, state)
    end

    test "re-publishes ArmStatus with error_code 35 (safety boundary limit)" do
      state = make_state()
      msg = arm_status_msg(35)

      BB
      |> expect(:publish, fn TestRobot, [:sensor, :collision], ^msg -> :ok end)

      assert {:noreply, ^state} =
               Collision.handle_info({:bb, [:sensor, :xarm, :arm_status], msg}, state)
    end

    test "does not re-publish ArmStatus with error_code 0 (no error)" do
      state = make_state()
      msg = arm_status_msg(0)

      # No publish call expected
      assert {:noreply, ^state} =
               Collision.handle_info({:bb, [:sensor, :xarm, :arm_status], msg}, state)
    end

    test "does not re-publish ArmStatus with non-collision error codes" do
      state = make_state()

      for code <- [1, 10, 18, 21, 24, 25, 33] do
        msg = arm_status_msg(code)

        assert {:noreply, ^state} =
                 Collision.handle_info({:bb, [:sensor, :xarm, :arm_status], msg}, state)
      end
    end

    test "ignores messages on other pubsub paths" do
      state = make_state()
      msg = arm_status_msg(22)

      assert {:noreply, ^state} =
               Collision.handle_info({:bb, [:sensor, :xarm, :wrench], msg}, state)
    end

    test "ignores non-ArmStatus messages on the arm_status path" do
      state = make_state()
      other_msg = %BB.Message{timestamp: 0, frame_id: :base, payload: %{something: :else}}

      assert {:noreply, ^state} =
               Collision.handle_info({:bb, [:sensor, :xarm, :arm_status], other_msg}, state)
    end

    test "ignores completely unrelated messages" do
      state = make_state()
      assert {:noreply, ^state} = Collision.handle_info(:unexpected, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "returns :ok without sending any commands" do
      BB.Process
      |> stub(:call, fn _robot, _controller, _msg ->
        flunk("disarm should not send any commands")
      end)

      assert :ok = Collision.disarm(make_opts())
    end

    test "returns :ok regardless of opts" do
      assert :ok = Collision.disarm([])
      assert :ok = Collision.disarm(make_opts(sensitivity: 5))
    end
  end
end
