# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.GripperTest do
  use ExUnit.Case, async: false
  use Mimic

  setup :verify_on_exit!

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.Ufactory.Actuator.Gripper
  alias BB.Ufactory.Protocol

  defp make_state(opts \\ []) do
    %{
      bb: %{robot: TestRobot, path: [:gripper]},
      controller: :xarm,
      speed: Keyword.get(opts, :speed, 1500),
      last_commanded: Keyword.get(opts, :last_commanded)
    }
  end

  defp position_msg(pos) do
    Message.new!(Command.Position, :gripper, position: pos * 1.0)
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "stores controller and speed in state without sending hardware commands" do
      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:register_arm_frames, :gripper, _frames} -> :ok end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert {:ok, state} = Gripper.init(opts)
      assert state.controller == :xarm
      assert state.speed == 1500
    end

    test "stores custom speed from options" do
      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:register_arm_frames, :gripper, _frames} -> :ok end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm, speed: 800]
      assert {:ok, state} = Gripper.init(opts)
      assert state.speed == 800
    end

    test "does not subscribe to its own command topic (BB.Actuator.Server owns it)" do
      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:register_arm_frames, :gripper, _frames} -> :ok end)

      BB
      |> reject(:subscribe, 2)
      |> reject(:subscribe, 3)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert {:ok, _state} = Gripper.init(opts)
    end

    test "registers enable + speed frames with the controller's arm sequence" do
      expected_enable = Protocol.cmd_gripper_enable(0, true)
      expected_speed = Protocol.cmd_gripper_speed(0, 800)
      test_pid = self()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:register_arm_frames, :gripper, frames} ->
        send(test_pid, {:registered, frames})
        :ok
      end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm, speed: 800]
      assert {:ok, _state} = Gripper.init(opts)

      assert_receive {:registered, [^expected_enable, ^expected_speed]}, 500
    end

    test "completes init even when arm-frame registration fails" do
      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:register_arm_frames, :gripper, _} ->
        {:error, :closed}
      end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert {:ok, _state} = Gripper.init(opts)
    end
  end

  # ── command_payloads/1 ───────────────────────────────────────────────────────

  describe "command_payloads/1" do
    test "declares only Position (no genuine gripper stop on the RS485 proxy)" do
      assert Gripper.command_payloads([]) == [Command.Position]
    end
  end

  # ── handle_command: Command.Position ─────────────────────────────────────────

  describe "handle_command(%Command.Position{}, state)" do
    test "sends cmd_gripper_position frame with rounded integer position" do
      state = make_state()
      expected_frame = Protocol.cmd_gripper_position(0, 420)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(420.0)
      assert {:noreply, new_state} = Gripper.handle_command(msg, state)
      assert new_state.last_commanded == 420
    end

    test "BeginMotion uses the last commanded position as initial estimate" do
      state = make_state(last_commanded: 700)
      test_pid = self()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> :ok end)

      BB
      |> expect(:publish, fn TestRobot, [:actuator, :gripper], msg ->
        send(test_pid, {:begin_motion, msg.payload})
        :ok
      end)

      assert {:noreply, _new_state} = Gripper.handle_command(position_msg(100.0), state)

      assert_receive {:begin_motion, bm}
      assert bm.initial_position == 700.0
      assert bm.target_position == 100.0
    end

    test "rounds float position to nearest integer" do
      state = make_state()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        expected = Protocol.cmd_gripper_position(0, 421)
        assert frame == expected
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(420.7)
      Gripper.handle_command(msg, state)
    end

    test "clamps position above 850 to 850" do
      state = make_state()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        expected = Protocol.cmd_gripper_position(0, 850)
        assert frame == expected
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(1000.0)
      Gripper.handle_command(msg, state)
    end

    test "clamps position below 0 to 0" do
      state = make_state()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        expected = Protocol.cmd_gripper_position(0, 0)
        assert frame == expected
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(-50.0)
      Gripper.handle_command(msg, state)
    end

    test "publishes BeginMotion with correct target_position" do
      state = make_state()
      test_pid = self()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> :ok end)

      BB
      |> expect(:publish, fn TestRobot,
                             [:actuator | _path],
                             %Message{
                               payload: %BeginMotion{} = bm
                             } ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = position_msg(500.0)
      Gripper.handle_command(msg, state)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.target_position, 500.0, 0.001
    end

    test "replies with the error when the controller cannot deliver the frame" do
      state = make_state()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, _frame} ->
        {:error, :closed}
      end)

      msg = position_msg(300.0)
      assert {:reply, {:error, :closed}, ^state} = Gripper.handle_command(msg, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "sends cmd_gripper_enable(false) via controller call" do
      expected_frame = Protocol.cmd_gripper_enable(0, false)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert :ok = Gripper.disarm(opts)
    end

    test "returns :ok even when controller call fails" do
      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, _frame} ->
        {:error, :noproc}
      end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert :ok = Gripper.disarm(opts)
    end

    test "returns :ok even when controller process is down" do
      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, _frame} ->
        raise "process down"
      end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert :ok = Gripper.disarm(opts)
    end
  end
end
