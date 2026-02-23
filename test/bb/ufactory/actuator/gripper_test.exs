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
      speed: Keyword.get(opts, :speed, 1500)
    }
  end

  defp position_msg(pos) do
    Message.new!(Command.Position, :gripper, position: pos * 1.0)
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "sends cmd_gripper_enable(true) via controller call on init" do
      expected_frame = Protocol.cmd_gripper_enable(0, true)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert {:ok, state} = Gripper.init(opts)
      assert state.controller == :xarm
      assert state.speed == 1500
    end

    test "stores custom speed from options" do
      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _} -> :ok end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm, speed: 800]
      assert {:ok, state} = Gripper.init(opts)
      assert state.speed == 800
    end

    test "completes init even when enable command fails" do
      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _} -> {:error, :closed} end)

      opts = [bb: %{robot: TestRobot, path: [:gripper]}, controller: :xarm]
      assert {:ok, _state} = Gripper.init(opts)
    end
  end

  # ── handle_cast position commands ────────────────────────────────────────────

  describe "handle_cast({:command, %Command.Position{}}, state)" do
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
      assert {:noreply, ^state} = Gripper.handle_cast({:command, msg}, state)
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
      Gripper.handle_cast({:command, msg}, state)
    end

    test "clamps position above 840 to 840" do
      state = make_state()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        expected = Protocol.cmd_gripper_position(0, 840)
        assert frame == expected
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(1000.0)
      Gripper.handle_cast({:command, msg}, state)
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
      Gripper.handle_cast({:command, msg}, state)
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
      Gripper.handle_cast({:command, msg}, state)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.target_position, 500.0, 0.001
    end

    test "ignores unknown casts" do
      state = make_state()
      assert {:noreply, ^state} = Gripper.handle_cast(:unexpected, state)
    end
  end

  # ── handle_info pubsub delivery ──────────────────────────────────────────────

  describe "handle_info({:bb, [:actuator | path], %Command.Position{}}, state)" do
    test "applies the same position logic as handle_cast" do
      state = make_state()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        expected = Protocol.cmd_gripper_position(0, 300)
        assert frame == expected
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(300.0)
      assert {:noreply, ^state} = Gripper.handle_info({:bb, [:actuator, :gripper], msg}, state)
    end

    test "ignores unrecognised messages" do
      state = make_state()
      assert {:noreply, ^state} = Gripper.handle_info(:unexpected, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "sends cmd_gripper_enable(false) over a fresh TCP connection" do
      {:ok, listen} = :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])
      {:ok, port} = :inet.port(listen)

      task =
        Task.async(fn ->
          {:ok, server} = :gen_tcp.accept(listen, 2_000)
          {:ok, data} = :gen_tcp.recv(server, 0, 1_000)
          data
        end)

      opts = [host: "127.0.0.1", port: port]
      assert :ok = Gripper.disarm(opts)

      received = Task.await(task, 3_000)
      expected = Protocol.cmd_gripper_enable(0, false)
      assert received == expected

      :gen_tcp.close(listen)
    end

    test "returns :ok even when TCP connection fails" do
      opts = [host: "127.0.0.1", port: 1]
      assert :ok = Gripper.disarm(opts)
    end
  end
end
