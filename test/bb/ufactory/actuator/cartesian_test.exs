# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.CartesianTest do
  use ExUnit.Case, async: false
  use Mimic

  setup :verify_on_exit!

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Ufactory.Actuator.Cartesian
  alias BB.Ufactory.Protocol

  defp make_state(opts \\ []) do
    %{
      bb: %{robot: TestRobot, path: [:cartesian]},
      controller: :xarm,
      speed: Keyword.get(opts, :speed, 100.0),
      acceleration: Keyword.get(opts, :acceleration, 2000.0)
    }
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "stores bb, controller, speed, and acceleration in state" do
      opts = [
        bb: %{robot: TestRobot, path: [:cartesian]},
        controller: :xarm,
        speed: 150.0,
        acceleration: 3000.0
      ]

      assert {:ok, state} = Cartesian.init(opts)
      assert state.bb == %{robot: TestRobot, path: [:cartesian]}
      assert state.controller == :xarm
      assert state.speed == 150.0
      assert state.acceleration == 3000.0
    end

    test "uses default speed and acceleration when not provided" do
      opts = [bb: %{robot: TestRobot, path: [:cartesian]}, controller: :xarm]

      assert {:ok, state} = Cartesian.init(opts)
      assert state.speed == 100.0
      assert state.acceleration == 2000.0
    end
  end

  # ── handle_cast {:move_cartesian, pose} ──────────────────────────────────────

  describe "handle_cast({:move_cartesian, pose}, state)" do
    test "sends cmd_move_cartesian frame via controller call" do
      state = make_state()
      pose = {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}
      expected_frame = Protocol.cmd_move_cartesian(0, pose, 100.0, 2000.0)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, ^state} = Cartesian.handle_cast({:move_cartesian, pose}, state)
    end

    test "publishes BeginMotion after sending the command" do
      state = make_state()
      pose = {300.0, 0.0, 0.0, 0.0, 0.0, 0.0}
      test_pid = self()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> :ok end)

      BB
      |> expect(:publish, fn TestRobot,
                             [:actuator | _path],
                             %Message{
                               payload: %BeginMotion{}
                             } = msg ->
        send(test_pid, {:published, msg})
        :ok
      end)

      Cartesian.handle_cast({:move_cartesian, pose}, state)
      assert_receive {:published, _msg}, 500
    end

    test "uses per-command speed and acceleration when provided as 4-tuple" do
      state = make_state()
      pose = {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}
      expected_frame = Protocol.cmd_move_cartesian(0, pose, 50.0, 1000.0)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, ^state} =
               Cartesian.handle_cast({:move_cartesian, pose, 50.0, 1000.0}, state)
    end

    test "ignores unknown casts" do
      state = make_state()
      assert {:noreply, ^state} = Cartesian.handle_cast(:unexpected, state)
    end

    test "does not publish BeginMotion when controller call fails" do
      state = make_state()
      pose = {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} ->
        {:error, :closed}
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg ->
        flunk("publish should not be called on controller error")
      end)

      assert {:noreply, ^state} = Cartesian.handle_cast({:move_cartesian, pose}, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "returns :ok" do
      assert :ok = Cartesian.disarm([])
    end
  end
end
