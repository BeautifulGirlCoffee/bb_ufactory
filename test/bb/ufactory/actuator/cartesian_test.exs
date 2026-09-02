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
  alias BB.Ufactory.Message.Command.CartesianMove
  alias BB.Ufactory.Protocol

  defp make_state(opts \\ []) do
    ets = Keyword.get_lazy(opts, :ets, fn -> make_ets() end)

    %{
      bb: %{robot: TestRobot, path: [:cartesian]},
      controller: :xarm,
      speed: Keyword.get(opts, :speed, 100.0),
      acceleration: Keyword.get(opts, :acceleration, 2000.0),
      ets: ets
    }
  end

  defp make_ets(pose \\ nil) do
    ets = :ets.new(:test_cartesian_ets, [:public, :set])
    :ets.insert(ets, {:arm, 0, 0, pose})
    ets
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "stores bb, controller, speed, acceleration, and ets in state" do
      ets = make_ets()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, :get_ets -> ets end)

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
      assert state.ets == ets
    end

    test "uses default speed and acceleration when not provided" do
      BB.Process
      |> expect(:call, fn TestRobot, :xarm, :get_ets -> nil end)

      opts = [bb: %{robot: TestRobot, path: [:cartesian]}, controller: :xarm]

      assert {:ok, state} = Cartesian.init(opts)
      assert state.speed == 100.0
      assert state.acceleration == 2000.0
    end
  end

  # ── command_payloads/1 ───────────────────────────────────────────────────────

  describe "command_payloads/1" do
    test "declares only CartesianMove (no scoped stop exists for MOVE_LINE)" do
      assert Cartesian.command_payloads([]) == [CartesianMove]
    end
  end

  # ── handle_command: CartesianMove ────────────────────────────────────────────

  describe "handle_command(%CartesianMove{}, state)" do
    defp cartesian_move_msg(pose, opts \\ []) do
      {x, y, z, roll, pitch, yaw} = pose

      Message.new!(
        CartesianMove,
        :cartesian,
        [x: x, y: y, z: z, roll: roll, pitch: pitch, yaw: yaw] ++ opts
      )
    end

    test "sends cmd_move_cartesian with the configured default speed/acceleration" do
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

      msg = cartesian_move_msg(pose)
      assert {:noreply, ^state} = Cartesian.handle_command(msg, state)
    end

    test "accepts integer pose fields and normalizes them to floats" do
      state = make_state()
      # `x: 300` is as natural as `x: 300.0` — the schema takes both, and
      # the encoder must receive floats.
      expected_frame =
        Protocol.cmd_move_cartesian(0, {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}, 80.0, 2000.0)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg =
        Message.new!(CartesianMove, :cartesian,
          x: 300,
          y: 0,
          z: 200,
          roll: 0,
          pitch: 0,
          yaw: 0,
          speed: 80
        )

      assert {:noreply, ^state} = Cartesian.handle_command(msg, state)
    end

    test "uses per-command speed and acceleration from the payload" do
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

      msg = cartesian_move_msg(pose, speed: 50.0, acceleration: 1000.0)
      assert {:noreply, ^state} = Cartesian.handle_command(msg, state)
    end

    test "publishes BeginMotion after sending the command" do
      state = make_state()
      test_pid = self()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> :ok end)

      BB
      |> expect(:publish, fn TestRobot, [:actuator | _], %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = cartesian_move_msg({300.0, 0.0, 0.0, 0.0, 0.0, 0.0})
      Cartesian.handle_command(msg, state)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.target_position, 0.3, 1.0e-6
    end

    test "replies with the error when the controller cannot deliver the frame" do
      state = make_state()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> {:error, :closed} end)

      BB
      |> stub(:publish, fn _robot, _path, _msg ->
        flunk("publish should not be called on controller error")
      end)

      msg = cartesian_move_msg({300.0, 0.0, 200.0, 0.0, 0.0, 0.0})
      assert {:reply, {:error, :closed}, ^state} = Cartesian.handle_command(msg, state)
    end
  end

  # ── handle_cast {:move_cartesian, pose} (legacy interface) ───────────────────

  describe "handle_cast({:move_cartesian, pose}, state)" do
    setup do
      # The legacy cast path enforces the armed check itself (bb's command
      # pipeline never sees casts); these tests exercise the armed case.
      BB.Safety
      |> stub(:armed?, fn TestRobot -> true end)

      :ok
    end

    test "drops the cast when the robot is not armed" do
      BB.Safety
      |> expect(:armed?, fn TestRobot -> false end)

      # No :send_command expectation: reaching the controller would fail the
      # Mimic verification.
      state = make_state()
      pose = {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}

      assert {:noreply, ^state} = Cartesian.handle_cast({:move_cartesian, pose}, state)
    end

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

    test "BeginMotion target_position is the travel distance in metres" do
      state = make_state()
      # No ETS → current position falls back to {0,0,0}; distance = 300 mm.
      pose = {300.0, 0.0, 0.0, 0.0, 0.0, 0.0}
      test_pid = self()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> :ok end)

      BB
      |> expect(:publish, fn TestRobot, [:actuator | _], %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      Cartesian.handle_cast({:move_cartesian, pose}, state)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.target_position, 0.3, 1.0e-6
      assert bm.initial_position == 0.0
    end

    test "BeginMotion expected_arrival uses the per-command speed override" do
      state = make_state()
      # 300 mm at the 10 mm/s override → 30 s travel, not 3 s at the
      # configured default of 100 mm/s.
      pose = {300.0, 0.0, 0.0, 0.0, 0.0, 0.0}
      test_pid = self()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, {:send_command, _frame} -> :ok end)

      BB
      |> expect(:publish, fn TestRobot, [:actuator | _], %Message{payload: %BeginMotion{} = bm} ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      before = System.monotonic_time(:millisecond)
      Cartesian.handle_cast({:move_cartesian, pose, 10.0, 1000.0}, state)

      assert_receive {:begin_motion, bm}, 500
      travel_ms = bm.expected_arrival - before
      assert travel_ms >= 29_000
      assert travel_ms <= 31_000
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
