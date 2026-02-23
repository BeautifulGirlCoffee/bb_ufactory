# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Sensor.ForceTorqueTest do
  use ExUnit.Case, async: false
  use Mimic

  setup :verify_on_exit!

  alias BB.Message
  alias BB.Ufactory.Message.Wrench
  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Sensor.ForceTorque

  defp make_state do
    %{
      bb: %{robot: TestRobot, path: [:wrench]},
      controller: :xarm
    }
  end

  defp make_opts(overrides \\ []) do
    [bb: %{robot: TestRobot, path: [:wrench]}, controller: :xarm] ++ overrides
  end

  defp wrench_msg do
    Message.new!(Wrench, :xarm,
      fx: 1.0,
      fy: 2.0,
      fz: 3.0,
      tx: 0.1,
      ty: 0.2,
      tz: 0.3
    )
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "sends cmd_ft_sensor_enable(true) via controller call" do
      expected_frame = Protocol.cmd_ft_sensor_enable(0, true)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      opts = make_opts()
      assert {:ok, state} = ForceTorque.init(opts)
      assert state.controller == :xarm
      assert state.bb.robot == TestRobot
    end

    test "subscribes to controller wrench pubsub path" do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:send_command, _} -> :ok end)

      BB
      |> expect(:subscribe, fn TestRobot, [:sensor, :xarm, :wrench] -> :ok end)

      opts = make_opts()
      assert {:ok, _state} = ForceTorque.init(opts)
    end

    test "completes init even when enable command fails" do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:send_command, _} -> {:error, :closed} end)

      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      opts = make_opts()
      assert {:ok, _state} = ForceTorque.init(opts)
    end
  end

  # ── handle_info — Wrench forwarding ─────────────────────────────────────────

  describe "handle_info({:bb, [:sensor, _, :wrench], msg}, state)" do
    test "re-publishes the Wrench message at the sensor's own path" do
      state = make_state()
      msg = wrench_msg()

      BB
      |> expect(:publish, fn TestRobot, [:sensor, :wrench], ^msg -> :ok end)

      assert {:noreply, ^state} =
               ForceTorque.handle_info({:bb, [:sensor, :xarm, :wrench], msg}, state)
    end

    test "ignores messages that are not Wrench payloads" do
      state = make_state()
      other_msg = %BB.Message{timestamp: 0, frame_id: :base, payload: %{something: :else}}

      assert {:noreply, ^state} =
               ForceTorque.handle_info({:bb, [:sensor, :xarm, :wrench], other_msg}, state)
    end

    test "ignores unrelated messages" do
      state = make_state()
      assert {:noreply, ^state} = ForceTorque.handle_info(:unexpected, state)
    end

    test "ignores pubsub messages on other paths" do
      state = make_state()
      msg = wrench_msg()

      assert {:noreply, ^state} =
               ForceTorque.handle_info({:bb, [:sensor, :xarm, :tcp_pose], msg}, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "sends cmd_ft_sensor_enable(false) via controller call" do
      expected_frame = Protocol.cmd_ft_sensor_enable(0, false)

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        assert frame == expected_frame
        :ok
      end)

      opts = make_opts()
      assert :ok = ForceTorque.disarm(opts)
    end

    test "returns :ok even when enable command fails" do
      BB.Process
      |> stub(:call, fn _robot, _controller, {:send_command, _} -> {:error, :closed} end)

      opts = make_opts()
      assert :ok = ForceTorque.disarm(opts)
    end

    test "returns :ok even when controller process is unreachable" do
      BB.Process
      |> stub(:call, fn _robot, _controller, _ -> raise "boom" end)

      opts = make_opts()
      assert :ok = ForceTorque.disarm(opts)
    end
  end
end
