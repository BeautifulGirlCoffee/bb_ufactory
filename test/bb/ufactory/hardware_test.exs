# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.HardwareTest do
  @moduledoc """
  Hardware integration tests for a physical xArm6.

  These tests require a real xArm connected on the network. They are excluded
  from the default test suite and must be run explicitly:

      mix test --include hardware

  Configure the arm IP via the `XARM_HOST` environment variable (defaults to
  `"192.168.1.111"`).

  **Safety:** These tests send real motion commands. Ensure the workspace is
  clear before running. Tests use small movements (0.01 rad) and always disarm
  on cleanup.
  """

  use ExUnit.Case, async: false

  @moduletag :hardware

  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Report

  @host System.get_env("XARM_HOST", "192.168.1.111")
  @cmd_port 502
  @report_port 30_003

  defp connect_cmd do
    {:ok, socket} =
      :gen_tcp.connect(
        String.to_charlist(@host),
        @cmd_port,
        [:binary, active: false, packet: :raw],
        5_000
      )

    socket
  end

  defp connect_report do
    {:ok, socket} =
      :gen_tcp.connect(
        String.to_charlist(@host),
        @report_port,
        [:binary, active: false, packet: :raw],
        5_000
      )

    socket
  end

  defp send_and_recv(socket, frame) do
    :ok = :gen_tcp.send(socket, frame)
    {:ok, data} = :gen_tcp.recv(socket, 0, 2_000)
    Protocol.parse_response(data)
  end

  # ── Connection ─────────────────────────────────────────────────────────────

  describe "TCP connection" do
    test "connects to command port and receives heartbeat response" do
      socket = connect_cmd()

      :ok = :gen_tcp.send(socket, Protocol.heartbeat())
      assert {:ok, _data} = :gen_tcp.recv(socket, 0, 2_000)

      :gen_tcp.close(socket)
    end

    test "connects to report port and receives data within 1 second" do
      socket = connect_report()

      assert {:ok, data} = :gen_tcp.recv(socket, 0, 1_000)
      assert byte_size(data) > 0

      :gen_tcp.close(socket)
    end
  end

  # ── State query ────────────────────────────────────────────────────────────

  describe "state query" do
    test "reads current joint positions — 6 floats in valid range" do
      report_socket = connect_report()
      {:ok, data} = :gen_tcp.recv(report_socket, 0, 1_000)

      # Accumulate until we have a full frame
      {report, _rest} = recv_report(report_socket, data)
      angles = Enum.take(report.angles, 6)

      assert length(angles) == 6

      Enum.each(angles, fn angle ->
        assert is_float(angle)
        assert angle >= -2 * :math.pi() and angle <= 2 * :math.pi()
      end)

      :gen_tcp.close(report_socket)
    end
  end

  # ── Enable/disable cycle ───────────────────────────────────────────────────

  describe "enable/disable cycle" do
    test "enables motors and then disables without error" do
      socket = connect_cmd()

      assert {:ok, {0x0B, 0x00, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_enable(0, true))

      assert {:ok, {0x0B, 0x00, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_enable(1, false))

      :gen_tcp.close(socket)
    end
  end

  # ── Joint motion ───────────────────────────────────────────────────────────

  describe "joint motion" do
    test "moves J1 by 0.01 rad and back" do
      cmd_socket = connect_cmd()
      report_socket = connect_report()

      # Read current position
      {:ok, initial_data} = :gen_tcp.recv(report_socket, 0, 1_000)
      {initial_report, _rest} = recv_report(report_socket, initial_data)
      initial_j1 = hd(initial_report.angles)

      # Enable and set to motion-ready
      send_and_recv(cmd_socket, Protocol.cmd_enable(0, true))
      send_and_recv(cmd_socket, Protocol.cmd_set_state(1, 3))

      # Small move
      target = initial_j1 + 0.01
      angles = [target | List.duplicate(0.0, 5)]
      :ok = :gen_tcp.send(cmd_socket, Protocol.cmd_move_joints(2, angles, 0.1, 1.0))

      Process.sleep(2_000)

      # Move back
      angles_back = [initial_j1 | List.duplicate(0.0, 5)]
      :ok = :gen_tcp.send(cmd_socket, Protocol.cmd_move_joints(3, angles_back, 0.1, 1.0))

      Process.sleep(2_000)

      # Disable
      send_and_recv(cmd_socket, Protocol.cmd_enable(4, false))

      :gen_tcp.close(cmd_socket)
      :gen_tcp.close(report_socket)
    end
  end

  # ── Gripper ────────────────────────────────────────────────────────────────

  describe "gripper" do
    test "enables, opens, closes, and disables without error" do
      socket = connect_cmd()

      assert {:ok, _resp, _rest} = send_and_recv(socket, Protocol.cmd_gripper_enable(0, true))

      Process.sleep(500)

      :ok = :gen_tcp.send(socket, Protocol.cmd_gripper_position(1, 840))
      Process.sleep(1_000)

      :ok = :gen_tcp.send(socket, Protocol.cmd_gripper_position(2, 0))
      Process.sleep(1_000)

      assert {:ok, _resp, _rest} = send_and_recv(socket, Protocol.cmd_gripper_enable(3, false))

      :gen_tcp.close(socket)
    end
  end

  # ── Force-torque sensor ────────────────────────────────────────────────────

  describe "force-torque sensor" do
    test "enables, reads one wrench, and disables" do
      socket = connect_cmd()

      assert {:ok, _resp, _rest} =
               send_and_recv(socket, Protocol.cmd_ft_sensor_enable(0, true))

      Process.sleep(200)

      assert {:ok, {0xC8, 0x00, params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_ft_data(1))

      values = Protocol.decode_fp32s(params, 6)
      assert length(values) == 6
      Enum.each(values, fn v -> assert is_float(v) end)

      assert {:ok, _resp, _rest} =
               send_and_recv(socket, Protocol.cmd_ft_sensor_enable(2, false))

      :gen_tcp.close(socket)
    end
  end

  # ── Helpers ────────────────────────────────────────────────────────────────

  defp recv_report(socket, buffer) do
    case Report.parse_report(buffer) do
      {:ok, report, rest} ->
        {report, rest}

      {:more} ->
        {:ok, more_data} = :gen_tcp.recv(socket, 0, 1_000)
        recv_report(socket, buffer <> more_data)
    end
  end
end
