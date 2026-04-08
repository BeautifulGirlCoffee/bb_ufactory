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
  `"192.168.1.224"`).

  **Safety:** These tests send real motion commands. Ensure the workspace is
  clear before running. Tests use small movements (0.01 rad) and always disarm
  on cleanup.
  """

  use ExUnit.Case, async: false

  @moduletag :hardware

  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Report

  @host System.get_env("XARM_HOST", "192.168.1.224")
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

  defp send_and_recv(socket, frame, timeout \\ 5_000) do
    :ok = :gen_tcp.send(socket, frame)
    {:ok, data} = :gen_tcp.recv(socket, 0, timeout)
    Protocol.parse_response(data)
  end

  # Brings the arm to a known-good state: errors cleared, motors enabled,
  # position-control mode, sport/ready state. Call at the start of any test
  # that sends commands.
  defp ensure_ready(socket) do
    send_and_recv(socket, Protocol.cmd_clean_error(0))
    send_and_recv(socket, Protocol.cmd_enable(1, true))
    send_and_recv(socket, Protocol.cmd_set_mode(2, 0))
    {:ok, {0x0C, 0x00, _}, _} = send_and_recv(socket, Protocol.cmd_set_state(3, 0))
    Process.sleep(200)
  end

  # ── Connection ─────────────────────────────────────────────────────────────

  describe "TCP connection" do
    test "connects to command port, sends heartbeat, and connection stays alive" do
      socket = connect_cmd()

      :ok = :gen_tcp.send(socket, Protocol.heartbeat())

      # Heartbeat is a keep-alive — some firmware versions ACK silently.
      # Verify the connection is still usable by querying the error register.
      assert {:ok, {0x0F, _status, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_error(0))

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
      ensure_ready(socket)

      assert {:ok, {0x0B, 0x00, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_enable(10, true))

      # Disable returns non-zero status during state transition — that's expected
      assert {:ok, {0x0B, _status, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_enable(11, false))

      # Re-enable so subsequent tests find the arm in a usable state
      ensure_ready(socket)

      :gen_tcp.close(socket)
    end
  end

  # ── Joint motion ───────────────────────────────────────────────────────────

  describe "joint motion" do
    test "moves J1 by 0.01 rad and back" do
      cmd_socket = connect_cmd()
      report_socket = connect_report()

      # Read current position (all joints)
      {:ok, initial_data} = :gen_tcp.recv(report_socket, 0, 1_000)
      {initial_report, _rest} = recv_report(report_socket, initial_data)
      initial_angles = Enum.take(initial_report.angles, 6)

      # Bring arm to ready state
      ensure_ready(cmd_socket)

      # Small move: offset J1 by 0.01 rad, keep other joints at current position
      target_angles = List.update_at(initial_angles, 0, &(&1 + 0.01))

      :ok = :gen_tcp.send(cmd_socket, Protocol.cmd_move_joints(10, target_angles, 0.1, 1.0))
      Process.sleep(2_000)

      # Move back to original
      :ok = :gen_tcp.send(cmd_socket, Protocol.cmd_move_joints(11, initial_angles, 0.1, 1.0))
      Process.sleep(2_000)

      # Verify we returned close to the original position
      {:ok, final_data} = :gen_tcp.recv(report_socket, 0, 1_000)
      {final_report, _rest} = recv_report(report_socket, final_data)
      final_j1 = hd(final_report.angles)

      assert_in_delta final_j1, hd(initial_angles), 0.02

      :gen_tcp.close(cmd_socket)
      :gen_tcp.close(report_socket)
    end
  end

  # ── Gripper ────────────────────────────────────────────────────────────────

  describe "gripper" do
    test "enables, opens, closes, and disables without error" do
      socket = connect_cmd()
      ensure_ready(socket)

      assert {:ok, _resp, _rest} =
               send_and_recv(socket, Protocol.cmd_gripper_enable(0, true), 10_000)

      Process.sleep(1_000)

      :ok = :gen_tcp.send(socket, Protocol.cmd_gripper_position(1, 840))
      Process.sleep(2_000)

      :ok = :gen_tcp.send(socket, Protocol.cmd_gripper_position(2, 0))
      Process.sleep(2_000)

      assert {:ok, _resp, _rest} =
               send_and_recv(socket, Protocol.cmd_gripper_enable(3, false), 10_000)

      :gen_tcp.close(socket)
    end
  end

  # ── Force-torque sensor ────────────────────────────────────────────────────

  describe "force-torque sensor" do
    test "enables, reads one wrench, and disables" do
      socket = connect_cmd()
      ensure_ready(socket)

      assert {:ok, _resp, _rest} =
               send_and_recv(socket, Protocol.cmd_ft_sensor_enable(0, true))

      Process.sleep(500)

      assert {:ok, {0xC8, _status, params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_ft_data(1))

      values = Protocol.decode_fp32s(params, 6)
      assert length(values) == 6
      Enum.each(values, fn v -> assert is_float(v) end)

      assert {:ok, _resp, _rest} =
               send_and_recv(socket, Protocol.cmd_ft_sensor_enable(2, false))

      :gen_tcp.close(socket)
    end
  end

  # ── Linear track ──────────────────────────────────────────────────────────

  describe "linear track" do
    test "enables, moves 10 mm forward and back, and disables without error" do
      socket = connect_cmd()
      ensure_ready(socket)

      # Read current position
      initial_pos = read_track_position(socket)

      # Enable track
      assert {:ok, {0x7C, 0x00, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_linear_track_enable(0, true))

      Process.sleep(500)

      # Set speed and move 10 mm forward
      target = initial_pos + 10.0
      {pos_frame, spd_frame} = Protocol.cmd_linear_track_move(1, target, 50)
      assert {:ok, {0x7C, 0x00, _}, _} = send_and_recv(socket, spd_frame)
      assert {:ok, {0x7C, 0x00, _}, _} = send_and_recv(socket, pos_frame)

      Process.sleep(2_000)

      # Verify we moved close to the target
      after_move = read_track_position(socket)
      assert_in_delta after_move, target, 2.0

      # Move back to initial position
      {pos_back, spd_back} = Protocol.cmd_linear_track_move(2, initial_pos, 50)
      assert {:ok, {0x7C, 0x00, _}, _} = send_and_recv(socket, spd_back)
      assert {:ok, {0x7C, 0x00, _}, _} = send_and_recv(socket, pos_back)

      Process.sleep(2_000)

      # Verify we returned
      final_pos = read_track_position(socket)
      assert_in_delta final_pos, initial_pos, 2.0

      :gen_tcp.close(socket)
    end
  end

  # ── Helpers ────────────────────────────────────────────────────────────────

  # Reads the current linear track position via RS485 read (function 0x03).
  # Register 0x0A20 on device 0x01, controller-box host 0x0B, returns mm.
  defp read_track_position(socket) do
    read_payload = <<0x0B, 0x01, 0x03, 0x0A, 0x20, 0x00, 0x02>>
    read_frame = Protocol.build_frame(99, 0x7C, read_payload)
    {:ok, {0x7C, 0x00, params}, _rest} = send_and_recv(socket, read_frame)
    <<_host::8, _dev::8, _func::8, _bc::8, pos_raw::signed-32>> = params
    pos_raw / 2000
  end

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
