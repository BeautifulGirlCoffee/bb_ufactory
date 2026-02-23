# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ProtocolTest do
  use ExUnit.Case, async: true

  alias BB.Ufactory.Protocol

  # ── heartbeat ───────────────────────────────────────────────────────────────

  describe "heartbeat/0" do
    test "is exactly the 8-byte SDK heartbeat sequence" do
      assert Protocol.heartbeat() == <<0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00>>
    end

    test "is 8 bytes" do
      assert byte_size(Protocol.heartbeat()) == 8
    end
  end

  # ── encode_fp32 / decode_fp32 ───────────────────────────────────────────────

  describe "encode_fp32/1 and decode_fp32/1" do
    test "round-trips 0.0" do
      assert Protocol.decode_fp32(Protocol.encode_fp32(0.0)) == 0.0
    end

    test "round-trips 1.0" do
      assert Protocol.decode_fp32(Protocol.encode_fp32(1.0)) == 1.0
    end

    test "round-trips -1.0" do
      assert Protocol.decode_fp32(Protocol.encode_fp32(-1.0)) == -1.0
    end

    test "round-trips pi" do
      pi = :math.pi()
      assert_in_delta Protocol.decode_fp32(Protocol.encode_fp32(pi)), pi, 1.0e-6
    end

    test "round-trips a negative Cartesian coordinate" do
      assert_in_delta Protocol.decode_fp32(Protocol.encode_fp32(-350.123)), -350.123, 0.001
    end

    test "encode_fp32 produces 4 bytes" do
      assert byte_size(Protocol.encode_fp32(42.0)) == 4
    end

    test "encode_fp32 is little-endian (1.0 = 0x3F800000 LE = <<0, 0, 128, 63>>)" do
      assert Protocol.encode_fp32(1.0) == <<0x00, 0x00, 0x80, 0x3F>>
    end

    test "round-trips very small positive float (1.0e-38)" do
      val = 1.0e-38
      assert_in_delta Protocol.decode_fp32(Protocol.encode_fp32(val)), val, 1.0e-44
    end

    test "round-trips very large positive float (1.0e+38)" do
      val = 1.0e+38
      assert_in_delta Protocol.decode_fp32(Protocol.encode_fp32(val)), val, 1.0e+32
    end

    test "round-trips negative zero" do
      encoded = Protocol.encode_fp32(-0.0)
      decoded = Protocol.decode_fp32(encoded)
      assert decoded == 0.0 or decoded == -0.0
    end
  end

  # ── decode_fp32s ────────────────────────────────────────────────────────────

  describe "decode_fp32s/2" do
    test "decodes a sequence of floats" do
      bin = Protocol.encode_fp32s([1.0, 2.0, 3.0])
      assert Protocol.decode_fp32s(bin, 3) == [1.0, 2.0, 3.0]
    end

    test "decodes zero floats from empty binary" do
      assert Protocol.decode_fp32s(<<>>, 0) == []
    end

    test "decodes only the first n floats from a longer binary" do
      bin = Protocol.encode_fp32s([1.0, 2.0, 3.0, 4.0])
      assert Protocol.decode_fp32s(bin, 2) == [1.0, 2.0]
    end
  end

  # ── build_frame ─────────────────────────────────────────────────────────────

  describe "build_frame/3" do
    test "produces correct 6-byte header for a frame with no params" do
      frame = Protocol.build_frame(1, 0x0C, <<>>)
      # header: txn_id=1, prot=0x0002, length=1 (register only), register=0x0C
      assert frame == <<0x00, 0x01, 0x00, 0x02, 0x00, 0x01, 0x0C>>
    end

    test "length field equals params_size + 1 (for register byte)" do
      params = Protocol.encode_fp32(42.0)
      frame = Protocol.build_frame(0, 0x17, params)
      <<_txn::16, _prot::16, length::16, _register, _params::binary>> = frame
      assert length == byte_size(params) + 1
    end

    test "transaction ID is big-endian in bytes 0–1" do
      frame = Protocol.build_frame(0x1234, 0x0B, <<>>)
      <<txn::16, _::binary>> = frame
      assert txn == 0x1234
    end

    test "protocol ID is always 0x0002 in bytes 2–3" do
      frame = Protocol.build_frame(0, 0x15, <<>>)
      <<_::16, prot::16, _::binary>> = frame
      assert prot == 0x0002
    end

    test "register is at byte 6" do
      frame = Protocol.build_frame(0, 0xAB, <<>>)
      assert binary_part(frame, 6, 1) == <<0xAB>>
    end

    test "params follow the register byte" do
      params = <<0x01, 0x02, 0x03>>
      frame = Protocol.build_frame(0, 0x0B, params)
      assert binary_part(frame, 7, 3) == params
    end
  end

  # ── parse_response ──────────────────────────────────────────────────────────

  describe "parse_response/1" do
    test "returns {:more} for empty binary" do
      assert Protocol.parse_response(<<>>) == {:more}
    end

    test "returns {:more} for 5-byte partial header" do
      assert Protocol.parse_response(<<0, 1, 0, 2, 0>>) == {:more}
    end

    test "returns {:more} when body is incomplete" do
      # length = 5 (register + 4 bytes data), but only register + 1 byte present
      header = <<0x00, 0x01, 0x00, 0x02, 0x00, 0x05>>
      partial = header <> <<0x0C, 0x00, 0xAA>>
      assert Protocol.parse_response(partial) == {:more}
    end

    test "parses a minimal response (status-only, length=2)" do
      # Transaction ID=1, Protocol=0x0002, Length=2, Register=0x0C, Status=0x00
      response = <<0x00, 0x01, 0x00, 0x02, 0x00, 0x02, 0x0C, 0x00>>
      assert {:ok, {0x0C, 0x00, <<>>}, <<>>} = Protocol.parse_response(response)
    end

    test "parses a response with data params" do
      param_bytes = Protocol.encode_fp32s([1.0, 2.0])
      # length = 2 (register + status) + byte_size(param_bytes)
      length = 2 + byte_size(param_bytes)
      response = <<0x00, 0x01, 0x00, 0x02, length::16, 0x2A, 0x00>> <> param_bytes
      assert {:ok, {0x2A, 0x00, ^param_bytes}, <<>>} = Protocol.parse_response(response)
    end

    test "returns rest bytes after a complete frame" do
      response = <<0x00, 0x01, 0x00, 0x02, 0x00, 0x02, 0x0C, 0x00, 0xFF, 0xFF>>
      assert {:ok, _, <<0xFF, 0xFF>>} = Protocol.parse_response(response)
    end

    test "returns {:error, {:bad_protocol_id, id}} for wrong protocol" do
      bad = <<0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x0C, 0x00>>
      assert {:error, {:bad_protocol_id, 0x0000}} = Protocol.parse_response(bad)
    end

    test "parses a response with non-zero status byte" do
      response = <<0x00, 0x01, 0x00, 0x02, 0x00, 0x02, 0x0B, 0x01>>
      assert {:ok, {0x0B, 0x01, <<>>}, <<>>} = Protocol.parse_response(response)
    end

    test "returns {:error, ...} for standard Modbus protocol ID 0x0000" do
      bad = <<0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x0C, 0x00>>
      assert {:error, {:bad_protocol_id, 0x0000}} = Protocol.parse_response(bad)
    end

    test "returns {:more} for exactly 6 bytes (header only, no body)" do
      assert Protocol.parse_response(<<0x00, 0x01, 0x00, 0x02, 0x00, 0x02>>) == {:more}
    end
  end

  # ── cmd_enable ──────────────────────────────────────────────────────────────

  describe "cmd_enable/2" do
    test "produces a 9-byte frame" do
      assert byte_size(Protocol.cmd_enable(0, true)) == 9
    end

    test "uses register 0x0B" do
      frame = Protocol.cmd_enable(0, true)
      assert binary_part(frame, 6, 1) == <<0x0B>>
    end

    test "axis_id byte is 0x08 (all joints)" do
      frame = Protocol.cmd_enable(0, true)
      assert binary_part(frame, 7, 1) == <<0x08>>
    end

    test "enable byte is 1 when enabling" do
      frame = Protocol.cmd_enable(0, true)
      assert binary_part(frame, 8, 1) == <<0x01>>
    end

    test "enable byte is 0 when disabling" do
      frame = Protocol.cmd_enable(0, false)
      assert binary_part(frame, 8, 1) == <<0x00>>
    end
  end

  # ── cmd_set_state / cmd_stop ─────────────────────────────────────────────────

  describe "cmd_set_state/2" do
    test "produces an 8-byte frame" do
      assert byte_size(Protocol.cmd_set_state(0, 3)) == 8
    end

    test "uses register 0x0C" do
      frame = Protocol.cmd_set_state(0, 3)
      assert binary_part(frame, 6, 1) == <<0x0C>>
    end

    test "encodes the state value at byte 7" do
      frame = Protocol.cmd_set_state(0, 3)
      assert binary_part(frame, 7, 1) == <<0x03>>
    end
  end

  describe "cmd_stop/1" do
    test "is equivalent to cmd_set_state with value 0" do
      assert Protocol.cmd_stop(1) == Protocol.cmd_set_state(1, 0)
    end
  end

  # ── cmd_get_joint_angles / cmd_get_cartesian_pose ───────────────────────────

  describe "cmd_get_joint_angles/1" do
    test "produces a 7-byte frame (header + register, no params)" do
      assert byte_size(Protocol.cmd_get_joint_angles(0)) == 7
    end

    test "uses register 0x2A" do
      frame = Protocol.cmd_get_joint_angles(0)
      assert binary_part(frame, 6, 1) == <<0x2A>>
    end
  end

  describe "cmd_get_cartesian_pose/1" do
    test "produces a 7-byte frame" do
      assert byte_size(Protocol.cmd_get_cartesian_pose(0)) == 7
    end

    test "uses register 0x29" do
      frame = Protocol.cmd_get_cartesian_pose(0)
      assert binary_part(frame, 6, 1) == <<0x29>>
    end
  end

  # ── cmd_move_joints ──────────────────────────────────────────────────────────

  describe "cmd_move_joints/4" do
    test "produces a 47-byte frame (7 header + 10×4 param bytes)" do
      angles = [0.0, 0.5, -0.5, 0.0, 1.0, 0.0]
      frame = Protocol.cmd_move_joints(0, angles, 1.0, 10.0)
      # 6 header + 1 register + 10 × 4 bytes params = 47
      assert byte_size(frame) == 47
    end

    test "uses register 0x17" do
      frame = Protocol.cmd_move_joints(0, [0.0], 1.0, 10.0)
      assert binary_part(frame, 6, 1) == <<0x17>>
    end

    test "pads a short angle list with 0.0 to 7 joints" do
      # Send only 3 angles; joints 4–7 should be 0.0
      frame = Protocol.cmd_move_joints(0, [1.0, 2.0, 3.0], 0.5, 5.0)
      # Parse the params (skip 7-byte header)
      params = binary_part(frame, 7, 40)
      floats = Protocol.decode_fp32s(params, 10)

      [j1, j2, j3, j4, j5, j6, j7 | _] = floats
      assert_in_delta j1, 1.0, 1.0e-6
      assert_in_delta j2, 2.0, 1.0e-6
      assert_in_delta j3, 3.0, 1.0e-6
      assert_in_delta j4, 0.0, 1.0e-6
      assert_in_delta j5, 0.0, 1.0e-6
      assert_in_delta j6, 0.0, 1.0e-6
      assert_in_delta j7, 0.0, 1.0e-6
    end

    test "appends speed and accel as the 8th and 9th fp32" do
      frame = Protocol.cmd_move_joints(0, [], 2.5, 15.0)
      params = binary_part(frame, 7, 40)
      floats = Protocol.decode_fp32s(params, 10)
      speed = Enum.at(floats, 7)
      accel = Enum.at(floats, 8)
      assert_in_delta speed, 2.5, 1.0e-6
      assert_in_delta accel, 15.0, 1.0e-6
    end

    test "10th fp32 (mvtime) is always 0.0" do
      frame = Protocol.cmd_move_joints(0, [], 1.0, 1.0)
      params = binary_part(frame, 7, 40)
      floats = Protocol.decode_fp32s(params, 10)
      assert_in_delta List.last(floats), 0.0, 1.0e-9
    end

    test "truncates angle list longer than 7 to exactly 7 joints" do
      angles = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
      frame = Protocol.cmd_move_joints(0, angles, 1.0, 10.0)
      params = binary_part(frame, 7, 40)
      floats = Protocol.decode_fp32s(params, 10)

      assert_in_delta Enum.at(floats, 6), 7.0, 1.0e-6
      assert_in_delta Enum.at(floats, 7), 1.0, 1.0e-6
    end

    test "produces the same 47-byte frame size regardless of input angle count" do
      for count <- [0, 3, 6, 7, 10] do
        angles = List.duplicate(0.5, count)
        frame = Protocol.cmd_move_joints(0, angles, 1.0, 1.0)
        assert byte_size(frame) == 47, "expected 47 bytes for #{count} angles"
      end
    end
  end

  # ── cmd_move_cartesian ───────────────────────────────────────────────────────

  describe "cmd_move_cartesian/4" do
    test "produces a 43-byte frame (7 header + 9×4 param bytes)" do
      frame = Protocol.cmd_move_cartesian(0, {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}, 100.0, 2000.0)
      # 6 header + 1 register + 9 × 4 bytes = 43
      assert byte_size(frame) == 43
    end

    test "uses register 0x15" do
      frame = Protocol.cmd_move_cartesian(0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 100.0, 2000.0)
      assert binary_part(frame, 6, 1) == <<0x15>>
    end

    test "encodes pose and speed correctly" do
      frame = Protocol.cmd_move_cartesian(0, {100.0, 200.0, 300.0, 0.1, 0.2, 0.3}, 50.0, 1000.0)
      params = binary_part(frame, 7, 36)
      floats = Protocol.decode_fp32s(params, 9)

      [x, y, z, roll, pitch, yaw, speed, accel, mvtime] = floats
      assert_in_delta x, 100.0, 0.001
      assert_in_delta y, 200.0, 0.001
      assert_in_delta z, 300.0, 0.001
      assert_in_delta roll, 0.1, 1.0e-6
      assert_in_delta pitch, 0.2, 1.0e-6
      assert_in_delta yaw, 0.3, 1.0e-6
      assert_in_delta speed, 50.0, 0.001
      assert_in_delta accel, 1000.0, 0.01
      assert_in_delta mvtime, 0.0, 1.0e-9
    end
  end

  # ── cmd_gripper_position (clamp) ─────────────────────────────────────────────

  describe "cmd_gripper_position/2" do
    test "uses register 0x7C (RS485_RTU proxy)" do
      frame = Protocol.cmd_gripper_position(0, 500)
      assert binary_part(frame, 6, 1) == <<0x7C>>
    end

    test "clamps value above 840 to 840" do
      frame_max = Protocol.cmd_gripper_position(0, 840)
      frame_over = Protocol.cmd_gripper_position(0, 1000)
      # Both should produce the same payload (same clamped pulse)
      assert frame_max == frame_over
    end

    test "clamps negative value to 0" do
      frame_zero = Protocol.cmd_gripper_position(0, 0)
      frame_neg = Protocol.cmd_gripper_position(0, -50)
      assert frame_zero == frame_neg
    end

    test "800 and 840 produce different frames" do
      refute Protocol.cmd_gripper_position(0, 800) == Protocol.cmd_gripper_position(0, 840)
    end
  end

  # ── cmd_gripper_enable ───────────────────────────────────────────────────────

  describe "cmd_gripper_enable/2" do
    test "uses register 0x7C" do
      frame = Protocol.cmd_gripper_enable(0, true)
      assert binary_part(frame, 6, 1) == <<0x7C>>
    end

    test "enable and disable produce different frames" do
      refute Protocol.cmd_gripper_enable(0, true) == Protocol.cmd_gripper_enable(0, false)
    end
  end

  # ── cmd_ft_sensor_enable / cmd_get_ft_data ──────────────────────────────────

  describe "cmd_ft_sensor_enable/2" do
    test "uses register 0xC9" do
      frame = Protocol.cmd_ft_sensor_enable(0, true)
      assert binary_part(frame, 6, 1) == <<0xC9>>
    end

    test "enable byte is 1 when enabling" do
      frame = Protocol.cmd_ft_sensor_enable(0, true)
      assert binary_part(frame, 7, 1) == <<0x01>>
    end

    test "enable byte is 0 when disabling" do
      frame = Protocol.cmd_ft_sensor_enable(0, false)
      assert binary_part(frame, 7, 1) == <<0x00>>
    end
  end

  describe "cmd_get_ft_data/1" do
    test "produces a 7-byte frame (no params)" do
      assert byte_size(Protocol.cmd_get_ft_data(0)) == 7
    end

    test "uses register 0xC8" do
      frame = Protocol.cmd_get_ft_data(0)
      assert binary_part(frame, 6, 1) == <<0xC8>>
    end
  end

  # ── cmd_set_tcp_offset ───────────────────────────────────────────────────────

  describe "cmd_set_tcp_offset/7" do
    test "produces a 31-byte frame (6 header + 1 register + 6×4 params)" do
      assert byte_size(Protocol.cmd_set_tcp_offset(0, 0.0, 0.0, 172.0, 0.0, 0.0, 0.0)) == 31
    end

    test "uses register 0x23" do
      frame = Protocol.cmd_set_tcp_offset(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
      assert binary_part(frame, 6, 1) == <<0x23>>
    end

    test "encodes all six pose fields as fp32 LE" do
      frame = Protocol.cmd_set_tcp_offset(0, 10.0, 20.0, 172.0, 0.1, 0.2, 0.3)
      params = binary_part(frame, 7, 24)
      [x, y, z, roll, pitch, yaw] = Protocol.decode_fp32s(params, 6)
      assert_in_delta x, 10.0, 0.001
      assert_in_delta y, 20.0, 0.001
      assert_in_delta z, 172.0, 0.001
      assert_in_delta roll, 0.1, 1.0e-6
      assert_in_delta pitch, 0.2, 1.0e-6
      assert_in_delta yaw, 0.3, 1.0e-6
    end

    test "two different offsets produce different frames" do
      f1 = Protocol.cmd_set_tcp_offset(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
      f2 = Protocol.cmd_set_tcp_offset(0, 0.0, 0.0, 172.0, 0.0, 0.0, 0.0)
      refute f1 == f2
    end
  end

  # ── cmd_set_tcp_load ─────────────────────────────────────────────────────────

  describe "cmd_set_tcp_load/5" do
    test "produces a 23-byte frame (6 header + 1 register + 4×4 params)" do
      assert byte_size(Protocol.cmd_set_tcp_load(0, 0.82, 0.0, 0.0, 48.0)) == 23
    end

    test "uses register 0x24" do
      frame = Protocol.cmd_set_tcp_load(0, 0.0, 0.0, 0.0, 0.0)
      assert binary_part(frame, 6, 1) == <<0x24>>
    end

    test "encodes mass and center of mass as fp32 LE" do
      frame = Protocol.cmd_set_tcp_load(0, 0.82, 1.0, 2.0, 48.0)
      params = binary_part(frame, 7, 16)
      [mass, cx, cy, cz] = Protocol.decode_fp32s(params, 4)
      assert_in_delta mass, 0.82, 0.001
      assert_in_delta cx, 1.0, 0.001
      assert_in_delta cy, 2.0, 0.001
      assert_in_delta cz, 48.0, 0.001
    end
  end

  # ── cmd_set_reduced_mode ─────────────────────────────────────────────────────

  describe "cmd_set_reduced_mode/2" do
    test "produces an 8-byte frame" do
      assert byte_size(Protocol.cmd_set_reduced_mode(0, true)) == 8
    end

    test "uses register 0x32" do
      frame = Protocol.cmd_set_reduced_mode(0, true)
      assert binary_part(frame, 6, 1) == <<0x32>>
    end

    test "enable byte is 1 when enabling" do
      frame = Protocol.cmd_set_reduced_mode(0, true)
      assert binary_part(frame, 7, 1) == <<0x01>>
    end

    test "enable byte is 0 when disabling" do
      frame = Protocol.cmd_set_reduced_mode(0, false)
      assert binary_part(frame, 7, 1) == <<0x00>>
    end
  end

  # ── cmd_set_reduced_tcp_speed ────────────────────────────────────────────────

  describe "cmd_set_reduced_tcp_speed/2" do
    test "produces an 11-byte frame (6 header + 1 register + 4 fp32)" do
      assert byte_size(Protocol.cmd_set_reduced_tcp_speed(0, 250.0)) == 11
    end

    test "uses register 0x2F" do
      frame = Protocol.cmd_set_reduced_tcp_speed(0, 250.0)
      assert binary_part(frame, 6, 1) == <<0x2F>>
    end

    test "encodes speed as fp32 LE" do
      frame = Protocol.cmd_set_reduced_tcp_speed(0, 250.0)
      <<speed::float-little-32>> = binary_part(frame, 7, 4)
      assert_in_delta speed, 250.0, 0.001
    end
  end

  # ── cmd_set_reduced_joint_speed ──────────────────────────────────────────────

  describe "cmd_set_reduced_joint_speed/2" do
    test "produces an 11-byte frame" do
      assert byte_size(Protocol.cmd_set_reduced_joint_speed(0, 1.0)) == 11
    end

    test "uses register 0x30" do
      frame = Protocol.cmd_set_reduced_joint_speed(0, 1.0)
      assert binary_part(frame, 6, 1) == <<0x30>>
    end

    test "encodes speed as fp32 LE" do
      frame = Protocol.cmd_set_reduced_joint_speed(0, 1.5708)
      <<speed::float-little-32>> = binary_part(frame, 7, 4)
      assert_in_delta speed, 1.5708, 0.001
    end
  end

  # ── cmd_set_reduced_joint_ranges ─────────────────────────────────────────────

  describe "cmd_set_reduced_joint_ranges/2" do
    setup do
      ranges = [
        {-6.2832, 6.2832},
        {-2.059, 2.094},
        {-3.927, 0.192},
        {-6.2832, 6.2832},
        {-1.693, 3.142},
        {-6.2832, 6.2832},
        {-6.2832, 6.2832}
      ]

      {:ok, ranges: ranges}
    end

    test "produces a 63-byte frame (6 header + 1 register + 14×4 params)", %{ranges: ranges} do
      assert byte_size(Protocol.cmd_set_reduced_joint_ranges(0, ranges)) == 63
    end

    test "uses register 0x3A", %{ranges: ranges} do
      frame = Protocol.cmd_set_reduced_joint_ranges(0, ranges)
      assert binary_part(frame, 6, 1) == <<0x3A>>
    end

    test "flattens pairs to 14× fp32 LE in [j1_min, j1_max, ...] order", %{ranges: ranges} do
      frame = Protocol.cmd_set_reduced_joint_ranges(0, ranges)
      floats = Protocol.decode_fp32s(binary_part(frame, 7, 56), 14)

      assert length(floats) == 14
      # J1 pair
      [j1_min, j1_max | _] = floats
      assert_in_delta j1_min, -6.2832, 0.001
      assert_in_delta j1_max, 6.2832, 0.001
      # J2 pair
      [_, _, j2_min, j2_max | _] = floats
      assert_in_delta j2_min, -2.059, 0.001
      assert_in_delta j2_max, 2.094, 0.001
    end
  end

  # ── cmd_set_tcp_boundary ─────────────────────────────────────────────────────

  describe "cmd_set_tcp_boundary/7" do
    test "produces a 31-byte frame (6 header + 1 register + 6×4 int32)" do
      assert byte_size(Protocol.cmd_set_tcp_boundary(0, -400, 400, -400, 400, 0, 800)) == 31
    end

    test "uses register 0x34" do
      frame = Protocol.cmd_set_tcp_boundary(0, 0, 0, 0, 0, 0, 0)
      assert binary_part(frame, 6, 1) == <<0x34>>
    end

    test "encodes boundary values as signed 32-bit big-endian integers" do
      frame = Protocol.cmd_set_tcp_boundary(0, -400, 400, -300, 300, 0, 800)
      params = binary_part(frame, 7, 24)

      <<x_min::signed-32, x_max::signed-32, y_min::signed-32, y_max::signed-32, z_min::signed-32,
        z_max::signed-32>> = params

      assert x_min == -400
      assert x_max == 400
      assert y_min == -300
      assert y_max == 300
      assert z_min == 0
      assert z_max == 800
    end

    test "handles negative z_min correctly" do
      frame = Protocol.cmd_set_tcp_boundary(0, 0, 0, 0, 0, -100, 100)
      params = binary_part(frame, 7, 24)
      <<_::64, _::64, z_min::signed-32, z_max::signed-32>> = params
      assert z_min == -100
      assert z_max == 100
    end
  end

  # ── cmd_set_fence_on ────────────────────────────────────────────────────────

  describe "cmd_set_fence_on/2" do
    test "produces an 8-byte frame" do
      assert byte_size(Protocol.cmd_set_fence_on(0, true)) == 8
    end

    test "uses register 0x3B" do
      frame = Protocol.cmd_set_fence_on(0, true)
      assert binary_part(frame, 6, 1) == <<0x3B>>
    end

    test "enable byte is 1 when enabling" do
      frame = Protocol.cmd_set_fence_on(0, true)
      assert binary_part(frame, 7, 1) == <<0x01>>
    end

    test "enable byte is 0 when disabling" do
      frame = Protocol.cmd_set_fence_on(0, false)
      assert binary_part(frame, 7, 1) == <<0x00>>
    end
  end

  # ── cmd_set_collision_sensitivity ────────────────────────────────────────────

  describe "cmd_set_collision_sensitivity/2" do
    test "produces an 8-byte frame" do
      assert byte_size(Protocol.cmd_set_collision_sensitivity(0, 3)) == 8
    end

    test "uses register 0x25" do
      frame = Protocol.cmd_set_collision_sensitivity(0, 3)
      assert binary_part(frame, 6, 1) == <<0x25>>
    end

    for level <- 0..5 do
      test "encodes sensitivity level #{level} as u8" do
        frame = Protocol.cmd_set_collision_sensitivity(0, unquote(level))
        assert binary_part(frame, 7, 1) == <<unquote(level)::8>>
      end
    end

    test "level 0 and level 5 produce different frames" do
      refute Protocol.cmd_set_collision_sensitivity(0, 0) ==
               Protocol.cmd_set_collision_sensitivity(0, 5)
    end
  end

  # ── cmd_set_collision_rebound ────────────────────────────────────────────────

  describe "cmd_set_collision_rebound/2" do
    test "produces an 8-byte frame" do
      assert byte_size(Protocol.cmd_set_collision_rebound(0, true)) == 8
    end

    test "uses register 0x3C" do
      frame = Protocol.cmd_set_collision_rebound(0, true)
      assert binary_part(frame, 6, 1) == <<0x3C>>
    end

    test "enable byte is 1 when enabling" do
      frame = Protocol.cmd_set_collision_rebound(0, true)
      assert binary_part(frame, 7, 1) == <<0x01>>
    end

    test "enable byte is 0 when disabling" do
      frame = Protocol.cmd_set_collision_rebound(0, false)
      assert binary_part(frame, 7, 1) == <<0x00>>
    end
  end

  # ── cmd_set_self_collision_check ─────────────────────────────────────────────

  describe "cmd_set_self_collision_check/2" do
    test "produces an 8-byte frame" do
      assert byte_size(Protocol.cmd_set_self_collision_check(0, true)) == 8
    end

    test "uses register 0x4D" do
      frame = Protocol.cmd_set_self_collision_check(0, true)
      assert binary_part(frame, 6, 1) == <<0x4D>>
    end

    test "enable byte is 1 when enabling" do
      frame = Protocol.cmd_set_self_collision_check(0, true)
      assert binary_part(frame, 7, 1) == <<0x01>>
    end

    test "enable byte is 0 when disabling" do
      frame = Protocol.cmd_set_self_collision_check(0, false)
      assert binary_part(frame, 7, 1) == <<0x00>>
    end
  end
end
