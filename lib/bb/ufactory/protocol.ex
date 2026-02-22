# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Protocol do
  @moduledoc """
  Pure encode/decode for the UFactory xArm Modbus-TCP wire protocol.

  No network I/O is performed here. All functions are pure binary transforms
  suitable for unit testing without hardware.

  ## Frame Format (command socket, port 502)

  ```
  ┌─ Header (6 bytes) ───────────────────────────────────┐
  │  Transaction ID  u16 big-endian                       │
  │  Protocol ID     u16 big-endian  (always 0x0002)      │
  │  Length          u16 big-endian  (= 1 + len(params))  │
  ├─ Body ───────────────────────────────────────────────┤
  │  Register        u8                                   │
  │  Params          n bytes (fp32 fields: little-endian) │
  └──────────────────────────────────────────────────────┘
  ```

  Response adds a `Status` u8 immediately after the `Register` byte:

  ```
  Header (6 bytes) | Register u8 | Status u8 | Response params n bytes
  ```

  ## Float encoding

  All `fp32` payload fields are **little-endian**. Header u16 fields are
  **big-endian**. This mixed endianness is explicitly documented in the
  xArm Developer Manual V1.10.0 and must be handled exactly.

  ## Heartbeat

  The arm drops the command connection if it does not receive a heartbeat
  at least every ~10 seconds. Send `@heartbeat` on the command socket once
  per second.

  ## Transaction IDs

  Transaction IDs are the caller's responsibility in Phase 2. The `BB.Ufactory.Controller`
  will manage an incrementing counter in Phase 3. Pass `0` in unit tests.
  """

  # ── Private protocol identifier used in frame headers ─────────────────────
  @protocol_id 0x0002

  # ── Heartbeat ──────────────────────────────────────────────────────────────
  @heartbeat <<0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00>>

  # ── Register addresses (inlined to avoid a cross-module dependency in
  #    pattern matching; the canonical source is BB.Ufactory.Registers) ───────
  @reg_motion_en 0x0B
  @reg_set_state 0x0C
  @reg_move_cart 0x15
  @reg_move_joints 0x17
  @reg_get_tcp_pose 0x29
  @reg_get_joints 0x2A
  @reg_rs485_rtu 0x7C
  @reg_ft_get_data 0xC8
  @reg_ft_enable 0xC9

  # ── RS485 proxy IDs used for accessories ──────────────────────────────────
  # host_id 9 = robot-side RS485 bus (ROBOT_RS485_HOST_ID)
  @rs485_host_id 0x09
  # device_id 8 = xArm gripper bus address (GRIPPER_ID)
  @gripper_device_id 0x08

  # Gripper servo register addresses (from XCONF.ServoConf)
  @gripper_reg_con_en 0x0100
  @gripper_reg_taget_pos 0x0700

  @doc "Raw heartbeat frame. Send on the command socket once per second."
  @spec heartbeat() :: binary()
  def heartbeat, do: @heartbeat

  # ── Core frame building ────────────────────────────────────────────────────

  @doc """
  Builds a complete Modbus-TCP frame binary.

  `params` is the raw parameter binary (fp32 LE fields). Pass `<<>>` for
  commands that take no parameters.

  ## Examples

      iex> BB.Ufactory.Protocol.build_frame(1, 0x0C, <<0x03>>)
      <<0, 1, 0, 2, 0, 2, 12, 3>>
  """
  @spec build_frame(non_neg_integer(), byte(), binary()) :: binary()
  def build_frame(transaction_id, register, params) when is_binary(params) do
    length = byte_size(params) + 1
    <<transaction_id::16, @protocol_id::16, length::16, register::8>> <> params
  end

  @doc """
  Parses a response frame from the command socket.

  Returns:
  - `{:ok, {register, status, params_binary}, rest}` — a complete frame was
    parsed; `rest` is any trailing bytes not consumed.
  - `{:more}` — not enough bytes yet; accumulate more data and retry.
  - `{:error, reason}` — malformed protocol identifier.
  """
  @spec parse_response(binary()) ::
          {:ok, {byte(), byte(), binary()}, binary()} | {:more} | {:error, term()}
  def parse_response(binary) when byte_size(binary) < 6, do: {:more}

  def parse_response(<<_txn_id::16, @protocol_id::16, length::16, rest::binary>>) do
    if byte_size(rest) < length do
      {:more}
    else
      <<body::binary-size(length), tail::binary>> = rest
      <<register::8, status::8, params::binary>> = body
      {:ok, {register, status, params}, tail}
    end
  end

  def parse_response(<<_txn_id::16, bad_prot::16, _::binary>>) do
    {:error, {:bad_protocol_id, bad_prot}}
  end

  # ── Encode / decode helpers ────────────────────────────────────────────────

  @doc """
  Encodes a float as a 4-byte little-endian IEEE 754 binary.

  ## Examples

      iex> BB.Ufactory.Protocol.encode_fp32(1.0)
      <<0, 0, 128, 63>>
  """
  @spec encode_fp32(float()) :: <<_::32>>
  def encode_fp32(f), do: <<f::float-little-32>>

  @doc """
  Decodes a 4-byte little-endian IEEE 754 binary to a float.

  ## Examples

      iex> BB.Ufactory.Protocol.decode_fp32(<<0, 0, 128, 63>>)
      1.0
  """
  @spec decode_fp32(<<_::32>>) :: float()
  def decode_fp32(<<f::float-little-32>>), do: f

  @doc """
  Decodes `count` consecutive 4-byte little-endian floats from the start of `binary`.

  Raises if `binary` is shorter than `count * 4` bytes.

  ## Examples

      iex> BB.Ufactory.Protocol.decode_fp32s(<<0, 0, 128, 63, 0, 0, 0, 64>>, 2)
      [1.0, 2.0]
  """
  @spec decode_fp32s(binary(), non_neg_integer()) :: [float()]
  def decode_fp32s(binary, count) do
    for <<f::float-little-32 <- binary_part(binary, 0, count * 4)>>, do: f
  end

  @doc """
  Encodes a list of floats as little-endian fp32 bytes.
  """
  @spec encode_fp32s([float()]) :: binary()
  def encode_fp32s(floats), do: for(f <- floats, into: <<>>, do: <<f::float-little-32>>)

  # ── Command builders ────────────────────────────────────────────────────────
  #
  # Each builder returns a complete binary frame ready to send on the TCP socket.

  @doc """
  Enables or disables all joint motors.

  Sends register `0x0B` (MOTION_EN) with axis_id `0x08` (all joints).

  ## Examples

      iex> frame = BB.Ufactory.Protocol.cmd_enable(1, true)
      iex> byte_size(frame)
      9
  """
  @spec cmd_enable(non_neg_integer(), boolean()) :: binary()
  def cmd_enable(txn_id, enable) do
    enable_byte = if enable, do: 1, else: 0
    build_frame(txn_id, @reg_motion_en, <<0x08, enable_byte>>)
  end

  @doc """
  Sets the arm state.

  State values:
  - `0` — stop / clear motion queue (also see `cmd_stop/1`)
  - `3` — start motion (play)
  - `4` — pause

  ## Examples

      iex> frame = BB.Ufactory.Protocol.cmd_set_state(1, 3)
      iex> byte_size(frame)
      8
  """
  @spec cmd_set_state(non_neg_integer(), 0..4) :: binary()
  def cmd_set_state(txn_id, state) do
    build_frame(txn_id, @reg_set_state, <<state::8>>)
  end

  @doc """
  Sends a stop command (SET_STATE with value 0 — clears motion queue).
  """
  @spec cmd_stop(non_neg_integer()) :: binary()
  def cmd_stop(txn_id), do: cmd_set_state(txn_id, 0)

  @doc """
  Requests the current joint angles from the arm.

  The arm responds with 7× fp32 LE (radians). Use `decode_fp32s/2` on the
  response params to extract the values.
  """
  @spec cmd_get_joint_angles(non_neg_integer()) :: binary()
  def cmd_get_joint_angles(txn_id) do
    build_frame(txn_id, @reg_get_joints, <<>>)
  end

  @doc """
  Requests the current Cartesian end-effector pose.

  The arm responds with 6× fp32 LE: [x, y, z, roll, pitch, yaw].
  """
  @spec cmd_get_cartesian_pose(non_neg_integer()) :: binary()
  def cmd_get_cartesian_pose(txn_id) do
    build_frame(txn_id, @reg_get_tcp_pose, <<>>)
  end

  @doc """
  Commands a joint-space move to the given angles.

  `angles` must have at most 7 elements (one per joint J1..J7). If fewer than
  7 are provided, the remainder are padded with `0.0`. This is safe for models
  with fewer than 7 joints since the arm ignores extra joints beyond its axis
  count.

  The frame payload is 10× fp32 LE: `[j1..j7, speed, accel, 0.0]`.

  All angle values are in **radians**. Speed in rad/s, acceleration in rad/s².

  ## Examples

      iex> frame = BB.Ufactory.Protocol.cmd_move_joints(1, [0.0, 0.5, -0.5, 0.0, 1.0, 0.0], 1.0, 10.0)
      iex> byte_size(frame)
      47
  """
  @spec cmd_move_joints(non_neg_integer(), [float()], float(), float()) :: binary()
  def cmd_move_joints(txn_id, angles, speed, accel) do
    padded = pad_angles(angles, 7)
    floats = padded ++ [speed, accel, 0.0]
    build_frame(txn_id, @reg_move_joints, encode_fp32s(floats))
  end

  @doc """
  Commands a Cartesian linear move (register 0x15, MOVE_LINE).

  Position is in millimetres; orientation (roll, pitch, yaw) is in radians.
  Speed in mm/s, acceleration in mm/s².

  The frame payload is 9× fp32 LE: `[x, y, z, roll, pitch, yaw, speed, accel, 0.0]`.

  ## Examples

      iex> frame = BB.Ufactory.Protocol.cmd_move_cartesian(1, {300.0, 0.0, 200.0, 0.0, 0.0, 0.0}, 100.0, 2000.0)
      iex> byte_size(frame)
      43
  """
  @spec cmd_move_cartesian(
          non_neg_integer(),
          {float(), float(), float(), float(), float(), float()},
          float(),
          float()
        ) :: binary()
  def cmd_move_cartesian(txn_id, {x, y, z, roll, pitch, yaw}, speed, accel) do
    floats = [x, y, z, roll, pitch, yaw, speed, accel, 0.0]
    build_frame(txn_id, @reg_move_cart, encode_fp32s(floats))
  end

  @doc """
  Enables or disables the xArm Gripper via the RS485-RTU proxy register.

  Sends a Modbus RTU "write single register" (function 0x10) frame to the
  gripper device (ID 0x08) targeting the CON_EN register (0x0100).
  """
  @spec cmd_gripper_enable(non_neg_integer(), boolean()) :: binary()
  def cmd_gripper_enable(txn_id, enable) do
    value = if enable, do: 1, else: 0
    params = rs485_write_registers(@gripper_device_id, @gripper_reg_con_en, <<0x00, value>>)
    build_frame(txn_id, @reg_rs485_rtu, params)
  end

  @doc """
  Sets the gripper position in servo pulse units.

  The Gripper G2 range is **0–840** pulse units. Values outside this range are
  clamped. The pulse is sent as an int32 big-endian to the TAGET_POS register
  (0x0700) on the gripper device.
  """
  @spec cmd_gripper_position(non_neg_integer(), non_neg_integer()) :: binary()
  def cmd_gripper_position(txn_id, pos) do
    clamped = max(0, min(840, pos))
    # int32 big-endian: upper 2 bytes to register 0x0700, lower 2 to 0x0701
    pulse_bytes = <<clamped::32>>
    params = rs485_write_registers(@gripper_device_id, @gripper_reg_taget_pos, pulse_bytes)
    build_frame(txn_id, @reg_rs485_rtu, params)
  end

  @doc """
  Enables or disables the force-torque sensor (register 0xC9).
  """
  @spec cmd_ft_sensor_enable(non_neg_integer(), boolean()) :: binary()
  def cmd_ft_sensor_enable(txn_id, enable) do
    value = if enable, do: 1, else: 0
    build_frame(txn_id, @reg_ft_enable, <<value::8>>)
  end

  @doc """
  Requests the latest force-torque data from the sensor (register 0xC8).

  The arm responds with 6× fp32 LE: `[Fx, Fy, Fz, Tx, Ty, Tz]`.
  """
  @spec cmd_get_ft_data(non_neg_integer()) :: binary()
  def cmd_get_ft_data(txn_id) do
    build_frame(txn_id, @reg_ft_get_data, <<>>)
  end

  # ── Private helpers ─────────────────────────────────────────────────────────

  # Pads `angles` to exactly `n` elements by appending 0.0 floats.
  defp pad_angles(angles, n) do
    len = length(angles)

    if len >= n do
      Enum.take(angles, n)
    else
      angles ++ List.duplicate(0.0, n - len)
    end
  end

  # Builds the RS485-RTU "write multiple registers" (Modbus 0x10) payload
  # for the xArm RS485 proxy register (0x7C).
  #
  # Format sent to the arm over 0x7C:
  #   host_id (1) | device_id (1) | 0x10 (1) | addr_hi (1) | addr_lo (1) |
  #   count_hi (1) | count_lo (1) | byte_count (1) | value_bytes (n)
  #
  # `value_bytes` must be an even number of bytes (16-bit registers).
  defp rs485_write_registers(device_id, reg_addr, value_bytes) do
    byte_count = byte_size(value_bytes)
    reg_count = div(byte_count, 2)

    <<@rs485_host_id::8, device_id::8, 0x10::8, reg_addr::16, reg_count::16, byte_count::8>> <>
      value_bytes
  end
end
