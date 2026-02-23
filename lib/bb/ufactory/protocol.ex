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
  @reg_get_error 0x0F
  @reg_ft_get_data 0xC8
  @reg_ft_enable 0xC9

  # ── Registers for hardware configuration (TCP offset, load, reduced mode,
  #    collision detection) ─────────────────────────────────────────────────
  @reg_set_tcp_offset 0x23
  @reg_set_load_param 0x24
  @reg_set_collis_sens 0x25
  @reg_set_reduced_trsv 0x2F
  @reg_set_reduced_p2pv 0x30
  @reg_set_reduced_mode 0x32
  @reg_set_limit_xyz 0x34
  @reg_set_reduced_jrange 0x3A
  @reg_set_fense_on 0x3B
  @reg_set_collis_reb 0x3C
  @reg_set_self_collis_check 0x4D

  # ── RS485 proxy IDs used for accessories ──────────────────────────────────
  # host_id 9 = robot-side RS485 bus (ROBOT_RS485_HOST_ID)
  @rs485_host_id 0x09
  # device_id 8 = xArm gripper bus address (GRIPPER_ID)
  @gripper_device_id 0x08

  # Gripper servo register addresses (from XCONF.ServoConf)
  @gripper_reg_con_en 0x0100
  @gripper_reg_taget_pos 0x0700
  @gripper_reg_speed 0x0303

  # Linear track RS485 device ID and servo register addresses (from XCONF.ServoConf)
  # device_id 1 = linear track bus address (LINEAR_TRACK_ID)
  @linear_track_device_id 0x01
  # Write position: TAGET_POS register, int32 big-endian / 2000 = mm (spans two 16-bit registers)
  # Note: 0x0A20 is the *read* address; writes go to 0x0700 via RS485 proxy.
  @linear_track_reg_pos 0x0700
  # Write speed: POS_SPD register, int16 big-endian (internal units = mm/s * 6.667)
  # Note: 0x0A26 is the *read* address; writes go to 0x0303 via RS485 proxy.
  @linear_track_reg_speed 0x0303

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
  Requests the current error and warning codes from the arm (register 0x0F).

  The arm responds with 2 bytes: `<<error_code::8, warn_code::8>>`.
  """
  @spec cmd_get_error(non_neg_integer()) :: binary()
  def cmd_get_error(txn_id) do
    build_frame(txn_id, @reg_get_error, <<>>)
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
  Sets the gripper speed in pulse units per second.

  The speed is sent as an unsigned 16-bit integer to the POS_SPD register
  (0x0303) on the gripper device. No scaling factor is applied — the value
  is the raw servo speed in pulse/s (matching the Python SDK's
  `gripper_modbus_set_posspd`).
  """
  @spec cmd_gripper_speed(non_neg_integer(), non_neg_integer()) :: binary()
  def cmd_gripper_speed(txn_id, speed) do
    spd_bytes = <<speed::unsigned-16>>
    params = rs485_write_registers(@gripper_device_id, @gripper_reg_speed, spd_bytes)
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

  @doc """
  Returns `{pos_frame, spd_frame}` — two frames to move the linear track to
  `position_mm` at `speed` mm/s.

  The position register (`TAGET_POS`, `0x0700`) and speed register (`POS_SPD`,
  `0x0303`) are at non-contiguous RS485 servo addresses, so they require
  separate write frames. The caller (actuator) sends the speed frame first,
  then the position frame.

  Position encoding: `round(position_mm * 2000)` as a signed 32-bit
  **big-endian** integer spanning two 16-bit registers (`0x0700`/`0x0701`).
  This is the **only** place in the UFactory protocol where position is
  big-endian int32 rather than little-endian fp32 — a documented exception in
  the Python SDK (`xarm/x3/linear_motor.py`).

  Speed encoding: `round(speed_mm_s * 6.667)` as an unsigned 16-bit integer.
  The `6.667` factor converts from mm/s to the servo's internal speed units
  (matching the Python SDK's `set_linear_motor_speed`).

  ## Examples

      iex> {pos_frame, spd_frame} = BB.Ufactory.Protocol.cmd_linear_track_move(1, 500.0, 200)
      iex> is_binary(pos_frame) and is_binary(spd_frame)
      true
  """
  @spec cmd_linear_track_move(non_neg_integer(), float(), non_neg_integer()) ::
          {binary(), binary()}
  def cmd_linear_track_move(txn_id, position_mm, speed) do
    pos_units = round(position_mm * 2000)
    pos_bytes = <<pos_units::signed-32>>
    spd_units = round(speed * 6.667)
    spd_bytes = <<spd_units::unsigned-16>>

    pos_frame =
      build_frame(
        txn_id,
        @reg_rs485_rtu,
        rs485_write_registers(@linear_track_device_id, @linear_track_reg_pos, pos_bytes)
      )

    spd_frame =
      build_frame(
        txn_id,
        @reg_rs485_rtu,
        rs485_write_registers(@linear_track_device_id, @linear_track_reg_speed, spd_bytes)
      )

    {pos_frame, spd_frame}
  end

  # ── TCP tool offset and payload ─────────────────────────────────────────────

  @doc """
  Sets the tool center point (TCP) offset relative to the flange.

  Tells the arm where the tool tip is, enabling accurate Cartesian motion and
  force-torque readings in tool coordinates. Applied immediately and persisted
  in the arm's NVRAM.

  Position in millimetres; orientation in radians. Payload: 6× fp32 LE.

  ## Examples

      iex> frame = BB.Ufactory.Protocol.cmd_set_tcp_offset(0, 0.0, 0.0, 172.0, 0.0, 0.0, 0.0)
      iex> byte_size(frame)
      31
  """
  @spec cmd_set_tcp_offset(
          non_neg_integer(),
          float(),
          float(),
          float(),
          float(),
          float(),
          float()
        ) ::
          binary()
  def cmd_set_tcp_offset(txn_id, x, y, z, roll, pitch, yaw) do
    build_frame(txn_id, @reg_set_tcp_offset, encode_fp32s([x, y, z, roll, pitch, yaw]))
  end

  @doc """
  Sets the TCP payload (tool mass and center of gravity).

  Accurate payload configuration improves motion planning, collision detection,
  and force-torque readings. Applied immediately and persisted in NVRAM.

  - `mass_kg` — tool mass in kilograms
  - `com_x/y/z` — center of mass offset from flange, in millimetres

  Payload: 4× fp32 LE `[mass, com_x, com_y, com_z]`.

  ## Examples

      iex> frame = BB.Ufactory.Protocol.cmd_set_tcp_load(0, 0.82, 0.0, 0.0, 48.0)
      iex> byte_size(frame)
      23
  """
  @spec cmd_set_tcp_load(non_neg_integer(), float(), float(), float(), float()) :: binary()
  def cmd_set_tcp_load(txn_id, mass_kg, com_x, com_y, com_z) do
    build_frame(txn_id, @reg_set_load_param, encode_fp32s([mass_kg, com_x, com_y, com_z]))
  end

  # ── Reduced / safe mode ──────────────────────────────────────────────────────

  @doc """
  Enables or disables the arm's firmware reduced mode.

  In reduced mode, the arm enforces lower speed limits and optionally a
  Cartesian workspace fence. Configure limits with `cmd_set_reduced_tcp_speed/2`,
  `cmd_set_reduced_joint_speed/2`, and `cmd_set_tcp_boundary/7` before enabling.

  Payload: 1× u8 (0 = off, 1 = on).
  """
  @spec cmd_set_reduced_mode(non_neg_integer(), boolean()) :: binary()
  def cmd_set_reduced_mode(txn_id, enable) do
    value = if enable, do: 1, else: 0
    build_frame(txn_id, @reg_set_reduced_mode, <<value::8>>)
  end

  @doc """
  Sets the maximum TCP linear speed used in reduced mode.

  `speed_mm_s` — maximum end-effector speed in mm/s. Payload: 1× fp32 LE.
  """
  @spec cmd_set_reduced_tcp_speed(non_neg_integer(), float()) :: binary()
  def cmd_set_reduced_tcp_speed(txn_id, speed_mm_s) do
    build_frame(txn_id, @reg_set_reduced_trsv, encode_fp32(speed_mm_s))
  end

  @doc """
  Sets the maximum joint speed used in reduced mode.

  `speed_rad_s` — maximum joint speed in rad/s. Payload: 1× fp32 LE.
  """
  @spec cmd_set_reduced_joint_speed(non_neg_integer(), float()) :: binary()
  def cmd_set_reduced_joint_speed(txn_id, speed_rad_s) do
    build_frame(txn_id, @reg_set_reduced_p2pv, encode_fp32(speed_rad_s))
  end

  @doc """
  Sets per-joint angle ranges enforced in reduced mode.

  `ranges` must be a list of exactly 7 `{lower_rad, upper_rad}` tuples (J1..J7).
  Pairs are flattened to 14× fp32 LE: `[j1_min, j1_max, j2_min, j2_max, ...]`.

  For arms with fewer than 7 joints, the unused joints' ranges are ignored by
  firmware but must still be included in the frame.
  """
  @spec cmd_set_reduced_joint_ranges(non_neg_integer(), [{float(), float()}]) :: binary()
  def cmd_set_reduced_joint_ranges(txn_id, ranges) when length(ranges) == 7 do
    floats = Enum.flat_map(ranges, fn {lo, hi} -> [lo, hi] end)
    build_frame(txn_id, @reg_set_reduced_jrange, encode_fp32s(floats))
  end

  @doc """
  Sets the Cartesian workspace boundary (fence) in millimetres.

  When the fence is enabled via `cmd_set_fence_on/2`, the arm stops if the
  TCP pose exceeds these limits.

  Encoded as 6× signed 32-bit **big-endian** integers (same endianness
  exception as the linear track position register).
  """
  @spec cmd_set_tcp_boundary(
          non_neg_integer(),
          integer(),
          integer(),
          integer(),
          integer(),
          integer(),
          integer()
        ) :: binary()
  def cmd_set_tcp_boundary(txn_id, x_min, x_max, y_min, y_max, z_min, z_max) do
    payload =
      <<x_min::signed-32, x_max::signed-32, y_min::signed-32, y_max::signed-32, z_min::signed-32,
        z_max::signed-32>>

    build_frame(txn_id, @reg_set_limit_xyz, payload)
  end

  @doc """
  Enables or disables the Cartesian workspace fence.

  The fence limits must first be set with `cmd_set_tcp_boundary/7`. When on,
  motion that would move the TCP outside the boundary is rejected by firmware.

  Payload: 1× u8 (0 = off, 1 = on).
  """
  @spec cmd_set_fence_on(non_neg_integer(), boolean()) :: binary()
  def cmd_set_fence_on(txn_id, enable) do
    value = if enable, do: 1, else: 0
    build_frame(txn_id, @reg_set_fense_on, <<value::8>>)
  end

  # ── Collision detection ──────────────────────────────────────────────────────

  @doc """
  Sets the collision detection sensitivity level.

  `sensitivity` must be in the range `0..5`:

  - `0` — collision detection disabled
  - `1` — lowest sensitivity (hardest to trigger)
  - `5` — highest sensitivity (easiest to trigger)

  Payload: 1× u8.
  """
  @spec cmd_set_collision_sensitivity(non_neg_integer(), 0..5) :: binary()
  def cmd_set_collision_sensitivity(txn_id, sensitivity)
      when is_integer(sensitivity) and sensitivity in 0..5 do
    build_frame(txn_id, @reg_set_collis_sens, <<sensitivity::8>>)
  end

  @doc """
  Enables or disables collision rebound.

  When enabled, the arm briefly moves back along its path after detecting a
  collision, helping to clear the contact. When disabled, the arm stops in place.

  Payload: 1× u8 (0 = off, 1 = on).
  """
  @spec cmd_set_collision_rebound(non_neg_integer(), boolean()) :: binary()
  def cmd_set_collision_rebound(txn_id, enable) do
    value = if enable, do: 1, else: 0
    build_frame(txn_id, @reg_set_collis_reb, <<value::8>>)
  end

  @doc """
  Enables or disables the self-collision geometric model check.

  When enabled, the firmware computes forward kinematics on each planned
  trajectory point and rejects moves where arm links would intersect.

  Payload: 1× u8 (0 = off, 1 = on).
  """
  @spec cmd_set_self_collision_check(non_neg_integer(), boolean()) :: binary()
  def cmd_set_self_collision_check(txn_id, enable) do
    value = if enable, do: 1, else: 0
    build_frame(txn_id, @reg_set_self_collis_check, <<value::8>>)
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
