# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Report do
  import Bitwise

  @moduledoc """
  Pure parser for xArm auto-push report frames.

  The arm continuously pushes state frames on the report socket (port 30003
  for real-time, port 30001 for normal, port 30002 for rich). This module
  handles the real-time report frame layout as documented in xArm Developer
  Manual V1.10.0 and the Python SDK `report.py`.

  No network I/O is performed here. All functions are pure binary transforms.

  ## Frame Layout (real-time report, port 30003)

  Each push frame has a 4-byte length prefix:

  ```
  Bytes 0–3:    frame_length  u32 big-endian (total bytes of the frame INCLUDING
                              these 4 bytes — the payload is frame_length - 4
                              bytes)
  Byte  4:      state_mode    lower nibble = state (0..5), upper nibble = mode (0..3)
  Bytes 5–6:    cmd_count     u16 big-endian
  Bytes 7–34:   angles        7× fp32 little-endian (radians, J1–J7)
  Bytes 35–58:  pose          6× fp32 little-endian (mm for x/y/z, rad for roll/pitch/yaw)
  Bytes 59–86:  torques       7× fp32 little-endian (Nm, J1–J7)
  —— frame_length >= 111 ——
  Bytes 87–110: ft_filtered   6× fp32 little-endian ([Fx, Fy, Fz, Tx, Ty, Tz])
  —— frame_length >= 135 ——
  Bytes 111–134: ft_raw       6× fp32 little-endian ([Fx, Fy, Fz, Tx, Ty, Tz])
  ```

  The SDK's Python `report.py` uses `len(rx_data)` (the payload size) as its
  `length` field. Here we use the wire `frame_length` value from bytes 0–3,
  which includes the 4-byte header prefix. The minimum complete base frame is
  87 bytes of payload, requiring `frame_length >= 87`.

  The parser tolerates frames larger than expected (future firmware versions may
  add fields). Extra trailing bytes are ignored.

  ## Normal Report (port 30001, frame_length >= 133)

  In addition to the real-time fields, bytes 87–132 contain controller status:
  - Byte  87: `mtbrake`    brake-state bitfield (J1=bit0 .. J7=bit6)
  - Byte  88: `mtable`     enable-state bitfield
  - Byte  89: `error_code` current error code (0 = no error)
  - Byte  90: `warn_code`  current warning code
  - Bytes 91–114: `tcp_offset`  6× fp32 LE (mm + rad)
  - Bytes 115–130: `tcp_load`   4× fp32 LE (kg, mm×3)
  - Byte 131: `collis_sens`  collision sensitivity (0–5)
  - Byte 132: `teach_sens`   teach sensitivity (0–5)

  These fields are parsed when `frame_length >= 133`.

  ## Return Value

  `parse_report/1` returns:

  - `{:ok, report, rest}` — successfully parsed one frame; `rest` is any
    trailing data in the buffer (may contain the start of the next frame).
  - `{:more}` — not enough bytes in the buffer yet; accumulate more data.

  The `report` map always contains: `:state`, `:mode`, `:cmd_count`, `:angles`,
  `:pose`, `:torques`.

  Optional fields present only when the frame is large enough:
  - `:ft_filtered` — 6-element float list when `frame_length >= 111`
  - `:ft_raw`      — 6-element float list when `frame_length >= 135`
  - `:mtbrake`     — 8-element bit list when `frame_length >= 133`
  - `:mtable`      — 8-element bit list when `frame_length >= 133`
  - `:error_code`  — integer when `frame_length >= 133`
  - `:warn_code`   — integer when `frame_length >= 133`
  - `:tcp_offset`  — 6-element float list when `frame_length >= 133`
  - `:tcp_load`    — 4-element float list when `frame_length >= 133`
  - `:collis_sens` — integer when `frame_length >= 133`
  - `:teach_sens`  — integer when `frame_length >= 133`
  """

  @doc """
  Parses one report frame from the start of `buffer`.

  Returns `{:ok, report_map, rest}` on success, or `{:more}` if the buffer
  does not yet contain a complete frame.

  This function is designed to be called in a loop to drain the buffer:

      defp drain_buffer(buffer, state) do
        case BB.Ufactory.Report.parse_report(buffer) do
          {:ok, report, rest} ->
            state = handle_report(report, state)
            drain_buffer(rest, state)
          {:more} ->
            %{state | buffer: buffer}
        end
      end
  """
  @spec parse_report(binary()) ::
          {:ok, map(), binary()} | {:more}
  def parse_report(buffer) when byte_size(buffer) < 4, do: {:more}

  def parse_report(<<frame_length::32, rest::binary>>) do
    # frame_length is the TOTAL frame size including the 4-byte length prefix
    # (confirmed from Python SDK: report_size = bytes_to_u32(buffer[0:4]),
    #  then data = buffer[:report_size] which includes the 4 prefix bytes).
    # So the payload to parse is frame_length - 4 bytes.
    payload_length = frame_length - 4

    if byte_size(rest) < payload_length do
      {:more}
    else
      <<payload::binary-size(payload_length), tail::binary>> = rest
      report = parse_payload(payload, frame_length)
      {:ok, report, tail}
    end
  end

  # ── Payload parsing ─────────────────────────────────────────────────────────

  defp parse_payload(payload, frame_length) do
    # Base fields — always present (frame_length >= 87 == 7 + 28 + 24 + 28)
    <<state_mode::8, cmd_count::16, angles_bin::binary-28, pose_bin::binary-24,
      torques_bin::binary-28, rest::binary>> = payload

    state = state_mode &&& 0x0F
    mode = state_mode >>> 4

    report = %{
      state: state,
      mode: mode,
      cmd_count: cmd_count,
      angles: decode_fp32s(angles_bin, 7),
      pose: decode_fp32s(pose_bin, 6),
      torques: decode_fp32s(torques_bin, 7)
    }

    report
    |> maybe_parse_ft_filtered(rest, frame_length)
    |> maybe_parse_ft_raw(rest, frame_length)
    |> maybe_parse_normal(rest, frame_length)
  end

  # Force-torque filtered: frame bytes 87–110 (fp32 LE × 6 = 24 bytes).
  # Only present in the 135-byte real-time frame (port 30003 with F/T firmware).
  # The threshold is >= 135, not >= 111, to avoid mis-parsing the 133-byte
  # normal-report frame whose bytes 87+ contain mtbrake/mtable/error fields,
  # not force-torque data.
  defp maybe_parse_ft_filtered(report, rest, frame_length) when frame_length >= 135 do
    <<ft_filtered_bin::binary-24, _::binary>> = rest
    Map.put(report, :ft_filtered, decode_fp32s(ft_filtered_bin, 6))
  end

  defp maybe_parse_ft_filtered(report, _rest, _frame_length), do: report

  # Force-torque raw: frame bytes 111–134 (fp32 LE × 6 = 24 bytes).
  # Co-present with ft_filtered; both require frame_length >= 135.
  defp maybe_parse_ft_raw(report, rest, frame_length) when frame_length >= 135 do
    <<_ft_filtered::binary-24, ft_raw_bin::binary-24, _::binary>> = rest
    Map.put(report, :ft_raw, decode_fp32s(ft_raw_bin, 6))
  end

  defp maybe_parse_ft_raw(report, _rest, _frame_length), do: report

  # Normal report fields: frame bytes 87–132 (port 30001, frame_length == 133).
  # These occupy the same byte positions as ft_filtered in the real-time report,
  # so the guard uses `frame_length >= 133 and frame_length < 135` to
  # distinguish normal-report frames (133 bytes) from ft-extended real-time
  # frames (135+ bytes).
  defp maybe_parse_normal(report, rest, frame_length)
       when frame_length >= 133 and frame_length < 135 do
    <<mtbrake::8, mtable::8, error_code::8, warn_code::8, tcp_offset_bin::binary-24,
      tcp_load_bin::binary-16, collis_sens::8, teach_sens::8, _::binary>> = rest

    report
    |> Map.put(:mtbrake, bits_to_list(mtbrake))
    |> Map.put(:mtable, bits_to_list(mtable))
    |> Map.put(:error_code, error_code)
    |> Map.put(:warn_code, warn_code)
    |> Map.put(:tcp_offset, decode_fp32s(tcp_offset_bin, 6))
    |> Map.put(:tcp_load, decode_fp32s(tcp_load_bin, 4))
    |> Map.put(:collis_sens, collis_sens)
    |> Map.put(:teach_sens, teach_sens)
  end

  defp maybe_parse_normal(report, _rest, _frame_length), do: report

  # ── Helpers ─────────────────────────────────────────────────────────────────

  defp decode_fp32s(binary, count) do
    for <<f::float-little-32 <- binary_part(binary, 0, count * 4)>>, do: f
  end

  # Expands a bitfield byte into an 8-element list of 0/1 integers,
  # least-significant bit first (bit0 = joint 1, bit1 = joint 2, …).
  defp bits_to_list(byte) do
    for i <- 0..7, do: byte >>> i &&& 0x01
  end
end
