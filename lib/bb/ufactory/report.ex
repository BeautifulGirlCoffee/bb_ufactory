# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Report do
  import Bitwise

  @moduledoc """
  Pure parser for xArm auto-push report frames.

  The arm continuously pushes state frames on the report socket (port 30003
  for real-time, port 30001 for normal, port 30002 for rich). This module
  handles the real-time and normal report frame layouts as documented in
  xArm Developer Manual V1.10.0 and the Python SDK `report.py`.

  No network I/O is performed here. All functions are pure binary transforms.

  ## Frame Layout (real-time report, port 30003)

  Each push frame has a 4-byte length prefix:

  ```
  Bytes 0–3:    frame_length  u32 big-endian (total bytes of the frame INCLUDING
                              these 4 bytes — the payload is frame_length - 4
                              bytes)
  Byte  4:      state_mode    lower nibble = state, upper nibble = mode
  Bytes 5–6:    cmd_count     u16 big-endian
  Bytes 7–34:   angles        7× fp32 little-endian (radians, J1–J7)
  Bytes 35–58:  pose          6× fp32 little-endian (mm for x/y/z, rad for roll/pitch/yaw)
  Bytes 59–86:  torques       7× fp32 little-endian (Nm, J1–J7)
  —— frame_length >= 135 (F/T sensor enabled) ——
  Bytes 87–110:  ft_filtered  6× fp32 little-endian ([Fx, Fy, Fz, Tx, Ty, Tz])
  Bytes 111–134: ft_raw       6× fp32 little-endian ([Fx, Fy, Fz, Tx, Ty, Tz])
  ```

  The SDK's Python `report.py` uses `len(rx_data)` (the payload size) as its
  `length` field. Here we use the wire `frame_length` value from bytes 0–3,
  which includes the 4-byte header prefix. The minimum complete base frame is
  87 bytes total (4-byte prefix + 83-byte payload), so `frame_length >= 87`.

  The parser tolerates frames larger than expected (future firmware versions may
  add fields). Extra trailing bytes are ignored.

  ## Normal Report (port 30001, `report_type: :normal`)

  In addition to the base real-time fields, bytes 87+ contain controller status:
  - Byte  87: `mtbrake`    brake-state bitfield (J1=bit0 .. J7=bit6)
  - Byte  88: `mtable`     enable-state bitfield
  - Byte  89: `error_code` current error code (0 = no error)
  - Byte  90: `warn_code`  current warning code
  - Bytes 91–114: `tcp_offset`  6× fp32 LE (mm + rad)
  - Bytes 115–130: `tcp_load`   4× fp32 LE (kg, mm×3)
  - Byte 131: `collis_sens`  collision sensitivity (0–5)
  - Byte 132: `teach_sens`   teach sensitivity (0–5)

  Normal-report frames and F/T-extended real-time frames overlap in size and
  cannot be distinguished by `frame_length` alone (current firmware sends
  normal reports well over 135 bytes). The **caller** must state which stream
  the buffer came from via the `report_type` argument — `:realtime` for port
  30003 (the default), `:normal` for port 30001.

  ## Return Value

  `parse_report/2` returns:

  - `{:ok, report, rest}` — successfully parsed one frame; `rest` is any
    trailing data in the buffer (may contain the start of the next frame).
  - `{:more}` — not enough bytes in the buffer yet; accumulate more data.
  - `{:error, {:bad_frame_length, n}}` — the length prefix is impossible
    (below the 87-byte minimum or above the 512-byte sanity cap). The stream
    is desynchronized or corrupt; the caller should reset the connection
    rather than continue parsing.

  The `report` map always contains: `:state`, `:mode`, `:cmd_count`, `:angles`,
  `:pose`, `:torques`.

  Optional fields present only when the frame is large enough:
  - `:ft_filtered`, `:ft_raw` — 6-element float lists when
    `report_type: :realtime` and `frame_length >= 135`
  - `:mtbrake`, `:mtable` (8-element bit lists), `:error_code`, `:warn_code`,
    `:tcp_offset`, `:tcp_load`, `:collis_sens`, `:teach_sens` — when
    `report_type: :normal` and `frame_length >= 133`
  """

  # Minimum: 4-byte prefix + 83-byte base payload.
  @min_frame_length 87
  # Sanity cap: the largest known firmware report is ~245 bytes. Anything
  # beyond this is a corrupted length prefix, not a future field extension.
  @max_frame_length 512

  @doc """
  Parses one report frame from the start of `buffer`.

  `report_type` selects the frame layout: `:realtime` (port 30003, the
  default) or `:normal` (port 30001).

  Returns `{:ok, report_map, rest}` on success, `{:more}` if the buffer does
  not yet contain a complete frame, or `{:error, {:bad_frame_length, n}}` if
  the length prefix is impossible (desynchronized/corrupt stream).

  This function is designed to be called in a loop to drain the buffer:

      defp drain_buffer(buffer, state) do
        case BB.Ufactory.Report.parse_report(buffer) do
          {:ok, report, rest} ->
            state = handle_report(report, state)
            drain_buffer(rest, state)
          {:more} ->
            %{state | buffer: buffer}
          {:error, reason} ->
            reset_report_connection(reason, state)
        end
      end
  """
  @spec parse_report(binary(), :realtime | :normal) ::
          {:ok, map(), binary()} | {:more} | {:error, {:bad_frame_length, non_neg_integer()}}
  def parse_report(buffer, report_type \\ :realtime)

  def parse_report(buffer, _report_type) when byte_size(buffer) < 4, do: {:more}

  def parse_report(<<frame_length::32, _rest::binary>>, _report_type)
      when frame_length < @min_frame_length or frame_length > @max_frame_length do
    {:error, {:bad_frame_length, frame_length}}
  end

  def parse_report(<<frame_length::32, rest::binary>>, report_type) do
    # frame_length is the TOTAL frame size including the 4-byte length prefix
    # (confirmed from Python SDK: report_size = bytes_to_u32(buffer[0:4]),
    #  then data = buffer[:report_size] which includes the 4 prefix bytes).
    # So the payload to parse is frame_length - 4 bytes.
    payload_length = frame_length - 4

    if byte_size(rest) < payload_length do
      {:more}
    else
      <<payload::binary-size(^payload_length), tail::binary>> = rest
      report = parse_payload(payload, frame_length, report_type)
      {:ok, report, tail}
    end
  end

  # ── Payload parsing ─────────────────────────────────────────────────────────

  defp parse_payload(payload, frame_length, report_type) do
    # Base fields — always present (payload >= 83 == 3 + 28 + 24 + 28,
    # guaranteed by the @min_frame_length guard in parse_report/2).
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

    case report_type do
      :realtime ->
        report
        |> maybe_parse_ft_filtered(rest, frame_length)
        |> maybe_parse_ft_raw(rest, frame_length)

      :normal ->
        maybe_parse_normal(report, rest, frame_length)
    end
  end

  # Force-torque filtered: frame bytes 87–110 (fp32 LE × 6 = 24 bytes).
  # Present in real-time frames of 135+ bytes (port 30003 with the F/T
  # sensor enabled).
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

  # Normal report fields: frame bytes 87–132 (port 30001, frame_length >= 133).
  # Only reached when the caller declares the stream `:normal`; these bytes
  # occupy the same positions as ft_filtered/ft_raw in the real-time layout.
  defp maybe_parse_normal(report, rest, frame_length) when frame_length >= 133 do
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
