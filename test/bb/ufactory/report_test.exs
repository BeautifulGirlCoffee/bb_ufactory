# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ReportTest do
  use ExUnit.Case, async: true

  import Bitwise

  alias BB.Ufactory.Report

  # ── Frame construction helpers ───────────────────────────────────────────────

  # Encodes a float list as little-endian fp32 bytes.
  defp fp32s(floats), do: for(f <- floats, into: <<>>, do: <<f::float-little-32>>)

  # Constructs the common base payload (83 bytes of content, not including
  # the 4-byte frame_length prefix):
  #   Byte 0:     state_mode (lower nibble = state, upper nibble = mode)
  #   Bytes 1-2:  cmd_count (u16 BE)
  #   Bytes 3-30: angles J1–J7 (7× fp32 LE)
  #   Bytes 31-54: pose x/y/z/roll/pitch/yaw (6× fp32 LE)
  #   Bytes 55-82: torques J1–J7 (7× fp32 LE)
  defp base_payload(state, mode, cmd_count, angles, pose, torques) do
    state_mode = mode <<< 4 ||| (state &&& 0x0F)
    <<state_mode::8, cmd_count::16>> <> fp32s(angles) <> fp32s(pose) <> fp32s(torques)
  end

  # Wraps a payload binary with the 4-byte frame_length prefix.
  # frame_length = total frame size (4-byte prefix + payload).
  defp wrap(payload) do
    frame_length = byte_size(payload) + 4
    <<frame_length::32>> <> payload
  end

  # Builds a minimal 87-byte real-time report frame.
  defp build_87_byte_frame(opts \\ []) do
    state = Keyword.get(opts, :state, 0)
    mode = Keyword.get(opts, :mode, 0)
    cmd_count = Keyword.get(opts, :cmd_count, 0)
    angles = Keyword.get(opts, :angles, List.duplicate(0.0, 7))
    pose = Keyword.get(opts, :pose, List.duplicate(0.0, 6))
    torques = Keyword.get(opts, :torques, List.duplicate(0.0, 7))

    payload = base_payload(state, mode, cmd_count, angles, pose, torques)
    # payload is 83 bytes; total frame = 83 + 4 = 87
    wrap(payload)
  end

  # Builds a 135-byte frame with ft_filtered and ft_raw appended.
  defp build_135_byte_frame(opts \\ []) do
    ft_filtered = Keyword.get(opts, :ft_filtered, List.duplicate(0.0, 6))
    ft_raw = Keyword.get(opts, :ft_raw, List.duplicate(0.0, 6))

    base = build_87_byte_frame(opts)
    # Strip the length prefix, append ft data, re-wrap.
    <<_len::32, payload::binary>> = base
    wrap(payload <> fp32s(ft_filtered) <> fp32s(ft_raw))
  end

  # Builds a 133-byte normal-report frame.
  defp build_133_byte_frame(opts \\ []) do
    mtbrake = Keyword.get(opts, :mtbrake, 0)
    mtable = Keyword.get(opts, :mtable, 0)
    error_code = Keyword.get(opts, :error_code, 0)
    warn_code = Keyword.get(opts, :warn_code, 0)
    tcp_offset = Keyword.get(opts, :tcp_offset, List.duplicate(0.0, 6))
    tcp_load = Keyword.get(opts, :tcp_load, List.duplicate(0.0, 4))
    collis_sens = Keyword.get(opts, :collis_sens, 3)
    teach_sens = Keyword.get(opts, :teach_sens, 2)

    base = build_87_byte_frame(opts)
    <<_len::32, base_payload::binary>> = base

    normal_ext =
      <<mtbrake::8, mtable::8, error_code::8, warn_code::8>> <>
        fp32s(tcp_offset) <>
        fp32s(tcp_load) <>
        <<collis_sens::8, teach_sens::8>>

    wrap(base_payload <> normal_ext)
  end

  # ── {:more} on truncated input ───────────────────────────────────────────────

  describe "parse_report/1 with insufficient data" do
    test "returns {:more} for empty binary" do
      assert Report.parse_report(<<>>) == {:more}
    end

    test "returns {:more} for 3-byte partial length prefix" do
      assert Report.parse_report(<<0, 0, 0>>) == {:more}
    end

    test "returns {:more} when length prefix is present but payload is incomplete" do
      # frame_length = 87, only 4 bytes (the header itself) present
      assert Report.parse_report(<<0, 0, 0, 87>>) == {:more}
    end

    test "returns {:more} when one byte short of a full frame" do
      frame = build_87_byte_frame()
      short = binary_part(frame, 0, byte_size(frame) - 1)
      assert Report.parse_report(short) == {:more}
    end
  end

  # ── 87-byte base frame ───────────────────────────────────────────────────────

  describe "parse_report/1 with an 87-byte real-time frame" do
    test "returns {:ok, report, rest}" do
      frame = build_87_byte_frame()
      assert {:ok, _report, <<>>} = Report.parse_report(frame)
    end

    test "total frame size is 87 bytes" do
      frame = build_87_byte_frame()
      assert byte_size(frame) == 87
    end

    test "extracts state from lower nibble of state_mode byte" do
      frame = build_87_byte_frame(state: 2, mode: 0)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.state == 2
    end

    test "extracts mode from upper nibble of state_mode byte" do
      frame = build_87_byte_frame(state: 0, mode: 1)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.mode == 1
    end

    test "extracts state and mode independently" do
      frame = build_87_byte_frame(state: 3, mode: 2)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.state == 3
      assert report.mode == 2
    end

    test "extracts cmd_count" do
      frame = build_87_byte_frame(cmd_count: 42)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.cmd_count == 42
    end

    test "extracts all 7 joint angles" do
      angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
      frame = build_87_byte_frame(angles: angles)
      {:ok, report, _} = Report.parse_report(frame)

      for {expected, actual} <- Enum.zip(angles, report.angles) do
        assert_in_delta actual, expected, 1.0e-5
      end
    end

    test "extracts all 6 Cartesian pose values" do
      pose = [100.0, 200.0, 300.0, 0.1, 0.2, 0.3]
      frame = build_87_byte_frame(pose: pose)
      {:ok, report, _} = Report.parse_report(frame)

      for {expected, actual} <- Enum.zip(pose, report.pose) do
        assert_in_delta actual, expected, 0.001
      end
    end

    test "extracts all 7 torque values" do
      torques = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7]
      frame = build_87_byte_frame(torques: torques)
      {:ok, report, _} = Report.parse_report(frame)

      for {expected, actual} <- Enum.zip(torques, report.torques) do
        assert_in_delta actual, expected, 0.001
      end
    end

    test "does not include :ft_filtered in a base frame" do
      {:ok, report, _} = Report.parse_report(build_87_byte_frame())
      refute Map.has_key?(report, :ft_filtered)
    end

    test "does not include :ft_raw in a base frame" do
      {:ok, report, _} = Report.parse_report(build_87_byte_frame())
      refute Map.has_key?(report, :ft_raw)
    end

    test "does not include :error_code in a base frame" do
      {:ok, report, _} = Report.parse_report(build_87_byte_frame())
      refute Map.has_key?(report, :error_code)
    end
  end

  # ── 135-byte frame with ft data ──────────────────────────────────────────────

  describe "parse_report/1 with a 135-byte ft-extended frame" do
    test "total frame size is 135 bytes" do
      frame = build_135_byte_frame()
      assert byte_size(frame) == 135
    end

    test "returns {:ok, report, rest}" do
      frame = build_135_byte_frame()
      assert {:ok, _report, <<>>} = Report.parse_report(frame)
    end

    test "includes :ft_filtered with 6 values" do
      ft = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
      frame = build_135_byte_frame(ft_filtered: ft)
      {:ok, report, _} = Report.parse_report(frame)

      assert Map.has_key?(report, :ft_filtered)
      assert length(report.ft_filtered) == 6

      for {expected, actual} <- Enum.zip(ft, report.ft_filtered) do
        assert_in_delta actual, expected, 0.001
      end
    end

    test "includes :ft_raw with 6 values" do
      raw = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
      frame = build_135_byte_frame(ft_raw: raw)
      {:ok, report, _} = Report.parse_report(frame)

      assert Map.has_key?(report, :ft_raw)
      assert length(report.ft_raw) == 6

      for {expected, actual} <- Enum.zip(raw, report.ft_raw) do
        assert_in_delta actual, expected, 0.001
      end
    end

    test "ft_filtered and ft_raw are independent" do
      filtered = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
      raw = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
      frame = build_135_byte_frame(ft_filtered: filtered, ft_raw: raw)
      {:ok, report, _} = Report.parse_report(frame)

      assert Enum.all?(report.ft_filtered, &assert_in_delta(&1, 1.0, 0.001))
      assert Enum.all?(report.ft_raw, &assert_in_delta(&1, 2.0, 0.001))
    end

    test "base fields are still correctly parsed" do
      angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
      frame = build_135_byte_frame(state: 1, mode: 0, angles: angles)
      {:ok, report, _} = Report.parse_report(frame)

      assert report.state == 1

      for {expected, actual} <- Enum.zip(angles, report.angles) do
        assert_in_delta actual, expected, 1.0e-5
      end
    end
  end

  # ── 133-byte normal report frame ─────────────────────────────────────────────

  describe "parse_report/1 with a 133-byte normal-report frame" do
    test "total frame size is 133 bytes" do
      frame = build_133_byte_frame()
      assert byte_size(frame) == 133
    end

    test "includes :error_code" do
      frame = build_133_byte_frame(error_code: 7)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.error_code == 7
    end

    test "includes :warn_code" do
      frame = build_133_byte_frame(warn_code: 2)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.warn_code == 2
    end

    test "includes :mtbrake as an 8-element bit list" do
      # Set bits 0 and 2 (joints 1 and 3 have brakes engaged)
      mtbrake_byte = 0b00000101
      frame = build_133_byte_frame(mtbrake: mtbrake_byte)
      {:ok, report, _} = Report.parse_report(frame)

      assert length(report.mtbrake) == 8
      assert Enum.at(report.mtbrake, 0) == 1
      assert Enum.at(report.mtbrake, 1) == 0
      assert Enum.at(report.mtbrake, 2) == 1
    end

    test "includes :tcp_offset with 6 values" do
      tcp_offset = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
      frame = build_133_byte_frame(tcp_offset: tcp_offset)
      {:ok, report, _} = Report.parse_report(frame)

      for {expected, actual} <- Enum.zip(tcp_offset, report.tcp_offset) do
        assert_in_delta actual, expected, 0.001
      end
    end

    test "includes :tcp_load with 4 values" do
      tcp_load = [1.5, 10.0, 0.0, 5.0]
      frame = build_133_byte_frame(tcp_load: tcp_load)
      {:ok, report, _} = Report.parse_report(frame)

      for {expected, actual} <- Enum.zip(tcp_load, report.tcp_load) do
        assert_in_delta actual, expected, 0.001
      end
    end

    test "includes :collis_sens and :teach_sens" do
      frame = build_133_byte_frame(collis_sens: 3, teach_sens: 5)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.collis_sens == 3
      assert report.teach_sens == 5
    end

    test "does NOT include :ft_filtered (normal report is not a real-time ft frame)" do
      frame = build_133_byte_frame()
      {:ok, report, _} = Report.parse_report(frame)
      refute Map.has_key?(report, :ft_filtered)
    end
  end

  # ── Trailing bytes / streaming buffer behaviour ──────────────────────────────

  describe "parse_report/1 with extra trailing bytes" do
    test "returns remaining bytes in rest" do
      frame = build_87_byte_frame()
      extra = <<0xDE, 0xAD, 0xBE, 0xEF>>
      {:ok, _report, rest} = Report.parse_report(frame <> extra)
      assert rest == extra
    end

    test "can parse two back-to-back frames" do
      frame1 = build_87_byte_frame(state: 1)
      frame2 = build_87_byte_frame(state: 2)
      buffer = frame1 <> frame2

      {:ok, report1, rest1} = Report.parse_report(buffer)
      {:ok, report2, rest2} = Report.parse_report(rest1)

      assert report1.state == 1
      assert report2.state == 2
      assert rest2 == <<>>
    end

    test "gracefully handles a 150-byte frame (larger than base, smaller than 135+4)" do
      # Build an artificial oversized frame that the parser should consume
      # without crashing (future-proofing: extra fields are ignored).
      # Total = 4 (prefix) + 83 (base payload) + 63 (extra) = 150 bytes.
      base = build_87_byte_frame()
      <<_len::32, payload::binary>> = base
      big_payload = payload <> :binary.copy(<<0>>, 63)
      frame = wrap(big_payload)

      assert byte_size(frame) == 150
      assert {:ok, _report, <<>>} = Report.parse_report(frame)
    end
  end

  # ── State / mode nibble extraction ───────────────────────────────────────────

  describe "state and mode nibble extraction" do
    for {state, mode} <- [{0, 0}, {5, 3}, {1, 2}, {3, 1}] do
      test "state=#{state}, mode=#{mode} extracted correctly" do
        frame = build_87_byte_frame(state: unquote(state), mode: unquote(mode))
        {:ok, report, _} = Report.parse_report(frame)
        assert report.state == unquote(state)
        assert report.mode == unquote(mode)
      end
    end

    test "state occupies the lower nibble (bits 0–3)" do
      # state=0xF (15) would need 4 bits; arm uses 0..5, but test nibble masking
      frame = build_87_byte_frame(state: 5, mode: 3)
      {:ok, report, _} = Report.parse_report(frame)
      assert report.state == 5
      assert report.mode == 3
    end
  end
end
