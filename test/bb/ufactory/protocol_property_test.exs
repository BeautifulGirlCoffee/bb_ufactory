# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ProtocolPropertyTest do
  @moduledoc """
  Property-based round-trips for the wire layer.

  The example-based tests pin known frames byte-for-byte; these properties
  cover the input space the examples cannot: arbitrary values through the
  fp32 encoders, and — most importantly — arbitrary TCP segmentation. The
  arm streams responses and reports over TCP with no message boundaries, so
  every parser must produce identical results no matter where the stream is
  split.
  """
  use ExUnit.Case, async: true
  use ExUnitProperties

  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Report

  # Values a robotics payload actually carries: angles, mm offsets, speeds.
  # Kept within fp32's exactly-representable magnitude so the round-trip
  # tolerance below is meaningful.
  defp payload_float, do: StreamData.float(min: -1.0e6, max: 1.0e6)

  describe "fp32 encoding" do
    property "encode_fp32 |> decode_fp32 round-trips within fp32 precision" do
      check all(f <- payload_float()) do
        decoded = Protocol.decode_fp32(Protocol.encode_fp32(f))
        # fp32 has a 24-bit mantissa: relative error is bounded by 2^-24.
        assert abs(decoded - f) <= max(abs(f) * 1.2e-7, 1.0e-30)
      end
    end

    property "encode_fp32s |> decode_fp32s preserves length and order" do
      check all(floats <- StreamData.list_of(payload_float(), max_length: 16)) do
        bin = Protocol.encode_fp32s(floats)
        assert byte_size(bin) == length(floats) * 4

        decoded = Protocol.decode_fp32s(bin, length(floats))

        for {original, roundtripped} <- Enum.zip(floats, decoded) do
          assert abs(roundtripped - original) <= max(abs(original) * 1.2e-7, 1.0e-30)
        end
      end
    end
  end

  describe "command-frame parsing" do
    # A response body is register + status + params, so any generated params
    # binary with >= 1 byte makes a parseable response when framed.
    defp response_frame do
      StreamData.bind(
        {StreamData.integer(0..65_535), StreamData.byte(), StreamData.byte(),
         StreamData.binary(max_length: 40)},
        fn {txn, register, status, params} ->
          frame = Protocol.build_frame(txn, register, <<status>> <> params)
          StreamData.constant({frame, register, status, params})
        end
      )
    end

    property "parse_response inverts build_frame for any register/status/params" do
      check all({frame, register, status, params} <- response_frame()) do
        assert {:ok, {^register, ^status, ^params}, <<>>} = Protocol.parse_response(frame)
      end
    end

    property "a frame stream parses identically at every split point" do
      check all(
              frames <- StreamData.list_of(response_frame(), min_length: 1, max_length: 4),
              seed <- StreamData.integer(0..1_000_000)
            ) do
        expected = Enum.map(frames, fn {_frame, r, s, p} -> {r, s, p} end)
        stream = Enum.map_join(frames, <<>>, fn {frame, _r, _s, _p} -> frame end)

        chunks = split_at_random_points(stream, seed)
        assert parse_chunked(chunks, &Protocol.parse_response/1) == expected
      end
    end
  end

  describe "report-frame parsing" do
    # An 87-byte real-time report: u32 length prefix (inclusive) + state/mode
    # byte + cmd count + 7 angles + 6 pose + 7 torques.
    defp report_frame do
      StreamData.bind(
        {StreamData.list_of(payload_float(), length: 7),
         StreamData.list_of(payload_float(), length: 6),
         StreamData.list_of(payload_float(), length: 7)},
        fn {angles, pose, torques} ->
          payload =
            <<0::8, 0::16>> <>
              Protocol.encode_fp32s(angles) <>
              Protocol.encode_fp32s(pose) <>
              Protocol.encode_fp32s(torques)

          frame = <<byte_size(payload) + 4::32>> <> payload
          StreamData.constant({frame, angles})
        end
      )
    end

    property "a report stream parses identically at every split point" do
      check all(
              frames <- StreamData.list_of(report_frame(), min_length: 1, max_length: 3),
              seed <- StreamData.integer(0..1_000_000)
            ) do
        stream = Enum.map_join(frames, <<>>, fn {frame, _angles} -> frame end)
        chunks = split_at_random_points(stream, seed)

        reports = parse_chunked(chunks, &Report.parse_report/1)

        for {{_frame, angles}, report} <- Enum.zip(frames, reports) do
          for {expected, actual} <- Enum.zip(angles, report.angles) do
            assert abs(actual - expected) <= max(abs(expected) * 1.2e-7, 1.0e-30)
          end
        end
      end
    end
  end

  # ── Helpers ─────────────────────────────────────────────────────────────────

  # Deterministically splits a binary into chunks at pseudo-random points
  # derived from `seed`, covering 1-byte slivers through whole-buffer chunks.
  defp split_at_random_points(binary, seed) do
    do_split(binary, :rand.seed_s(:exsss, {seed, 17, 29}), [])
  end

  defp do_split(<<>>, _rand_state, acc), do: Enum.reverse(acc)

  defp do_split(binary, rand_state, acc) do
    {len, rand_state} = :rand.uniform_s(byte_size(binary), rand_state)
    <<chunk::binary-size(^len), rest::binary>> = binary
    do_split(rest, rand_state, [chunk | acc])
  end

  # Feeds chunks through a `{:ok, result, rest} | {:more}` parser the way the
  # controller's buffer loop does, returning every parsed result in order.
  defp parse_chunked(chunks, parse) do
    {results, leftover} =
      Enum.reduce(chunks, {[], <<>>}, fn chunk, {results, buffer} ->
        drain(buffer <> chunk, parse, results)
      end)

    assert leftover == <<>>, "unparsed bytes remained: #{inspect(leftover)}"
    Enum.reverse(results)
  end

  defp drain(buffer, parse, results) do
    case parse.(buffer) do
      {:ok, result, rest} -> drain(rest, parse, [result | results])
      {:more} -> {results, buffer}
    end
  end
end
