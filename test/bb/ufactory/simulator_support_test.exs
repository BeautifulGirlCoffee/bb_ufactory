# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.SimulatorSupportTest do
  @moduledoc """
  Unit tests for the `BB.Ufactory.Simulator` / `BB.Ufactory.SimulatorCase`
  public API that do NOT require a running simulator: offline behavior,
  option handling, and helper semantics. The live-firmware coverage lives
  in `simulator_test.exs` (`:simulator` tag).
  """

  use ExUnit.Case, async: false

  alias BB.Ufactory.Model
  alias BB.Ufactory.Simulator
  alias BB.Ufactory.SimulatorCase

  # Reserves a TCP port and closes it, so connections to it are refused.
  defp closed_port do
    {:ok, listen} = :gen_tcp.listen(0, [:binary])
    {:ok, port} = :inet.port(listen)
    :gen_tcp.close(listen)
    port
  end

  describe "with no firmware listening" do
    setup do
      %{opts: [host: "127.0.0.1", command_port: closed_port(), report_port: closed_port()]}
    end

    test "available?/1 returns false", %{opts: opts} do
      refute Simulator.available?(opts)
    end

    test "wait_until_ready/1 times out with an error tuple", %{opts: opts} do
      assert {:error, :timeout} = Simulator.wait_until_ready(opts ++ [timeout: 0])
    end

    test "connect_command/1 and connect_report/1 return error tuples", %{opts: opts} do
      assert {:error, :econnrefused} = Simulator.connect_command(opts)
      assert {:error, :econnrefused} = Simulator.connect_report(opts)
    end

    test "current_state/1 returns an error tuple", %{opts: opts} do
      assert {:error, :econnrefused} = Simulator.current_state(opts)
    end
  end

  describe "report helpers against a local stub server" do
    # A minimal in-test "firmware" proving the helpers work over real
    # sockets without Docker: serves canned report frames.
    defp report_frame(j1_angle) do
      angles =
        for a <- [j1_angle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], into: <<>>, do: <<a::float-little-32>>

      pose = for p <- List.duplicate(0.0, 6), into: <<>>, do: <<p::float-little-32>>
      torques = for t <- List.duplicate(0.0, 7), into: <<>>, do: <<t::float-little-32>>
      payload = <<0::8, 0::16>> <> angles <> pose <> torques
      <<byte_size(payload) + 4::32>> <> payload
    end

    defp serve_frames(frames) do
      {:ok, listen} = :gen_tcp.listen(0, [:binary, active: false, reuseaddr: true])
      {:ok, port} = :inet.port(listen)

      spawn_link(fn ->
        {:ok, client} = :gen_tcp.accept(listen, 5_000)
        Enum.each(frames, &:gen_tcp.send(client, &1))
        # Hold the socket open briefly so the reader drains everything.
        Process.sleep(500)
        :gen_tcp.close(client)
        :gen_tcp.close(listen)
      end)

      port
    end

    test "read_report/3 parses a frame and threads the remainder" do
      port = serve_frames([report_frame(0.1) <> report_frame(0.2)])
      {:ok, socket} = Simulator.connect_report(host: "127.0.0.1", report_port: port)

      assert {:ok, first, rest} = Simulator.read_report(socket)
      assert_in_delta hd(first.angles), 0.1, 1.0e-5

      assert {:ok, second, <<>>} = Simulator.read_report(socket, rest)
      assert_in_delta hd(second.angles), 0.2, 1.0e-5

      :gen_tcp.close(socket)
    end

    test "await_report/4 skips frames until the condition holds" do
      port = serve_frames([report_frame(0.0), report_frame(0.1), report_frame(0.5)])
      {:ok, socket} = Simulator.connect_report(host: "127.0.0.1", report_port: port)

      assert {:ok, frame, _rest} =
               Simulator.await_report(socket, <<>>, 2_000, fn r ->
                 hd(r.angles) > 0.4
               end)

      assert_in_delta hd(frame.angles), 0.5, 1.0e-5
      :gen_tcp.close(socket)
    end

    test "read_report/3 surfaces a desynchronized stream as an error" do
      port = serve_frames([<<0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0>>])
      {:ok, socket} = Simulator.connect_report(host: "127.0.0.1", report_port: port)

      assert {:error, {:bad_frame_length, 0xFFFFFFFF}} = Simulator.read_report(socket)
      :gen_tcp.close(socket)
    end
  end

  describe "SimulatorCase model helpers" do
    test "sim_model/0 defaults to :xarm6 and sim_joints/0 follows the model" do
      # SIM_MODEL is unset in the default unit-test run.
      case System.get_env("SIM_MODEL") do
        nil ->
          assert SimulatorCase.sim_model() == :xarm6
          assert SimulatorCase.sim_joints() == 6

        model ->
          assert SimulatorCase.sim_model() == String.to_existing_atom(model)

          assert SimulatorCase.sim_joints() ==
                   Model.joint_count(SimulatorCase.sim_model())
      end
    end
  end
end
