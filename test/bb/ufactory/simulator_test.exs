# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.SimulatorTest do
  @moduledoc """
  Integration tests against the UFACTORY firmware simulator running in Docker.

  Unlike `sim_test.exs` (which mocks the controller with bb's kinematic
  simulation), these tests exercise the REAL `BB.Ufactory.Controller` and the
  real wire protocol against UFACTORY's own firmware — the closest thing to
  hardware testing without an arm.

  Start the simulator first, then include the tag:

      scripts/sim.sh start
      mix test --include simulator
      scripts/sim.sh stop

  Configure the simulator address via `SIM_HOST` (default `127.0.0.1`).

  ## Simulator quirks (observed against danielwang123321/uf-ubuntu-docker)

  - `MOTION_EN` (0x0B) never gets a response — enable must be sent
    fire-and-forget. The controller's arm sequence already works this way.
  - Response status bytes carry SDK-style flags (0x20 = warning pending,
    0x40 = error pending) rather than plain 0.
  - `IS_TCP_LIMIT`/`IS_JOINT_LIMIT` check the *configured* boundary, not
    reachability, so they answer `:within_limits` for everything by default.
    True reachability is probed via the FK(IK(pose)) round-trip instead.
  """

  use ExUnit.Case, async: false

  @moduletag :simulator
  # Everything runs over real TCP against emulated firmware; be generous.
  @moduletag timeout: 120_000

  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Report

  @host System.get_env("SIM_HOST", "127.0.0.1")
  @cmd_port 502
  @report_port 30_003

  # ── Raw socket helpers ──────────────────────────────────────────────────────

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

  # Sends a frame without waiting for a response (see MOTION_EN quirk), then
  # drains anything pending so the next send_and_recv reads its own reply.
  defp send_fire_and_forget(socket, frame) do
    :ok = :gen_tcp.send(socket, frame)
    drain(socket)
  end

  defp drain(socket) do
    case :gen_tcp.recv(socket, 0, 200) do
      {:ok, _} -> drain(socket)
      {:error, _} -> :ok
    end
  end

  defp ensure_ready(socket) do
    send_fire_and_forget(socket, Protocol.cmd_clean_error(0))
    send_fire_and_forget(socket, Protocol.cmd_clean_warn(1))
    send_fire_and_forget(socket, Protocol.cmd_enable(2, true))
    send_fire_and_forget(socket, Protocol.cmd_set_mode(3, 0))
    send_fire_and_forget(socket, Protocol.cmd_set_state(4, 0))
    Process.sleep(300)
    drain(socket)
  end

  defp recv_report(socket, buffer \\ <<>>) do
    case Report.parse_report(buffer) do
      {:ok, report, rest} ->
        {report, rest}

      {:more} ->
        {:ok, more} = :gen_tcp.recv(socket, 0, 2_000)
        recv_report(socket, buffer <> more)
    end
  end

  # Polls the report stream until `fun` returns true or `deadline_ms` passes.
  # `buffer` must be the unconsumed remainder from the previous recv_report —
  # starting fresh mid-stream would misalign frame boundaries.
  defp await_report(socket, buffer, deadline_ms, fun) do
    deadline = System.monotonic_time(:millisecond) + deadline_ms
    do_await_report(socket, buffer, deadline, fun)
  end

  defp do_await_report(socket, buffer, deadline, fun) do
    {report, rest} = recv_report(socket, buffer)

    cond do
      fun.(report) -> {:ok, report, rest}
      System.monotonic_time(:millisecond) > deadline -> {:timeout, report}
      true -> do_await_report(socket, rest, deadline, fun)
    end
  end

  # ── Connection & heartbeat ──────────────────────────────────────────────────

  describe "command socket" do
    test "heartbeat gets a GET_STATE response and the connection stays usable" do
      socket = connect_cmd()

      assert {:ok, {0x0D, _status, <<_state::8>>}, _rest} =
               send_and_recv(socket, Protocol.heartbeat())

      assert {:ok, {0x0F, _status, <<_err::8, _warn::8>>}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_error(1))

      :gen_tcp.close(socket)
    end

    test "connection survives 12 seconds of idle with 1 Hz heartbeats" do
      # Validates the reworked heartbeat frame as a keep-alive: the firmware
      # drops idle command connections after ~10 s, so surviving 12 s of
      # heartbeat-only traffic proves the frame is accepted.
      socket = connect_cmd()

      for _ <- 1..12 do
        :ok = :gen_tcp.send(socket, Protocol.heartbeat())
        Process.sleep(1_000)
        drain(socket)
      end

      assert {:ok, {0x0F, _status, _params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_error(99))

      :gen_tcp.close(socket)
    end
  end

  describe "report socket" do
    test "streams parseable real-time frames with sane joint data" do
      socket = connect_report()

      # Parse several consecutive frames to prove framing stays in sync.
      {report, rest} = recv_report(socket)

      reports =
        Enum.reduce(1..4, {[report], rest}, fn _, {acc, buf} ->
          {r, rest} = recv_report(socket, buf)
          {[r | acc], rest}
        end)
        |> elem(0)

      for r <- reports do
        assert length(r.angles) == 7
        assert length(r.pose) == 6
        Enum.each(r.angles, fn a -> assert abs(a) <= 2 * :math.pi() + 0.1 end)
      end

      :gen_tcp.close(socket)
    end
  end

  # ── Kinematics / workspace API against real firmware ───────────────────────

  describe "firmware kinematics" do
    setup do
      socket = connect_cmd()
      ensure_ready(socket)
      on_exit(fn -> :gen_tcp.close(socket) end)
      %{socket: socket}
    end

    test "FK of the zero pose matches the xArm6 kinematic model", %{socket: socket} do
      assert {:ok, {0x2C, _status, params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_fk(0, List.duplicate(0.0, 6)))

      assert {:ok, {x, y, z, roll, _pitch, _yaw}} = Protocol.parse_fk_response(params)

      # xArm6 all-zeros TCP pose: x = 207 mm, z = 112 mm, tool pointing down.
      assert_in_delta x, 207.0, 1.0
      assert_in_delta y, 0.0, 1.0
      assert_in_delta z, 112.0, 1.0
      assert_in_delta abs(roll), :math.pi(), 0.01
    end

    test "IK round-trips through FK for a reachable pose", %{socket: socket} do
      pose = {300.0, 50.0, 300.0, :math.pi(), 0.0, 0.0}

      assert {:ok, {0x2B, _status, ik_params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_ik(0, pose))

      assert {:ok, angles} = Protocol.parse_ik_response(ik_params)

      assert {:ok, {0x2C, _status, fk_params}, _rest} =
               send_and_recv(socket, Protocol.cmd_get_fk(1, angles))

      assert {:ok, {x, y, z, _r, _p, _yw}} = Protocol.parse_fk_response(fk_params)

      {ex, ey, ez, _, _, _} = pose
      assert_in_delta x, ex, 5.0
      assert_in_delta y, ey, 5.0
      assert_in_delta z, ez, 5.0
    end

    test "FK(IK(pose)) round-trip distinguishes reachable from unreachable poses",
         %{socket: socket} do
      # The reachability recipe for workspace mapping: a pose is reachable iff
      # feeding the IK solution back through FK lands near the original pose.
      # (The firmware returns unclamped garbage angles for unreachable poses.)
      round_trip_error = fn {x, y, z, _, _, _} = pose ->
        {:ok, {0x2B, _s1, ik}, _} = send_and_recv(socket, Protocol.cmd_get_ik(0, pose))
        {:ok, angles} = Protocol.parse_ik_response(ik)
        {:ok, {0x2C, _s2, fk}, _} = send_and_recv(socket, Protocol.cmd_get_fk(1, angles))
        {:ok, {fx, fy, fz, _, _, _}} = Protocol.parse_fk_response(fk)
        :math.sqrt((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2)
      end

      reachable = {300.0, 0.0, 300.0, :math.pi(), 0.0, 0.0}
      unreachable = {2000.0, 0.0, 300.0, :math.pi(), 0.0, 0.0}

      assert round_trip_error.(reachable) < 5.0
      assert round_trip_error.(unreachable) > 100.0
    end

    test "limit checks respond with parseable results", %{socket: socket} do
      # The simulator's limit checks validate against the *configured*
      # boundary (none by default), so only protocol-level correctness is
      # asserted here; boundary semantics are a physical-hardware test.
      assert {:ok, {0x2E, _status, params}, _rest} =
               send_and_recv(
                 socket,
                 Protocol.cmd_tcp_limit_check(0, {300.0, 0.0, 300.0, :math.pi(), 0.0, 0.0})
               )

      assert {:ok, verdict} = Protocol.parse_limit_check(params)
      assert verdict in [:within_limits, :beyond_limits]

      assert {:ok, {0x2D, _status, jparams}, _rest} =
               send_and_recv(socket, Protocol.cmd_joint_limit_check(1, List.duplicate(0.0, 6)))

      assert {:ok, jverdict} = Protocol.parse_limit_check(jparams)
      assert jverdict in [:within_limits, :beyond_limits]
    end
  end

  # ── Motion over the real protocol ───────────────────────────────────────────

  describe "joint motion" do
    test "moves J1 and observes it in the report stream" do
      cmd = connect_cmd()
      report = connect_report()
      ensure_ready(cmd)

      {initial, rest} = recv_report(report)
      initial_angles = Enum.take(initial.angles, 6)
      target_j1 = hd(initial_angles) + 0.3
      target_angles = List.replace_at(initial_angles, 0, target_j1)

      :ok = :gen_tcp.send(cmd, Protocol.cmd_move_joints(10, target_angles, 0.8, 8.0))

      assert {:ok, _report, rest} =
               await_report(report, rest, 20_000, fn r ->
                 abs(hd(r.angles) - target_j1) < 0.02
               end)

      # Return to the initial pose for test isolation.
      :ok = :gen_tcp.send(cmd, Protocol.cmd_move_joints(11, initial_angles, 0.8, 8.0))

      assert {:ok, _report, _rest} =
               await_report(report, rest, 20_000, fn r ->
                 abs(hd(r.angles) - hd(initial_angles)) < 0.02
               end)

      :gen_tcp.close(cmd)
      :gen_tcp.close(report)
    end
  end

  # ── Full stack: real BB.Ufactory.Controller against the simulator ──────────

  defmodule Robot do
    @moduledoc false
    use BB
    import BB.Unit

    controllers do
      controller(
        :xarm,
        {BB.Ufactory.Controller,
         host: System.get_env("SIM_HOST", "127.0.0.1"), model: :xarm6, loop_hz: 50}
      )
    end

    topology do
      link :base do
        joint :j1 do
          type(:revolute)

          limit do
            lower(~u(-360 degree))
            upper(~u(360 degree))
            effort(~u(50 newton_meter))
            velocity(~u(180 degree_per_second))
          end

          actuator(:j1_motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm})

          link :link1 do
          end
        end
      end
    end
  end

  describe "full controller stack against the simulator" do
    setup do
      {:ok, _pid} = Robot.start_link()

      on_exit(fn ->
        try do
          BB.Safety.disarm(Robot)
        catch
          :exit, _ -> :ok
        end

        if Process.whereis(Robot) do
          try do
            Supervisor.stop(Robot, :normal, 5_000)
          catch
            :exit, _ -> :ok
          end
        end
      end)

      :ok
    end

    test "controller connects, publishes JointState from the live report stream" do
      BB.subscribe(Robot, [:sensor, :xarm])

      assert_receive {:bb, [:sensor, :xarm],
                      %BB.Message{payload: %BB.Message.Sensor.JointState{} = js}},
                     5_000

      assert length(js.positions) == 6
    end

    test "arms, moves J1 through the actuator, and observes convergence" do
      BB.subscribe(Robot, [:sensor, :xarm])

      assert :ok = BB.Safety.arm(Robot)
      # Let the arm sequence (clean/enable/mode/state) land in the firmware.
      Process.sleep(1_000)

      # Current J1 from the live stream, then command a relative move.
      assert_receive {:bb, _, %BB.Message{payload: %BB.Message.Sensor.JointState{} = js0}},
                     5_000

      target = hd(js0.positions) + 0.2
      BB.Actuator.set_position!(Robot, :j1_motor, target)

      assert await_joint(1, target, 30_000), "J1 did not converge to #{target}"

      assert :ok = BB.Safety.disarm(Robot)
    end

    defp await_joint(_index, _target, deadline_ms) when deadline_ms <= 0, do: false

    defp await_joint(index, target, deadline_ms) do
      receive do
        {:bb, _, %BB.Message{payload: %BB.Message.Sensor.JointState{positions: positions}}} ->
          if abs(Enum.at(positions, index - 1) - target) < 0.02 do
            true
          else
            await_joint(index, target, deadline_ms - 10)
          end
      after
        1_000 -> await_joint(index, target, deadline_ms - 1_000)
      end
    end
  end
end
