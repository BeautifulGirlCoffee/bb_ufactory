# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ControllerTest do
  use ExUnit.Case, async: false
  use Mimic

  import Bitwise

  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.StateMachine.Transition
  alias BB.Ufactory.Controller
  alias BB.Ufactory.Message.CartesianPose
  alias BB.Ufactory.Message.Wrench
  alias BB.Ufactory.Protocol

  setup :verify_on_exit!

  # ── TCP socket pair helper ───────────────────────────────────────────────────
  # Creates two connected TCP sockets over the loopback interface.
  # `client` is the controller's end; `server` is the test's end.
  defp tcp_pair(active \\ false) do
    {:ok, listen} =
      :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

    {:ok, port} = :inet.port(listen)
    {:ok, client} = :gen_tcp.connect(~c"127.0.0.1", port, [:binary, active: active, packet: :raw])
    {:ok, server} = :gen_tcp.accept(listen, 1_000)
    :gen_tcp.close(listen)
    {client, server}
  end

  # Receives all pending bytes from `socket` within `timeout_ms`.
  defp recv_all(socket, timeout_ms \\ 100) do
    :gen_tcp.recv(socket, 0, timeout_ms)
  end

  # Accumulates TCP data until at least `min_bytes` have arrived or `timeout_ms` elapses.
  defp recv_at_least(socket, min_bytes, timeout_ms \\ 500) do
    recv_at_least(socket, min_bytes, timeout_ms, <<>>)
  end

  defp recv_at_least(_socket, min_bytes, _timeout_ms, acc)
       when byte_size(acc) >= min_bytes do
    {:ok, acc}
  end

  defp recv_at_least(socket, min_bytes, timeout_ms, acc) do
    case :gen_tcp.recv(socket, 0, timeout_ms) do
      {:ok, data} -> recv_at_least(socket, min_bytes, timeout_ms, acc <> data)
      {:error, _reason} when acc != <<>> -> {:ok, acc}
      {:error, reason} -> {:error, reason}
    end
  end

  # ── Report frame helpers ─────────────────────────────────────────────────────

  defp fp32s(floats), do: for(f <- floats, into: <<>>, do: <<f::float-little-32>>)

  defp build_87_byte_frame(opts \\ []) do
    state = Keyword.get(opts, :state, 0)
    mode = Keyword.get(opts, :mode, 0)
    cmd_count = Keyword.get(opts, :cmd_count, 0)
    angles = Keyword.get(opts, :angles, List.duplicate(0.0, 7))
    pose = Keyword.get(opts, :pose, List.duplicate(0.0, 6))
    torques = Keyword.get(opts, :torques, List.duplicate(0.0, 7))

    state_mode = mode <<< 4 ||| (state &&& 0x0F)
    payload = <<state_mode::8, cmd_count::16>> <> fp32s(angles) <> fp32s(pose) <> fp32s(torques)
    frame_length = byte_size(payload) + 4
    <<frame_length::32>> <> payload
  end

  # Builds a 135-byte real-time report frame that includes ft_filtered and ft_raw
  # fields (bytes 87–110 and 111–134 respectively). This is the format sent by the
  # arm when the F/T sensor is enabled.
  defp build_135_byte_frame(opts \\ []) do
    state = Keyword.get(opts, :state, 0)
    mode = Keyword.get(opts, :mode, 0)
    cmd_count = Keyword.get(opts, :cmd_count, 0)
    angles = Keyword.get(opts, :angles, List.duplicate(0.0, 7))
    pose = Keyword.get(opts, :pose, List.duplicate(0.0, 6))
    torques = Keyword.get(opts, :torques, List.duplicate(0.0, 7))
    ft_filtered = Keyword.get(opts, :ft_filtered, List.duplicate(0.0, 6))
    ft_raw = Keyword.get(opts, :ft_raw, List.duplicate(0.0, 6))

    state_mode = mode <<< 4 ||| (state &&& 0x0F)

    payload =
      <<state_mode::8, cmd_count::16>> <>
        fp32s(angles) <>
        fp32s(pose) <>
        fp32s(torques) <>
        fp32s(ft_filtered) <>
        fp32s(ft_raw)

    frame_length = byte_size(payload) + 4
    <<frame_length::32>> <> payload
  end

  # ── Shared state builder ─────────────────────────────────────────────────────

  defp make_state(cmd_socket, extra \\ %{}) do
    ets = :ets.new(:test_controller_ets, [:public, :set])
    # Pre-populate 6-joint rows (xarm6)
    for i <- 1..6, do: :ets.insert(ets, {i, nil, nil, nil})
    :ets.insert(ets, {:arm, 0, 0, nil})

    base = %{
      bb: %{robot: TestRobot, path: [:xarm]},
      host: "127.0.0.1",
      port: 502,
      model_config: %{joints: 6, max_speed_rads: :math.pi(), limits: []},
      controller_name: :xarm,
      loop_interval_ms: 10,
      heartbeat_interval_ms: 1_000,
      disarm_action: :stop,
      cmd_socket: cmd_socket,
      report_socket: nil,
      buffer: <<>>,
      ets: ets,
      txn_id: 0,
      last_error_code: 0,
      last_arm_status: nil
    }

    Map.merge(base, extra)
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "sends cmd_stop on a fresh TCP connection" do
      {:ok, listen} = :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])
      {:ok, port} = :inet.port(listen)

      task =
        Task.async(fn ->
          {:ok, server} = :gen_tcp.accept(listen, 2_000)
          {:ok, data} = recv_all(server)
          data
        end)

      opts = [host: "127.0.0.1", port: port, disarm_action: :stop]
      assert :ok = Controller.disarm(opts)

      received = Task.await(task, 3_000)
      expected_frame = Protocol.cmd_stop(0)
      assert received == expected_frame

      :gen_tcp.close(listen)
    end

    test "sends cmd_enable(false) when disarm_action is :hold" do
      {:ok, listen} = :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])
      {:ok, port} = :inet.port(listen)

      task =
        Task.async(fn ->
          {:ok, server} = :gen_tcp.accept(listen, 2_000)
          {:ok, data} = recv_all(server)
          data
        end)

      opts = [host: "127.0.0.1", port: port, disarm_action: :hold]
      assert :ok = Controller.disarm(opts)

      received = Task.await(task, 3_000)
      expected_frame = Protocol.cmd_enable(0, false)
      assert received == expected_frame

      :gen_tcp.close(listen)
    end

    test "returns :ok even when TCP connection fails" do
      # Port 1 is privileged and will be refused on loopback
      opts = [host: "127.0.0.1", port: 1, disarm_action: :stop]
      assert :ok = Controller.disarm(opts)
    end

    test "returns :ok even when host is unreachable" do
      opts = [host: "192.0.2.1", port: 502, disarm_action: :stop]
      assert :ok = Controller.disarm(opts)
    end
  end

  # ── init/1 ──────────────────────────────────────────────────────────────────

  describe "init/1" do
    setup do
      BB
      |> stub(:subscribe, fn _robot, _path -> :ok end)

      BB.Safety
      |> stub(:register, fn _module, _opts -> :ok end)

      :ok
    end

    test "connects to both TCP sockets and returns :ok state" do
      # Start stub servers for both the command and report ports
      {:ok, cmd_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, cmd_port} = :inet.port(cmd_listen)

      {:ok, report_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, report_port} = :inet.port(report_listen)

      # Accept connections asynchronously
      cmd_task = Task.async(fn -> :gen_tcp.accept(cmd_listen, 2_000) end)
      report_task = Task.async(fn -> :gen_tcp.accept(report_listen, 2_000) end)

      opts = [
        bb: %{robot: TestRobot, path: [:xarm]},
        host: "127.0.0.1",
        port: cmd_port,
        report_port: report_port,
        model: :xarm6
      ]

      assert {:ok, state} = Controller.init(opts)

      assert {:ok, _} = Task.await(cmd_task, 2_000)
      assert {:ok, _} = Task.await(report_task, 2_000)

      assert state.controller_name == :xarm
      assert state.cmd_socket != nil
      assert state.report_socket != nil
      assert state.buffer == <<>>
      assert state.txn_id == 0
      assert state.last_error_code == 0

      :gen_tcp.close(state.cmd_socket)
      :gen_tcp.close(state.report_socket)
      :gen_tcp.close(cmd_listen)
      :gen_tcp.close(report_listen)
    end

    test "populates ETS with joint rows for the model" do
      {:ok, cmd_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, cmd_port} = :inet.port(cmd_listen)

      {:ok, report_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, report_port} = :inet.port(report_listen)

      Task.start(fn -> :gen_tcp.accept(cmd_listen, 2_000) end)
      Task.start(fn -> :gen_tcp.accept(report_listen, 2_000) end)

      opts = [
        bb: %{robot: TestRobot, path: [:xarm]},
        host: "127.0.0.1",
        port: cmd_port,
        report_port: report_port,
        model: :xarm6
      ]

      assert {:ok, state} = Controller.init(opts)

      # xArm6 has 6 joints
      for i <- 1..6 do
        assert [{^i, nil, nil, nil}] = :ets.lookup(state.ets, i)
      end

      assert [{:arm, 0, 0, nil}] = :ets.lookup(state.ets, :arm)

      :gen_tcp.close(state.cmd_socket)
      :gen_tcp.close(state.report_socket)
      :gen_tcp.close(cmd_listen)
      :gen_tcp.close(report_listen)
    end

    test "registers with safety system" do
      {:ok, cmd_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, cmd_port} = :inet.port(cmd_listen)

      {:ok, report_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, report_port} = :inet.port(report_listen)

      Task.start(fn -> :gen_tcp.accept(cmd_listen, 2_000) end)
      Task.start(fn -> :gen_tcp.accept(report_listen, 2_000) end)

      BB.Safety
      |> expect(:register, fn Controller, opts ->
        assert Keyword.get(opts, :robot) == TestRobot
        assert Keyword.get(opts, :path) == [:xarm]
        :ok
      end)

      opts = [
        bb: %{robot: TestRobot, path: [:xarm]},
        host: "127.0.0.1",
        port: cmd_port,
        report_port: report_port
      ]

      assert {:ok, state} = Controller.init(opts)

      :gen_tcp.close(state.cmd_socket)
      :gen_tcp.close(state.report_socket)
      :gen_tcp.close(cmd_listen)
      :gen_tcp.close(report_listen)
    end

    test "subscribes to state machine transitions" do
      {:ok, cmd_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, cmd_port} = :inet.port(cmd_listen)

      {:ok, report_listen} =
        :gen_tcp.listen(0, [:binary, active: false, packet: :raw, reuseaddr: true])

      {:ok, report_port} = :inet.port(report_listen)

      Task.start(fn -> :gen_tcp.accept(cmd_listen, 2_000) end)
      Task.start(fn -> :gen_tcp.accept(report_listen, 2_000) end)

      BB
      |> expect(:subscribe, fn TestRobot, [:state_machine] -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:xarm]},
        host: "127.0.0.1",
        port: cmd_port,
        report_port: report_port
      ]

      assert {:ok, state} = Controller.init(opts)

      :gen_tcp.close(state.cmd_socket)
      :gen_tcp.close(state.report_socket)
      :gen_tcp.close(cmd_listen)
      :gen_tcp.close(report_listen)
    end

    test "stops with reason when command TCP connection fails" do
      # Use a port that nobody is listening on
      opts = [
        bb: %{robot: TestRobot, path: [:xarm]},
        host: "127.0.0.1",
        port: 1,
        report_port: 30_003
      ]

      assert {:stop, _reason} = Controller.init(opts)
    end
  end

  # ── handle_info({:tcp, ...}) — report ingestion ──────────────────────────────

  describe "handle_info({:tcp, ...})" do
    setup do
      {cmd_client, _cmd_server} = tcp_pair()

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      BB.Safety
      |> stub(:armed?, fn _robot -> false end)

      state = make_state(cmd_client)
      on_exit(fn -> :gen_tcp.close(cmd_client) end)
      %{state: state}
    end

    test "updates ETS joint rows from report angles", %{state: state} do
      angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
      frame = build_87_byte_frame(angles: angles)

      assert {:noreply, new_state} = Controller.handle_info({:tcp, nil, frame}, state)
      assert new_state.buffer == <<>>

      for {angle, i} <- Enum.with_index(Enum.take(angles, 6), 1) do
        [{^i, read_pos, _torq, _set}] = :ets.lookup(new_state.ets, i)
        assert_in_delta read_pos, angle, 1.0e-5
      end
    end

    test "updates ETS joint rows from report torques", %{state: state} do
      torques = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7]
      frame = build_87_byte_frame(torques: torques)

      assert {:noreply, new_state} = Controller.handle_info({:tcp, nil, frame}, state)

      for {torque, i} <- Enum.with_index(Enum.take(torques, 6), 1) do
        [{^i, _pos, read_torq, _set}] = :ets.lookup(new_state.ets, i)
        assert_in_delta read_torq, torque, 1.0e-5
      end
    end

    test "updates arm-level ETS row with state and mode", %{state: state} do
      frame = build_87_byte_frame(state: 4, mode: 1)

      assert {:noreply, new_state} = Controller.handle_info({:tcp, nil, frame}, state)

      assert [{:arm, 4, 1, _pose}] = :ets.lookup(new_state.ets, :arm)
    end

    test "updates arm-level ETS row with TCP pose", %{state: state} do
      pose = [100.0, 200.0, 300.0, 0.1, 0.2, 0.3]
      frame = build_87_byte_frame(pose: pose)

      assert {:noreply, new_state} = Controller.handle_info({:tcp, nil, frame}, state)

      [{:arm, _st, _mode, {x, y, z, roll, pitch, yaw}}] = :ets.lookup(new_state.ets, :arm)
      assert_in_delta x, 100.0, 1.0e-3
      assert_in_delta y, 200.0, 1.0e-3
      assert_in_delta z, 300.0, 1.0e-3
      assert_in_delta roll, 0.1, 1.0e-5
      assert_in_delta pitch, 0.2, 1.0e-5
      assert_in_delta yaw, 0.3, 1.0e-5
    end

    test "preserves set_position in ETS when report arrives", %{state: state} do
      :ets.insert(state.ets, {1, nil, nil, 1.57})

      frame = build_87_byte_frame(angles: [0.1 | List.duplicate(0.0, 6)])

      assert {:noreply, new_state} = Controller.handle_info({:tcp, nil, frame}, state)

      [{1, _cur, _torq, set_pos}] = :ets.lookup(new_state.ets, 1)
      assert set_pos == 1.57
    end

    test "publishes JointState message", %{state: state} do
      angles = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
      frame = build_87_byte_frame(angles: angles)

      BB
      |> expect(:publish, fn TestRobot, [:sensor, :xarm], %Message{payload: %JointState{} = js} ->
        # fp32 encoding introduces small rounding errors — use delta comparison
        Enum.zip(js.positions, Enum.take(angles, 6))
        |> Enum.each(fn {got, want} -> assert_in_delta got, want, 1.0e-5 end)

        assert js.names == [:j1, :j2, :j3, :j4, :j5, :j6]
        :ok
      end)
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      assert {:noreply, _state} = Controller.handle_info({:tcp, nil, frame}, state)
    end

    test "publishes CartesianPose message", %{state: state} do
      pose = [100.0, 200.0, 300.0, 0.1, 0.2, 0.3]
      frame = build_87_byte_frame(pose: pose)

      BB
      |> stub(:publish, fn
        TestRobot, [:sensor, :xarm, :tcp_pose], %Message{payload: %CartesianPose{} = cp} ->
          assert_in_delta cp.x, 100.0, 1.0e-3
          assert_in_delta cp.y, 200.0, 1.0e-3
          assert_in_delta cp.z, 300.0, 1.0e-3
          :ok

        _robot, _path, _msg ->
          :ok
      end)

      assert {:noreply, _state} = Controller.handle_info({:tcp, nil, frame}, state)
    end

    test "publishes Wrench message when ft_filtered data is present", %{state: state} do
      ft_values = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
      frame = build_135_byte_frame(ft_filtered: ft_values)

      BB
      |> stub(:publish, fn
        TestRobot, [:sensor, :xarm, :wrench], %Message{payload: %Wrench{} = w} ->
          assert_in_delta w.fx, 1.0, 1.0e-4
          assert_in_delta w.fy, 2.0, 1.0e-4
          assert_in_delta w.fz, 3.0, 1.0e-4
          assert_in_delta w.tx, 0.1, 1.0e-4
          assert_in_delta w.ty, 0.2, 1.0e-4
          assert_in_delta w.tz, 0.3, 1.0e-4
          :ok

        _robot, _path, _msg ->
          :ok
      end)

      assert {:noreply, _state} = Controller.handle_info({:tcp, nil, frame}, state)
    end

    test "does not publish Wrench when ft_filtered is absent (87-byte frame)", %{state: state} do
      frame = build_87_byte_frame()

      BB
      |> stub(:publish, fn
        TestRobot, [:sensor, :xarm, :wrench], _msg ->
          flunk("Wrench should not be published for 87-byte frames")

        _robot, _path, _msg ->
          :ok
      end)

      assert {:noreply, _state} = Controller.handle_info({:tcp, nil, frame}, state)
    end

    test "accumulates partial frames in buffer", %{state: state} do
      frame = build_87_byte_frame()
      <<first_half::binary-size(40), second_half::binary>> = frame

      assert {:noreply, partial_state} =
               Controller.handle_info({:tcp, nil, first_half}, state)

      assert byte_size(partial_state.buffer) == 40

      assert {:noreply, full_state} =
               Controller.handle_info({:tcp, nil, second_half}, partial_state)

      assert full_state.buffer == <<>>
    end

    test "drains multiple complete frames from a single TCP chunk", %{state: state} do
      frame = build_87_byte_frame()
      two_frames = frame <> frame

      # Two frames → 2× JointState + 2× CartesianPose + 1× ArmStatus (only on first frame,
      # second frame has same state/mode so ArmStatus is not re-published) = 5 publish calls total
      BB
      |> expect(:publish, 5, fn TestRobot, _path, %Message{} -> :ok end)

      assert {:noreply, new_state} = Controller.handle_info({:tcp, nil, two_frames}, state)
      assert new_state.buffer == <<>>
    end
  end

  # ── handle_info(:loop) — control loop ────────────────────────────────────────

  describe "handle_info(:loop)" do
    setup do
      {cmd_client, cmd_server} = tcp_pair()

      BB.Safety
      |> stub(:armed?, fn _robot -> true end)

      state = make_state(cmd_client)

      on_exit(fn ->
        :gen_tcp.close(cmd_client)
        :gen_tcp.close(cmd_server)
      end)

      %{state: state, cmd_server: cmd_server}
    end

    test "sends cmd_move_joints when set_positions are pending and robot is armed",
         %{state: state, cmd_server: cmd_server} do
      # Write set_position for all 6 joints
      for i <- 1..6, do: :ets.insert(state.ets, {i, 0.0, 0.0, 0.5})

      assert {:noreply, new_state} = Controller.handle_info(:loop, state)

      # A joint move frame should have been sent
      assert {:ok, data} = recv_all(cmd_server, 200)
      assert byte_size(data) > 0

      # Frame should be 47 bytes: 6-byte header + 1-byte reg + 10 fp32 params (40 bytes)
      assert byte_size(data) == 47

      # txn_id should have incremented
      assert new_state.txn_id == 1
    end

    test "does not send when no set_positions are pending", %{
      state: state,
      cmd_server: cmd_server
    } do
      # All set_position remain nil
      assert {:noreply, _new_state} = Controller.handle_info(:loop, state)

      assert {:error, :timeout} = recv_all(cmd_server, 50)
    end

    test "does not send when robot is not armed", %{state: state, cmd_server: cmd_server} do
      BB.Safety
      |> expect(:armed?, fn TestRobot -> false end)

      for i <- 1..6, do: :ets.insert(state.ets, {i, 0.0, 0.0, 1.0})

      assert {:noreply, _new_state} = Controller.handle_info(:loop, state)

      assert {:error, :timeout} = recv_all(cmd_server, 50)
    end

    test "uses current_position for joints where set_position is nil",
         %{state: state, cmd_server: cmd_server} do
      # Only joint 1 has a pending set_position; others have current from report
      :ets.insert(state.ets, {1, 0.1, 0.0, 0.5})

      for i <- 2..6, do: :ets.insert(state.ets, {i, 0.2, 0.0, nil})

      assert {:noreply, _new_state} = Controller.handle_info(:loop, state)

      # Should still send a joint move since joint 1 has set_position
      assert {:ok, data} = recv_all(cmd_server, 200)
      assert byte_size(data) == 47
    end
  end

  # ── handle_info(:heartbeat) ──────────────────────────────────────────────────

  describe "handle_info(:heartbeat)" do
    setup do
      {cmd_client, cmd_server} = tcp_pair()
      state = make_state(cmd_client)

      on_exit(fn ->
        :gen_tcp.close(cmd_client)
        :gen_tcp.close(cmd_server)
      end)

      %{state: state, cmd_server: cmd_server}
    end

    test "sends the 8-byte heartbeat frame over the command socket",
         %{state: state, cmd_server: cmd_server} do
      assert {:noreply, _new_state} = Controller.handle_info(:heartbeat, state)

      assert {:ok, data} = recv_all(cmd_server, 200)
      assert data == Protocol.heartbeat()
    end
  end

  # ── handle_info(state machine) ───────────────────────────────────────────────

  describe "handle_info(state machine transition)" do
    setup do
      {cmd_client, cmd_server} = tcp_pair()
      state = make_state(cmd_client)

      on_exit(fn ->
        :gen_tcp.close(cmd_client)
        :gen_tcp.close(cmd_server)
      end)

      %{state: state, cmd_server: cmd_server}
    end

    test "sends cmd_enable(true) + cmd_set_state(3) on transition to :armed",
         %{state: state, cmd_server: cmd_server} do
      {:ok, msg} = Transition.new(:disarmed, from: :disarmed, to: :armed)
      bb_msg = {:bb, [:state_machine], msg}

      assert {:noreply, new_state} = Controller.handle_info(bb_msg, state)

      enable_frame = Protocol.cmd_enable(0, true)
      play_frame = Protocol.cmd_set_state(1, 3)
      expected_bytes = byte_size(enable_frame) + byte_size(play_frame)

      assert {:ok, data} = recv_at_least(cmd_server, expected_bytes)
      assert data == enable_frame <> play_frame

      # txn_id advances by 2 (one per frame sent)
      assert new_state.txn_id == 2
    end

    test "sends cmd_stop on transition away from :armed when disarm_action is :stop",
         %{state: state, cmd_server: cmd_server} do
      {:ok, msg} = Transition.new(:armed, from: :armed, to: :disarmed)
      bb_msg = {:bb, [:state_machine], msg}

      assert {:noreply, new_state} = Controller.handle_info(bb_msg, state)

      assert {:ok, data} = recv_all(cmd_server, 200)
      assert data == Protocol.cmd_stop(0)
      assert new_state.txn_id == 1
    end

    test "sends cmd_enable(false) on transition away from :armed when disarm_action is :hold",
         %{state: state, cmd_server: cmd_server} do
      state = %{state | disarm_action: :hold}

      {:ok, msg} = Transition.new(:armed, from: :armed, to: :disarmed)
      bb_msg = {:bb, [:state_machine], msg}

      assert {:noreply, _new_state} = Controller.handle_info(bb_msg, state)

      assert {:ok, data} = recv_all(cmd_server, 200)
      assert data == Protocol.cmd_enable(0, false)
    end
  end

  # ── handle_call({:send_command, frame}) ──────────────────────────────────────

  describe "handle_call({:send_command, frame})" do
    setup do
      {cmd_client, cmd_server} = tcp_pair()
      state = make_state(cmd_client)

      on_exit(fn ->
        :gen_tcp.close(cmd_client)
        :gen_tcp.close(cmd_server)
      end)

      %{state: state, cmd_server: cmd_server}
    end

    test "sends frame over command socket and returns :ok",
         %{state: state, cmd_server: cmd_server} do
      frame = Protocol.cmd_gripper_position(1, 420)

      assert {:reply, :ok, new_state} =
               Controller.handle_call({:send_command, frame}, {self(), make_ref()}, state)

      assert {:ok, data} = recv_all(cmd_server, 200)
      assert data == frame
      assert new_state.txn_id == 1
    end

    test "increments txn_id on each send", %{state: state} do
      frame = Protocol.cmd_get_ft_data(0)

      {:reply, :ok, state1} =
        Controller.handle_call({:send_command, frame}, {self(), make_ref()}, state)

      {:reply, :ok, state2} =
        Controller.handle_call({:send_command, frame}, {self(), make_ref()}, state1)

      assert state2.txn_id == 2
    end
  end

  # ── handle_call(:get_ets) ────────────────────────────────────────────────────

  describe "handle_call(:get_ets)" do
    test "returns the ETS table reference" do
      {cmd_client, _cmd_server} = tcp_pair()
      state = make_state(cmd_client)
      on_exit(fn -> :gen_tcp.close(cmd_client) end)

      assert {:reply, ets, ^state} =
               Controller.handle_call(:get_ets, {self(), make_ref()}, state)

      assert ets == state.ets
    end
  end

  # ── txn_id rollover ───────────────────────────────────────────────────────────

  describe "txn_id rollover" do
    setup do
      {cmd_client, cmd_server} = tcp_pair()

      BB.Safety
      |> stub(:armed?, fn _robot -> true end)

      state = make_state(cmd_client, %{txn_id: 65_535})

      on_exit(fn ->
        :gen_tcp.close(cmd_client)
        :gen_tcp.close(cmd_server)
      end)

      %{state: state, cmd_server: cmd_server}
    end

    test "wraps txn_id from 65535 to 0", %{state: state, cmd_server: cmd_server} do
      frame = Protocol.cmd_get_ft_data(0)

      assert {:reply, :ok, new_state} =
               Controller.handle_call({:send_command, frame}, {self(), make_ref()}, state)

      assert new_state.txn_id == 0
      assert {:ok, _data} = recv_all(cmd_server, 200)
    end
  end
end
