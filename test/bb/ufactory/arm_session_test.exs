# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ArmSessionTest do
  use ExUnit.Case, async: false

  alias BB.Error.Protocol.Ufactory.CommandFailed
  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.Ufactory.Message.Command.CartesianMove
  alias BB.Ufactory.Protocol

  test "re-arming does not replay cached, legacy, or controller-queued motion" do
    {listener, report, cmd, robot} = start_loopback_robot()
    arm(robot, cmd)
    {:ok, epoch} = BB.Safety.epoch(robot)
    publish_report(robot, report)

    assert :ok = BB.Actuator.set_position(robot, :j1_motor, 0.75)
    assert {0x17, <<0.75::float-little-32, _::binary>>} = await_frame(cmd, [0x17])

    controller = BB.Process.whereis(robot, :xarm)
    assert :ok = :sys.suspend(controller)
    assert :timeout = await_frame(cmd, [], 20)

    assert :ok =
             BB.cast(robot, :cartesian, {:move_cartesian, {100.0, 0.0, 100.0, 0.0, 0.0, 0.0}})

    message =
      Message.new!(CartesianMove, :cartesian,
        x: 100.0,
        y: 0.0,
        z: 100.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0
      )

    # Let BB authorize the typed command before disarm. The pending controller
    # call outlives that authorization while the controller is suspended.
    task =
      Task.async(fn ->
        BB.call(robot, :cartesian, {:command, %{message | arm_epoch: epoch}})
      end)

    assert eventually(fn ->
             {:messages, messages} = Process.info(controller, :messages)
             Enum.any?(messages, &match?({:"$gen_call", _, {:send_motion, ^epoch, _}}, &1))
           end)

    assert :ok = disarm_with_ack(robot, listener)
    assert :ok = BB.Safety.arm(robot)
    assert {:ok, next_epoch} = BB.Safety.epoch(robot)
    refute next_epoch == epoch

    assert :ok = :sys.resume(controller)
    assert {:error, %CommandFailed{reason: :inactive_arm_epoch}} = Task.await(task)

    assert {0x0C, <<4>>} = await_frame(cmd, [0x0C, 0x15, 0x17])
    assert {0x0C, <<4>>} = await_frame(cmd, [0x0C, 0x15, 0x17])
    assert {0x0C, <<0>>} = await_frame(cmd, [0x0C, 0x15, 0x17])

    publish_report(robot, report)
    assert :timeout = await_frame(cmd, [0x15, 0x17], 50)

    assert :ok = BB.Actuator.set_position(robot, :j1_motor, 0.5)
    assert {0x17, <<0.5::float-little-32, _::binary>>} = await_frame(cmd, [0x17])

    :gen_tcp.close(listener)

    assert {:error, {:disarm_failed, [{[:xarm], {:returned_error, %CommandFailed{}}}]}} =
             BB.Safety.disarm(robot)

    assert BB.Safety.state(robot) == :error
  end

  test "Cartesian dispatch retires earlier joint targets in the same arm session" do
    {listener, report, cmd, robot} = start_loopback_robot(__MODULE__.Cartesian)
    arm(robot, cmd)
    publish_report(robot, report)
    assert :ok = BB.Actuator.set_position(robot, :j1_motor, 0.75)
    assert {0x17, <<0.75::float-little-32, _::binary>>} = await_frame(cmd, [0x17])

    message =
      Message.new!(CartesianMove, :cartesian, x: 100, y: 0, z: 100, roll: 0, pitch: 0, yaw: 0)

    {:ok, epoch} = BB.Safety.epoch(robot)

    assert {:ok, :accepted} =
             BB.call(robot, :cartesian, {:command, %{message | arm_epoch: epoch}})

    assert {0x15, _} = await_frame(cmd, [0x15])
    assert :timeout = await_frame(cmd, [0x17], 50)
    assert :ok = BB.Actuator.set_position(robot, :j1_motor, 0.25)
    assert {0x17, <<0.25::float-little-32, _::binary>>} = await_frame(cmd, [0x17])
    assert :ok = disarm_with_ack(robot, listener)
  end

  test "surviving accessories re-enable after controller restart and confirm their own disarm" do
    {listener, _report, _cmd, robot} = start_loopback_robot(__MODULE__.Accessories, true)
    gripper = BB.Process.whereis(robot, :gripper)
    track = BB.Process.whereis(robot, :track)
    :ok = GenServer.stop(BB.Process.whereis(robot, :xarm))
    {:ok, cmd} = :gen_tcp.accept(listener, 1000)
    arm(robot, cmd)
    assert BB.Process.whereis(robot, :gripper) == gripper
    assert BB.Process.whereis(robot, :track) == track

    setup_frames =
      for _ <- 1..3 do
        {0x7C, payload} = await_frame(cmd, [0x7C])
        payload
      end

    expected = [
      Protocol.cmd_gripper_enable(0, true),
      Protocol.cmd_gripper_speed(0, 1500),
      Protocol.cmd_linear_track_enable(0, true)
    ]

    assert Enum.sort(setup_frames) == Enum.sort(Enum.map(expected, &wire_payload/1))

    replies = Task.async(fn -> acknowledge_disables(cmd, :ok) end)
    assert :ok = disarm_with_ack(robot, listener)
    Task.await(replies)

    arm(robot, cmd)
    for _ <- 1..3, do: assert({0x7C, _} = await_frame(cmd, [0x7C]))
    replies = Task.async(fn -> acknowledge_disables(cmd, :reject_track) end)

    assert {:error, {:disarm_failed, [{path, {:returned_error, %CommandFailed{}}}]}} =
             disarm_with_ack(robot, listener)

    assert List.last(path) == :track
    assert BB.Safety.state(robot) == :error
    Task.await(replies)
  end

  test "report processing and ticks proceed while a heartbeat poll is unanswered" do
    {listener, report, cmd, robot} = start_loopback_robot(__MODULE__.Polling)
    arm(robot, cmd)
    controller = BB.Process.whereis(robot, :xarm)
    send(controller, :heartbeat)
    assert {0x0F, _} = await_frame(cmd, [0x0F])
    publish_report(robot, report)
    assert :ok = BB.Actuator.set_position(robot, :j1_motor, 0.25)
    assert {0x17, _} = await_frame(cmd, [0x17], 200)
    assert map_size(:sys.get_state(controller).user_state.pending) == 1
    assert :ok = disarm_with_ack(robot, listener)
  end

  test "feedback retains its receive time across a suspended controller" do
    {_listener, report, _cmd, robot} = start_loopback_robot(__MODULE__.Feedback)
    controller = BB.Process.whereis(robot, :xarm)
    :ok = :sys.suspend(controller)
    :ok = :gen_tcp.send(report, <<87::32, 0::664>>)

    assert eventually(fn ->
             {:messages, messages} = Process.info(controller, :messages)
             Enum.any?(messages, &match?({:report_bytes, _, _, _}, &1))
           end)

    before_resume = System.monotonic_time(:nanosecond)
    :ok = :sys.resume(controller)
    assert_receive {:bb, [:sensor, :xarm], %Message{monotonic_time: received}}, 1000
    assert received <= before_resume

    assert :sys.get_state(controller).user_state.last_report_at ==
             System.convert_time_unit(received, :nanosecond, :millisecond)
  end

  defp wire_payload(<<_::56, payload::binary>>), do: payload

  defp acknowledge_disables(socket, mode) do
    expected = [Protocol.cmd_gripper_enable(0, false), Protocol.cmd_linear_track_enable(0, false)]

    writes =
      for _ <- 1..2 do
        {id, 0x7C, payload} = read_register(socket, 0x7C)
        <<host, _::binary>> = payload

        echo =
          if mode == :reject_track and host == 11,
            do: <<11, 1, 0x90, 2>>,
            else: binary_part(payload, 0, 7)

        :ok = :gen_tcp.send(socket, Protocol.build_frame(id, 0x7C, <<0, echo::binary>>))
        payload
      end

    assert Enum.sort(writes) == Enum.sort(Enum.map(expected, &wire_payload/1))
  end

  defp read_register(socket, register) do
    {:ok, <<id::16, 2::16, length::16>>} = :gen_tcp.recv(socket, 6, 1000)
    {:ok, <<actual, payload::binary>>} = :gen_tcp.recv(socket, length, 1000)
    if actual == register, do: {id, actual, payload}, else: read_register(socket, register)
  end

  defp start_loopback_robot(robot \\ __MODULE__.Robot, accessories \\ false) do
    {listener, port} = listen()
    {report_listener, report_port} = listen()

    Code.eval_quoted(
      quote do
        defmodule unquote(robot) do
          use BB.Ufactory.Robots.XArm6,
            host: "127.0.0.1",
            gripper: unquote(accessories),
            linear_track: unquote(accessories),
            controller: [
              port: unquote(port),
              report_port: unquote(report_port),
              heartbeat_interval_ms: 60_000,
              report_stale_ms: 5_000
            ]
        end
      end
    )

    start_supervised!({robot, []})
    {:ok, cmd} = :gen_tcp.accept(listener, 1_000)
    {:ok, report} = :gen_tcp.accept(report_listener, 1_000)
    {:ok, _} = BB.subscribe(robot, [:sensor, :xarm])
    {listener, report, cmd, robot}
  end

  defp listen do
    {:ok, socket} = :gen_tcp.listen(0, [:binary, active: false, ip: {127, 0, 0, 1}])
    {:ok, port} = :inet.port(socket)
    {socket, port}
  end

  defp arm(robot, cmd) do
    assert :ok = BB.Safety.arm(robot)
    assert {0x0C, <<0>>} = await_frame(cmd, [{0x0C, <<0>>}])
  end

  defp publish_report(robot, socket) do
    assert :ok = :gen_tcp.send(socket, <<87::32, 0::664>>)
    assert_receive {:bb, [:sensor, :xarm], %Message{robot: ^robot, payload: %JointState{}}}, 1_000
  end

  defp disarm_with_ack(robot, listener) do
    task =
      Task.async(fn ->
        {:ok, socket} = :gen_tcp.accept(listener, 1_000)
        assert {0x0C, <<4>>} = await_frame(socket, [0x0C])
        :ok = :gen_tcp.send(socket, <<0::16, 2::16, 2::16, 0x0C, 0>>)
        :gen_tcp.close(socket)
      end)

    result = BB.Safety.disarm(robot)
    :ok = Task.await(task)
    result
  end

  defp await_frame(socket, registers, timeout \\ 1_000) do
    read_matching_frame(socket, registers, System.monotonic_time(:millisecond) + timeout)
  end

  defp read_matching_frame(socket, registers, deadline) do
    remaining = max(deadline - System.monotonic_time(:millisecond), 0)

    with {:ok, <<_transaction::16, 2::16, length::16>>} <- :gen_tcp.recv(socket, 6, remaining),
         {:ok, <<register, payload::binary>>} <- :gen_tcp.recv(socket, length, remaining) do
      if register in registers or {register, payload} in registers,
        do: {register, payload},
        else: read_matching_frame(socket, registers, deadline)
    else
      {:error, :timeout} -> :timeout
    end
  end

  defp eventually(predicate, attempts \\ 100)
  defp eventually(_predicate, 0), do: false

  defp eventually(predicate, attempts) do
    if predicate.() do
      true
    else
      Process.sleep(5)
      eventually(predicate, attempts - 1)
    end
  end
end
