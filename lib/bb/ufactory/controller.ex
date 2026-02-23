# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Controller do
  @moduledoc """
  Controller for UFactory xArm robotic arms.

  Manages two TCP connections to the arm:

  - **Port 502** — command socket: sends register frames, receives responses,
    sends the 1-second heartbeat to keep the connection alive.
  - **Port 30003** — real-time report socket: receives arm-pushed state frames
    at ~100 Hz. The socket is opened in `active: true` mode so incoming frames
    arrive as `{:tcp, socket, data}` messages without polling.

  ## ETS Table

  An ETS table (`:public`, `:set`) is created on `init/1` and shared with
  actuators. Per-joint rows are keyed by joint index (1-based integer):

      {joint_index, current_position :: float | nil, current_torque :: float | nil,
       set_position :: float | nil}

  The arm-level row is keyed by `:arm`:

      {:arm, state :: 0..5, mode :: 0..3, tcp_pose :: {x, y, z, roll, pitch, yaw} | nil}

  Actuators write `set_position` into the ETS table. The 100 Hz control loop
  reads all pending positions and batches them into a single `cmd_move_joints`
  frame per tick.

  ## Safety

  On arm transition → `:armed`, the controller sends `cmd_enable(true)` then
  `cmd_set_state(3)` (play/motion-ready). On any transition away from `:armed`
  it sends `cmd_enable(false)` (hold) or `cmd_stop` depending on
  `disarm_action`.

  The `disarm/1` safety callback opens a **fresh** TCP connection so it can
  send `cmd_stop` even if the GenServer process has crashed. It never raises.
  """

  use BB.Controller,
    options_schema: [
      host: [
        type: :string,
        required: true,
        doc: "IP address or hostname of the xArm controller"
      ],
      port: [
        type: :pos_integer,
        default: 502,
        doc: "Command socket port (default: 502)"
      ],
      report_port: [
        type: :pos_integer,
        default: 30_003,
        doc: "Real-time report socket port (default: 30003)"
      ],
      model: [
        type: {:in, [:xarm5, :xarm6, :xarm7, :lite6, :xarm850]},
        default: :xarm6,
        doc: "xArm model variant"
      ],
      loop_hz: [
        type: :pos_integer,
        default: 100,
        doc: "Control loop frequency in Hz (default: 100)"
      ],
      heartbeat_interval_ms: [
        type: :pos_integer,
        default: 1000,
        doc: "Heartbeat interval in milliseconds (default: 1000)"
      ],
      disarm_action: [
        type: {:in, [:stop, :hold]},
        default: :stop,
        doc: "Action taken when robot is disarmed: :stop clears motion, :hold holds position"
      ]
    ]

  require Logger

  alias BB.Message
  alias BB.Message.Sensor.JointState
  alias BB.StateMachine.Transition
  alias BB.Ufactory.Message.CartesianPose
  alias BB.Ufactory.Model
  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Report

  # Statically-defined joint name atoms so atom creation is never dynamic.
  # All xArm models have at most 7 joints.
  @all_joint_names [:j1, :j2, :j3, :j4, :j5, :j6, :j7]

  # ── Safety callback ─────────────────────────────────────────────────────────

  @doc """
  Opens a fresh TCP connection and sends a stop command.

  This runs outside the GenServer process so it is safe even if the controller
  has crashed. It never raises — any TCP failure is silently swallowed and `:ok`
  is returned regardless.
  """
  @impl BB.Controller
  def disarm(opts) do
    host = Keyword.fetch!(opts, :host)
    port = Keyword.get(opts, :port, 502)
    disarm_action = Keyword.get(opts, :disarm_action, :stop)

    try do
      case :gen_tcp.connect(String.to_charlist(host), port, [:binary, active: false], 2_000) do
        {:ok, sock} ->
          frame =
            case disarm_action do
              :stop -> Protocol.cmd_stop(0)
              :hold -> Protocol.cmd_enable(0, false)
            end

          :gen_tcp.send(sock, frame)
          :gen_tcp.close(sock)

        {:error, _reason} ->
          :ok
      end
    catch
      _, _ -> :ok
    end

    :ok
  end

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Controller
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    host = Keyword.fetch!(opts, :host)
    port = Keyword.get(opts, :port, 502)
    report_port = Keyword.get(opts, :report_port, 30_003)
    model = Keyword.get(opts, :model, :xarm6)
    loop_hz = Keyword.get(opts, :loop_hz, 100)
    heartbeat_interval_ms = Keyword.get(opts, :heartbeat_interval_ms, 1_000)
    disarm_action = Keyword.get(opts, :disarm_action, :stop)

    model_config = Model.get(model)
    loop_interval_ms = max(1, div(1_000, loop_hz))
    controller_name = List.last(bb.path)

    charlist_host = String.to_charlist(host)
    tcp_opts = [:binary, active: false, packet: :raw]

    with {:ok, cmd_socket} <- :gen_tcp.connect(charlist_host, port, tcp_opts),
         {:ok, report_socket} <-
           :gen_tcp.connect(charlist_host, report_port, [:binary, active: true, packet: :raw]) do
      ets = create_ets(bb.robot, controller_name, model_config.joints)

      BB.Safety.register(__MODULE__,
        robot: bb.robot,
        path: bb.path,
        opts: [host: host, port: port, disarm_action: disarm_action]
      )

      BB.subscribe(bb.robot, [:state_machine])

      Process.send_after(self(), :loop, loop_interval_ms)
      Process.send_after(self(), :heartbeat, heartbeat_interval_ms)

      state = %{
        bb: bb,
        host: host,
        port: port,
        model_config: model_config,
        controller_name: controller_name,
        loop_interval_ms: loop_interval_ms,
        heartbeat_interval_ms: heartbeat_interval_ms,
        disarm_action: disarm_action,
        cmd_socket: cmd_socket,
        report_socket: report_socket,
        buffer: <<>>,
        ets: ets,
        txn_id: 0,
        last_error_code: 0
      }

      {:ok, state}
    else
      {:error, reason} ->
        Logger.error(
          "[BB.Ufactory.Controller] TCP connect failed for #{inspect(bb.path)}: #{inspect(reason)}"
        )

        {:stop, reason}
    end
  end

  # ── Control loop ─────────────────────────────────────────────────────────────

  @impl BB.Controller
  def handle_info(:loop, state) do
    state = maybe_send_joint_move(state)
    Process.send_after(self(), :loop, state.loop_interval_ms)
    {:noreply, state}
  end

  # ── Report socket ─────────────────────────────────────────────────────────────

  def handle_info({:tcp, _socket, data}, state) do
    buffer = state.buffer <> data
    state = drain_buffer(buffer, state)
    {:noreply, state}
  end

  def handle_info({:tcp_closed, _socket}, state) do
    Logger.warning("[BB.Ufactory.Controller] Report socket closed for #{state.controller_name}")
    {:noreply, %{state | report_socket: nil}}
  end

  def handle_info({:tcp_error, _socket, reason}, state) do
    Logger.error(
      "[BB.Ufactory.Controller] Report socket error for #{state.controller_name}: #{inspect(reason)}"
    )

    {:noreply, %{state | report_socket: nil}}
  end

  # ── Heartbeat ─────────────────────────────────────────────────────────────────

  def handle_info(:heartbeat, state) do
    case :gen_tcp.send(state.cmd_socket, Protocol.heartbeat()) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Controller] Heartbeat send failed for #{state.controller_name}: #{inspect(reason)}"
        )
    end

    Process.send_after(self(), :heartbeat, state.heartbeat_interval_ms)
    {:noreply, state}
  end

  # ── State machine subscription ─────────────────────────────────────────────────

  def handle_info(
        {:bb, [:state_machine], %Message{payload: %Transition{to: :armed}}},
        state
      ) do
    # send_command increments txn_id after each send, so the second frame
    # uses state.txn_id (already updated to 1 after the first send).
    state = send_command(Protocol.cmd_enable(state.txn_id, true), state)
    state = send_command(Protocol.cmd_set_state(state.txn_id, 3), state)
    {:noreply, state}
  end

  def handle_info({:bb, [:state_machine], %Message{payload: %Transition{}}}, state) do
    frame =
      case state.disarm_action do
        :stop -> Protocol.cmd_stop(state.txn_id)
        :hold -> Protocol.cmd_enable(state.txn_id, false)
      end

    state = send_command(frame, state)
    {:noreply, state}
  end

  # ── handle_call: send_command for actuators/sensors ─────────────────────────

  @impl BB.Controller
  def handle_call({:send_command, frame}, _from, state) do
    result = :gen_tcp.send(state.cmd_socket, frame)
    {:reply, result, %{state | txn_id: next_txn(state.txn_id)}}
  end

  # ── handle_call: ETS table reference for actuators ──────────────────────────

  def handle_call(:get_ets, _from, state) do
    {:reply, state.ets, state}
  end

  # ── terminate ─────────────────────────────────────────────────────────────────

  @impl BB.Controller
  def terminate(_reason, state) do
    if state.cmd_socket, do: :gen_tcp.close(state.cmd_socket)
    if state.report_socket, do: :gen_tcp.close(state.report_socket)
    :ok
  end

  # ── Private helpers ───────────────────────────────────────────────────────────

  defp create_ets(robot, controller_name, joint_count) do
    # Build the ETS table name from known module atoms to avoid dynamic atom creation.
    table_name = Module.concat([robot, "Controller", controller_name])

    ets =
      case :ets.whereis(table_name) do
        :undefined -> :ets.new(table_name, [:public, :set, :named_table])
        existing -> existing
      end

    # Pre-populate per-joint rows
    for i <- 1..joint_count do
      :ets.insert(ets, {i, nil, nil, nil})
    end

    # Arm-level row
    :ets.insert(ets, {:arm, 0, 0, nil})

    ets
  end

  defp maybe_send_joint_move(state) do
    joint_count = state.model_config.joints
    rows = for i <- 1..joint_count, do: :ets.lookup(state.ets, i)
    rows = List.flatten(rows)

    pending? = Enum.any?(rows, fn {_i, _cur_pos, _cur_torq, set_pos} -> set_pos != nil end)

    if pending? and BB.Safety.armed?(state.bb.robot) do
      positions =
        Enum.map(rows, fn {_i, cur_pos, _cur_torq, set_pos} ->
          set_pos || cur_pos || 0.0
        end)

      max_speed = state.model_config.max_speed_rads
      # Use a conservative default for acceleration (rad/s²); the arm's own
      # motion planner will further clamp this per its firmware configuration.
      max_accel = max_speed * 10.0

      frame = Protocol.cmd_move_joints(state.txn_id, positions, max_speed, max_accel)

      case :gen_tcp.send(state.cmd_socket, frame) do
        :ok ->
          :ok

        {:error, reason} ->
          Logger.warning(
            "[BB.Ufactory.Controller] Joint move send failed for #{state.controller_name}: #{inspect(reason)}"
          )
      end

      %{state | txn_id: next_txn(state.txn_id)}
    else
      state
    end
  end

  defp drain_buffer(buffer, state) do
    case Report.parse_report(buffer) do
      {:ok, report, rest} ->
        state = update_from_report(report, state)
        drain_buffer(rest, %{state | buffer: rest})

      {:more} ->
        %{state | buffer: buffer}
    end
  end

  defp update_from_report(report, state) do
    joint_count = state.model_config.joints
    angles = Enum.take(report.angles, joint_count)
    torques = Enum.take(report.torques, joint_count)

    # Update per-joint ETS rows, preserving set_position
    angles
    |> Enum.zip(torques)
    |> Enum.with_index(1)
    |> Enum.each(fn {{angle, torque}, idx} ->
      set_pos =
        case :ets.lookup(state.ets, idx) do
          [{^idx, _cur, _torq, sp}] -> sp
          [] -> nil
        end

      :ets.insert(state.ets, {idx, angle, torque, set_pos})
    end)

    # Update arm-level row
    [x, y, z, roll, pitch, yaw] = report.pose
    :ets.insert(state.ets, {:arm, report.state, report.mode, {x, y, z, roll, pitch, yaw}})

    # Publish JointState
    joint_names = Enum.take(@all_joint_names, joint_count)

    case JointState.new(state.controller_name,
           names: joint_names,
           positions: angles,
           efforts: torques
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:sensor, state.controller_name], msg)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Controller] Failed to build JointState message: #{inspect(reason)}"
        )
    end

    # Publish CartesianPose
    case CartesianPose.new(state.controller_name,
           x: x,
           y: y,
           z: z,
           roll: roll,
           pitch: pitch,
           yaw: yaw
         ) do
      {:ok, msg} ->
        BB.publish(state.bb.robot, [:sensor, state.controller_name, :tcp_pose], msg)

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Controller] Failed to build CartesianPose message: #{inspect(reason)}"
        )
    end

    # Report errors on normal-report frames that include error_code
    state = maybe_report_error(report, state)

    state
  end

  defp maybe_report_error(%{error_code: error_code} = _report, state)
       when is_integer(error_code) and error_code != 0 and
              error_code != state.last_error_code do
    BB.Safety.report_error(
      state.bb.robot,
      state.bb.path,
      %{error_code: error_code, description: "xArm hardware error code #{error_code}"}
    )

    %{state | last_error_code: error_code}
  end

  defp maybe_report_error(%{error_code: 0}, state) do
    %{state | last_error_code: 0}
  end

  defp maybe_report_error(_report, state), do: state

  defp send_command(frame, state) do
    case :gen_tcp.send(state.cmd_socket, frame) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Controller] Command send failed for #{state.controller_name}: #{inspect(reason)}"
        )
    end

    %{state | txn_id: next_txn(state.txn_id)}
  end

  defp next_txn(txn_id), do: rem(txn_id + 1, 65_536)
end
