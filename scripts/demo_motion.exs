# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

# Demo script — combines joint motion, gripper actuation, and linear track travel.
#
# Usage:
#     XARM_HOST=192.168.1.224 mix run scripts/demo_motion.exs
#
# Safety: Uses slow speeds and small movements. Press Ctrl-C to abort at any time;
# the arm will hold position until the next heartbeat timeout (~10s) then coast.

alias BB.Ufactory.Protocol
alias BB.Ufactory.Report

host = System.get_env("XARM_HOST", "192.168.1.224") |> String.to_charlist()

# ── Home position (captured from live arm) ──────────────────────────────────

home_angles = [-0.0556, -1.4866, -0.2007, -0.0001, 0.5322, -0.0523]
home_track = 900.0

# ── Connection helpers ──────────────────────────────────────────────────────

{:ok, cmd} =
  :gen_tcp.connect(host, 502, [:binary, active: false, packet: :raw], 5_000)

send_recv = fn frame, timeout ->
  :ok = :gen_tcp.send(cmd, frame)
  {:ok, data} = :gen_tcp.recv(cmd, 0, timeout)
  {:ok, resp, _rest} = Protocol.parse_response(data)
  resp
end

send_cmd = fn frame -> send_recv.(frame, 5_000) end

read_angles = fn ->
  {:ok, sock} = :gen_tcp.connect(host, 30003, [:binary, active: false, packet: :raw], 5_000)
  {:ok, data} = :gen_tcp.recv(sock, 0, 2_000)

  {report, _rest} =
    case Report.parse_report(data) do
      {:ok, r, rest} ->
        {r, rest}

      {:more} ->
        {:ok, more} = :gen_tcp.recv(sock, 0, 2_000)
        {:ok, r, rest} = Report.parse_report(data <> more)
        {r, rest}
    end

  :gen_tcp.close(sock)
  Enum.take(report.angles, 6)
end

flush_socket = fn ->
  case :gen_tcp.recv(cmd, 0, 50) do
    {:ok, _} -> :ok
    {:error, :timeout} -> :ok
  end
end

read_track_pos = fn ->
  flush_socket.()
  payload = <<0x0B, 0x01, 0x03, 0x0A, 0x20, 0x00, 0x02>>
  {0x7C, _status, params} = send_cmd.(Protocol.build_frame(99, 0x7C, payload))
  <<_host::8, _dev::8, 0x03, _bc::8, pos::signed-32>> = params
  pos / 2000
end

move_joints = fn angles, speed, accel ->
  :ok = :gen_tcp.send(cmd, Protocol.cmd_move_joints(0, angles, speed, accel))
end

move_track = fn pos_mm, speed ->
  {pos_frame, spd_frame} = Protocol.cmd_linear_track_move(0, pos_mm, speed)
  send_cmd.(spd_frame)
  send_cmd.(pos_frame)
end

log = fn msg ->
  ts = DateTime.utc_now() |> Calendar.strftime("%H:%M:%S")
  IO.puts("[#{ts}] #{msg}")
end

# ── Initialise arm ──────────────────────────────────────────────────────────

log.("Clearing errors and enabling arm...")
send_cmd.(Protocol.cmd_clean_error(0))
send_cmd.(Protocol.cmd_enable(1, true))
send_cmd.(Protocol.cmd_set_mode(2, 0))
send_cmd.(Protocol.cmd_set_state(3, 0))
Process.sleep(500)

log.("Enabling gripper...")
send_recv.(Protocol.cmd_gripper_enable(4, true), 10_000)
Process.sleep(500)

log.("Enabling linear track...")
send_cmd.(Protocol.cmd_linear_track_enable(5, true))
Process.sleep(500)

# ── Read starting position ──────────────────────────────────────────────────

start_angles = read_angles.()
start_track = read_track_pos.()

log.(
  "Starting joints: #{inspect(Enum.map(start_angles, &Float.round(&1, 3)))}"
)

log.("Starting track:  #{Float.round(start_track, 1)} mm")

# ── Phase 1: Home to known position ────────────────────────────────────────

log.("Homing joints to starting position...")
move_joints.(home_angles, 0.3, 2.0)
Process.sleep(3_000)

log.("Homing track to #{home_track} mm...")
move_track.(home_track, 100)
Process.sleep(3_000)

log.("Home position reached.")

# ── Phase 2: Gripper open/close cycle ───────────────────────────────────────

log.("Opening gripper...")
:ok = :gen_tcp.send(cmd, Protocol.cmd_gripper_position(0, 840))
Process.sleep(1_500)

log.("Closing gripper...")
:ok = :gen_tcp.send(cmd, Protocol.cmd_gripper_position(0, 0))
Process.sleep(1_500)

log.("Opening gripper...")
:ok = :gen_tcp.send(cmd, Protocol.cmd_gripper_position(0, 840))
Process.sleep(1_500)

# ── Phase 3: Joint wave — small movements through each joint ───────────────

log.("Starting joint wave sequence...")

offsets = [0.15, 0.10, 0.10, 0.15, 0.10, 0.20]

for {offset, idx} <- Enum.with_index(offsets) do
  joint_name = "J#{idx + 1}"

  target = List.update_at(home_angles, idx, &(&1 + offset))
  log.("  #{joint_name} +#{offset} rad")
  move_joints.(target, 0.3, 2.0)
  Process.sleep(1_200)

  log.("  #{joint_name} back")
  move_joints.(home_angles, 0.3, 2.0)
  Process.sleep(1_200)
end

log.("Joint wave complete.")

# ── Phase 4: Close gripper, then track traverse ─────────────────────────────

log.("Closing gripper for travel...")
:ok = :gen_tcp.send(cmd, Protocol.cmd_gripper_position(0, 0))
Process.sleep(1_500)

log.("Track → 100 mm (near home end)...")
move_track.(100.0, 200)

# Poll until the track arrives or timeout
deadline = System.monotonic_time(:millisecond) + 15_000

Stream.repeatedly(fn ->
  Process.sleep(500)
  read_track_pos.()
end)
|> Enum.reduce_while(nil, fn pos, _acc ->
  if abs(pos - 100.0) < 5.0 or System.monotonic_time(:millisecond) > deadline do
    log.("  Track at #{Float.round(pos, 1)} mm")
    {:halt, pos}
  else
    {:cont, nil}
  end
end)

log.("Track → #{home_track} mm (return)...")
move_track.(home_track, 200)

deadline = System.monotonic_time(:millisecond) + 15_000

Stream.repeatedly(fn ->
  Process.sleep(500)
  read_track_pos.()
end)
|> Enum.reduce_while(nil, fn pos, _acc ->
  if abs(pos - home_track) < 5.0 or System.monotonic_time(:millisecond) > deadline do
    log.("  Track at #{Float.round(pos, 1)} mm")
    {:halt, pos}
  else
    {:cont, nil}
  end
end)

# ── Phase 5: Final gripper flourish and return home ─────────────────────────

log.("Opening gripper...")
:ok = :gen_tcp.send(cmd, Protocol.cmd_gripper_position(0, 840))
Process.sleep(1_000)

log.("Closing gripper...")
:ok = :gen_tcp.send(cmd, Protocol.cmd_gripper_position(0, 0))
Process.sleep(1_000)

# ── Cleanup ─────────────────────────────────────────────────────────────────

log.("Returning to home position...")
move_joints.(home_angles, 0.3, 2.0)
Process.sleep(2_000)

log.("Disabling gripper...")
send_recv.(Protocol.cmd_gripper_enable(0, false), 10_000)

log.("Done! Arm is at home position, motors remain enabled.")

final_angles = read_angles.()
final_track = read_track_pos.()

log.(
  "Final joints: #{inspect(Enum.map(final_angles, &Float.round(&1, 3)))}"
)

log.("Final track:  #{Float.round(final_track, 1)} mm")

:gen_tcp.close(cmd)
