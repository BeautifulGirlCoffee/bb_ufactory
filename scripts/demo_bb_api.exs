# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

# Demo script — exercises the xArm6 through the BB framework's high-level API.
#
# Contrasts with demo_motion.exs which uses raw gen_tcp + Protocol frames.
# Here the controller, ETS, heartbeat, report socket, arming sequence, and
# protocol encoding are all handled automatically by the BB supervision tree.
#
# Usage:
#     XARM_HOST=192.168.1.224 mix run scripts/demo_bb_api.exs
#
# Safety: Uses slow speeds and small movements. Press Ctrl-C to abort;
# BB.Safety disarm callbacks will attempt to stop the arm cleanly.

require Logger

host = System.get_env("XARM_HOST", "192.168.1.224")

log = fn msg ->
  ts = DateTime.utc_now() |> Calendar.strftime("%H:%M:%S")
  IO.puts("[#{ts}] #{msg}")
end

# ── Robot definition ─────────────────────────────────────────────────────────
#
# Define a robot module inline with the real controller pointed at hardware.
# The topology models the full kinematic chain:
#   linear track (prismatic) → J1..J6 (revolute) → gripper (prismatic)
#
# Each actuator is wired to the :xarm controller and managed automatically
# by the BB supervision tree — including safety registration for disarm.

defmodule DemoRobot do
  use BB
  import BB.Unit

  controllers do
    controller(
      :xarm,
      {BB.Ufactory.Controller,
       host: System.get_env("XARM_HOST", "192.168.1.224"), model: :xarm6, loop_hz: 100}
    )
  end

  topology do
    link :track_mount do
      joint :track do
        type(:prismatic)

        limit do
          lower(~u(0 millimeter))
          upper(~u(1000 millimeter))
          effort(~u(100 newton))
          velocity(~u(500 millimeter_per_second))
        end

        actuator(
          :linear_track,
          {BB.Ufactory.Actuator.LinearTrack, controller: :xarm, speed: 200}
        )

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
              joint :j2 do
                type(:revolute)

                limit do
                  lower(~u(-118 degree))
                  upper(~u(120 degree))
                  effort(~u(50 newton_meter))
                  velocity(~u(180 degree_per_second))
                end

                actuator(:j2_motor, {BB.Ufactory.Actuator.Joint, joint: 2, controller: :xarm})

                link :link2 do
                  joint :j3 do
                    type(:revolute)

                    limit do
                      lower(~u(-225 degree))
                      upper(~u(11 degree))
                      effort(~u(32 newton_meter))
                      velocity(~u(180 degree_per_second))
                    end

                    actuator(
                      :j3_motor,
                      {BB.Ufactory.Actuator.Joint, joint: 3, controller: :xarm}
                    )

                    link :link3 do
                      joint :j4 do
                        type(:revolute)

                        limit do
                          lower(~u(-360 degree))
                          upper(~u(360 degree))
                          effort(~u(32 newton_meter))
                          velocity(~u(180 degree_per_second))
                        end

                        actuator(
                          :j4_motor,
                          {BB.Ufactory.Actuator.Joint, joint: 4, controller: :xarm}
                        )

                        link :link4 do
                          joint :j5 do
                            type(:revolute)

                            limit do
                              lower(~u(-97 degree))
                              upper(~u(180 degree))
                              effort(~u(32 newton_meter))
                              velocity(~u(180 degree_per_second))
                            end

                            actuator(
                              :j5_motor,
                              {BB.Ufactory.Actuator.Joint, joint: 5, controller: :xarm}
                            )

                            link :link5 do
                              joint :j6 do
                                type(:revolute)

                                limit do
                                  lower(~u(-360 degree))
                                  upper(~u(360 degree))
                                  effort(~u(20 newton_meter))
                                  velocity(~u(180 degree_per_second))
                                end

                                actuator(
                                  :j6_motor,
                                  {BB.Ufactory.Actuator.Joint, joint: 6, controller: :xarm}
                                )

                                actuator(
                                  :cartesian,
                                  {BB.Ufactory.Actuator.Cartesian,
                                   controller: :xarm, speed: 50.0}
                                )

                                link :link6 do
                                  joint :gripper_joint do
                                    type(:prismatic)

                                    limit do
                                      lower(~u(0 millimeter))
                                      upper(~u(840 millimeter))
                                      effort(~u(20 newton))
                                      velocity(~u(1500 millimeter_per_second))
                                    end

                                    actuator(
                                      :gripper,
                                      {BB.Ufactory.Actuator.Gripper, controller: :xarm}
                                    )

                                    link(:end_effector, do: nil)
                                  end
                                end
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
    end
  end
end

alias BB.Message
alias BB.Message.Sensor.JointState
alias BB.Ufactory.Message.CartesianPose

# ── Start the robot ──────────────────────────────────────────────────────────

log.("Starting BB supervision tree for DemoRobot (host: #{host})...")
{:ok, _pid} = DemoRobot.start_link([])

# Give the controller time to connect both sockets and start the report stream
Process.sleep(1_000)

log.("Robot started. Safety state: #{BB.Safety.state(DemoRobot)}")

# ── Subscribe to sensor data ────────────────────────────────────────────────
#
# The controller automatically publishes JointState at 100 Hz from the report
# socket. We subscribe so we can read current positions.

BB.subscribe(DemoRobot, [:sensor, :xarm])

read_joints = fn ->
  Process.sleep(50)

  Stream.repeatedly(fn ->
    receive do
      {:bb, [:sensor, :xarm], %Message{payload: %JointState{} = js}} -> js
    after
      0 -> nil
    end
  end)
  |> Enum.take_while(&(&1 != nil))
  |> List.last()
end

# ── Arm the robot ────────────────────────────────────────────────────────────
#
# This single call triggers the full init sequence we discovered during
# hardware testing: clean_error → enable → set_mode(0) → set_state(0).

log.("Arming robot...")
:ok = BB.Safety.arm(DemoRobot)
Process.sleep(500)
log.("Armed! Safety state: #{BB.Safety.state(DemoRobot)}")

# ── Read current joint positions ─────────────────────────────────────────────

js = read_joints.()

if js do
  angles_str =
    js.positions
    |> Enum.zip(js.names)
    |> Enum.map(fn {pos, name} -> "  #{name}: #{Float.round(pos, 4)} rad" end)
    |> Enum.join("\n")

  log.("Current joint positions:\n#{angles_str}")
else
  log.("Warning: No JointState received yet")
end

# ── Joint wave — small movements through each joint ─────────────────────────
#
# BB.Actuator.set_position! sends a position command directly to the named
# actuator. The Joint actuator clamps to limits, writes to ETS, and the
# controller's 100 Hz loop batches into protocol frames automatically.

offsets = [
  {:j1_motor, 0.10},
  {:j2_motor, 0.08},
  {:j3_motor, 0.08},
  {:j4_motor, 0.10},
  {:j5_motor, 0.08},
  {:j6_motor, 0.12}
]

joint_name_for_motor = %{
  j1_motor: :j1,
  j2_motor: :j2,
  j3_motor: :j3,
  j4_motor: :j4,
  j5_motor: :j5,
  j6_motor: :j6
}

initial_positions =
  if js do
    Enum.zip(js.names, js.positions) |> Map.new()
  else
    %{}
  end

log.("Starting joint wave sequence...")

for {motor, offset} <- offsets do
  joint_name = Map.fetch!(joint_name_for_motor, motor)
  initial = Map.get(initial_positions, joint_name, 0.0)
  target = initial + offset

  log.("  #{motor} → #{Float.round(target, 4)} rad (+#{offset})")
  BB.Actuator.set_position!(DemoRobot, motor, target)
  Process.sleep(1_500)

  log.("  #{motor} → #{Float.round(initial, 4)} rad (return)")
  BB.Actuator.set_position!(DemoRobot, motor, initial)
  Process.sleep(1_500)
end

log.("Joint wave complete.")

# ── Gripper ──────────────────────────────────────────────────────────────────
#
# The gripper is wired into the topology as a prismatic joint with actuator
# name :gripper. BB.Actuator.set_position! sends a Command.Position directly
# to the Gripper actuator, which converts to a protocol frame and sends
# through the controller. Position is in pulse units (0 = closed, 840 = open).

log.("Opening gripper...")
BB.Actuator.set_position!(DemoRobot, :gripper, 840.0)
Process.sleep(1_500)

log.("Closing gripper...")
BB.Actuator.set_position!(DemoRobot, :gripper, 0.0)
Process.sleep(1_500)

log.("Opening gripper...")
BB.Actuator.set_position!(DemoRobot, :gripper, 840.0)
Process.sleep(1_500)

# ── Linear track ─────────────────────────────────────────────────────────────
#
# The linear track is wired as a prismatic joint at the base of the chain.
# BB.Actuator.set_position! sends the target position in mm.
# The LinearTrack actuator reads current position, sends speed + position
# frames through the controller, and publishes BeginMotion with real travel
# distance — all from a single function call.

log.("Closing gripper for travel...")
BB.Actuator.set_position!(DemoRobot, :gripper, 0.0)
Process.sleep(1_500)

log.("Moving track 50 mm forward...")
BB.subscribe(DemoRobot, [:actuator])

# Read current track position from the actuator's initial_position in BeginMotion
BB.Actuator.set_position!(DemoRobot, :linear_track, 950.0)
Process.sleep(3_000)

log.("Moving track back...")
BB.Actuator.set_position!(DemoRobot, :linear_track, 900.0)
Process.sleep(3_000)

# ── Multi-joint simultaneous movement ────────────────────────────────────────
#
# Set all joints at once. The 100 Hz control loop batches every pending
# set_position into a single cmd_move_joints frame, so the arm moves all
# joints in coordination — unlike the wave sequence which moved one at a time.

motors = [:j1_motor, :j2_motor, :j3_motor, :j4_motor, :j5_motor, :j6_motor]
multi_offsets = [0.05, 0.04, 0.04, 0.05, 0.04, 0.06]

log.("Starting multi-joint simultaneous movement...")

for {motor, offset} <- Enum.zip(motors, multi_offsets) do
  joint_name = Map.fetch!(joint_name_for_motor, motor)
  initial = Map.get(initial_positions, joint_name, 0.0)
  BB.Actuator.set_position!(DemoRobot, motor, initial + offset)
end

log.("  All 6 joints shifted simultaneously")
Process.sleep(2_000)

for motor <- motors do
  joint_name = Map.fetch!(joint_name_for_motor, motor)
  initial = Map.get(initial_positions, joint_name, 0.0)
  BB.Actuator.set_position!(DemoRobot, motor, initial)
end

log.("  All 6 joints returned simultaneously")
Process.sleep(2_000)
log.("Multi-joint move complete.")

# ── Cartesian motion ────────────────────────────────────────────────────────
#
# The Cartesian actuator commands the whole arm in task space (x,y,z,r,p,y)
# using the arm's built-in IK solver. It's wired as a second actuator on
# the j6 joint and accessed via BB.Process.cast.

BB.subscribe(DemoRobot, [:sensor, :xarm, :tcp_pose])
Process.sleep(100)

pose =
  Stream.repeatedly(fn ->
    receive do
      {:bb, [:sensor, :xarm, :tcp_pose], %Message{payload: %CartesianPose{} = p}} -> p
    after
      0 -> nil
    end
  end)
  |> Enum.take_while(&(&1 != nil))
  |> List.last()

if pose do
  log.(
    "Current TCP pose: x=#{Float.round(pose.x, 1)}, y=#{Float.round(pose.y, 1)}, z=#{Float.round(pose.z, 1)} mm"
  )

  log.("  Moving TCP +20mm X, +20mm Y...")

  BB.Process.cast(
    DemoRobot,
    :cartesian,
    {:move_cartesian, {pose.x + 20.0, pose.y + 20.0, pose.z, pose.roll, pose.pitch, pose.yaw}}
  )

  Process.sleep(2_000)

  log.("  Returning TCP to original position...")

  BB.Process.cast(
    DemoRobot,
    :cartesian,
    {:move_cartesian, {pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw}}
  )

  Process.sleep(2_000)
  log.("Cartesian motion complete.")
else
  log.("Warning: No CartesianPose received, skipping Cartesian motion")
end

# ── Final gripper flourish ──────────────────────────────────────────────────

log.("Opening gripper...")
BB.Actuator.set_position!(DemoRobot, :gripper, 840.0)
Process.sleep(1_000)

log.("Closing gripper...")
BB.Actuator.set_position!(DemoRobot, :gripper, 0.0)
Process.sleep(1_000)

# ── Read final positions ─────────────────────────────────────────────────────

final_js = read_joints.()

if final_js do
  final_str =
    final_js.positions
    |> Enum.zip(final_js.names)
    |> Enum.map(fn {pos, name} -> "  #{name}: #{Float.round(pos, 4)} rad" end)
    |> Enum.join("\n")

  log.("Final joint positions:\n#{final_str}")
end

# ── Disarm and clean up ──────────────────────────────────────────────────────
#
# BB.Safety.disarm triggers all registered disarm callbacks automatically:
# - Controller sends cmd_stop (or hold)
# - Gripper actuator sends cmd_gripper_enable(false)
# - LinearTrack actuator sends cmd_linear_track_enable(false)
# All from a single call.

log.("Disarming robot...")
BB.Safety.disarm(DemoRobot)
Process.sleep(500)
log.("Safety state: #{BB.Safety.state(DemoRobot)}")

log.("Stopping supervision tree...")
Supervisor.stop(DemoRobot, :normal, 5_000)

log.("Done!")
