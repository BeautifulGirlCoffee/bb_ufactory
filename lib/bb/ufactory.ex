# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory do
  @moduledoc """
  Beam Bots integration for UFactory xArm robotic arms.

  `BB.Ufactory` provides controller, actuator, and sensor modules that plug into
  the [BB robotics framework](https://hex.pm/packages/bb) for UFactory xArm arms:
  **xArm5, xArm6, xArm7, Lite6, and xArm850**.

  ## Architecture

  The arm communicates over two independent TCP connections:

  | Socket | Port | Direction | Purpose |
  |--------|------|-----------|---------|
  | Command | 502 | Client → Arm | Send register commands; receive responses |
  | Report | 30003 | Arm → Client | Arm pushes joint state at ~100 Hz |

  `BB.Ufactory.Controller` owns both connections. An ETS table acts as a shared
  blackboard — joint actuators write target positions into it, and the controller's
  100 Hz loop batches all pending positions into a single `MOVE_JOINT` frame each
  tick. Cartesian and accessory commands (gripper, linear track) bypass the ETS
  loop and are sent immediately via `BB.Process.call/3`.

  ```
  Actuator.Joint  ──writes set_position──▶  ETS table
                                                │
  Controller loop (100 Hz) ──reads ETS──▶  cmd_move_joints frame ──▶ arm:502
  arm:30003 ──pushes report──▶ Controller ──updates ETS current_position
                                         ──publishes JointState, CartesianPose
  ```

  ## Quick Start

  The fastest path to a working xArm6 is to `use BB.Ufactory.Robots.XArm6`,
  which provides the complete six-joint topology and only requires a host address:

      defmodule MyRobot do
        use BB.Ufactory.Robots.XArm6

        controllers do
          controller :xarm, {BB.Ufactory.Controller,
            host: "192.168.1.111",
            model: :xarm6,
            loop_hz: 100
          }
        end
      end

  ## Full Robot Definition Example

  For custom topologies or to add accessories, define the robot with `use BB`
  directly. The example below combines all available components:

      defmodule MyRobot do
        use BB
        import BB.Unit

        controller :xarm, {BB.Ufactory.Controller,
          host: "192.168.1.111",
          model: :xarm6,
          loop_hz: 100
        }

        topology do
          link :base do
            joint :j1 do
              type :revolute
              limit do
                lower ~u(-360 degree)
                upper ~u(360 degree)
                effort ~u(50 newton_meter)
                velocity ~u(180 degree_per_second)
              end
              actuator :j1_motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm}

              link :link1 do
                joint :j2 do
                  type :revolute
                  limit do
                    lower ~u(-118 degree)
                    upper ~u(120 degree)
                    effort ~u(50 newton_meter)
                    velocity ~u(180 degree_per_second)
                  end
                  actuator :j2_motor, {BB.Ufactory.Actuator.Joint, joint: 2, controller: :xarm}

                  link :link2 do
                    joint :j3 do
                      type :revolute
                      limit do
                        lower ~u(-225 degree)
                        upper ~u(11 degree)
                        effort ~u(32 newton_meter)
                        velocity ~u(180 degree_per_second)
                      end
                      actuator :j3_motor, {BB.Ufactory.Actuator.Joint, joint: 3, controller: :xarm}

                      link :link3 do
                        joint :j4 do
                          type :revolute
                          limit do
                            lower ~u(-360 degree)
                            upper ~u(360 degree)
                            effort ~u(32 newton_meter)
                            velocity ~u(180 degree_per_second)
                          end
                          actuator :j4_motor, {BB.Ufactory.Actuator.Joint, joint: 4, controller: :xarm}

                          link :link4 do
                            joint :j5 do
                              type :revolute
                              limit do
                                lower ~u(-97 degree)
                                upper ~u(180 degree)
                                effort ~u(32 newton_meter)
                                velocity ~u(180 degree_per_second)
                              end
                              actuator :j5_motor, {BB.Ufactory.Actuator.Joint, joint: 5, controller: :xarm}

                              link :link5 do
                                joint :j6 do
                                  type :revolute
                                  limit do
                                    lower ~u(-360 degree)
                                    upper ~u(360 degree)
                                    effort ~u(20 newton_meter)
                                    velocity ~u(180 degree_per_second)
                                  end
                                  actuator :j6_motor, {BB.Ufactory.Actuator.Joint, joint: 6, controller: :xarm}

                                  link :link6 do
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

            # Optional: Cartesian actuator — commands end-effector pose directly
            actuator :cartesian, {BB.Ufactory.Actuator.Cartesian,
              controller: :xarm,
              speed: 100.0,
              acceleration: 2000.0
            }

            # Optional: Gripper G2 — position in pulse units (0–840)
            actuator :gripper, {BB.Ufactory.Actuator.Gripper,
              controller: :xarm,
              speed: 1500
            }

            # Optional: Linear track — position in millimetres
            actuator :track, {BB.Ufactory.Actuator.LinearTrack,
              controller: :xarm,
              speed: 200
            }
          end
        end

        sensors do
          # Optional: UFactory Force/Torque sensor — publishes BB.Ufactory.Message.Wrench
          sensor :wrench, {BB.Ufactory.Sensor.ForceTorque,
            controller: :xarm,
            poll_interval_ms: 20
          }
        end
      end

  ## Module Inventory

  | Module | Role |
  |--------|------|
  | `BB.Ufactory.Controller` | `BB.Controller` — TCP sockets, ETS table, 100 Hz loop, heartbeat, pubsub |
  | `BB.Ufactory.Protocol` | Frame builder and parser for port 502 commands |
  | `BB.Ufactory.Report` | Report frame parser for port 30003 push data |
  | `BB.Ufactory.Registers` | Module-attribute constants for all register addresses |
  | `BB.Ufactory.Model` | Per-model joint counts and limits (xArm5/6/7, Lite6, xArm850) |
  | `BB.Ufactory.Actuator.Joint` | `BB.Actuator` — joint-space position via ETS + 100 Hz loop |
  | `BB.Ufactory.Actuator.Cartesian` | `BB.Actuator` — Cartesian end-effector pose via `MOVE_LINE` |
  | `BB.Ufactory.Actuator.Gripper` | `BB.Actuator` — Gripper G2 position (pulse units 0–840) |
  | `BB.Ufactory.Actuator.LinearTrack` | `BB.Actuator` — linear track position in mm (RS485 proxy) |
  | `BB.Ufactory.Sensor.ForceTorque` | `BB.Sensor` — polls register 0xC8, publishes `Wrench` |
  | `BB.Ufactory.Message.ArmStatus` | State, mode, error/warning codes from the report socket |
  | `BB.Ufactory.Message.CartesianPose` | TCP pose (x/y/z/roll/pitch/yaw) from report socket |
  | `BB.Ufactory.Message.Wrench` | Fx/Fy/Fz/Tx/Ty/Tz from the F/T sensor |
  | `BB.Ufactory.Robots.XArm6` | Pre-built `BB` robot definition for the xArm6 |
  | `BB.Error.Protocol.Ufactory.HardwareFault` | Structured error for xArm hardware fault codes |
  | `BB.Error.Protocol.Ufactory.ConnectionError` | TCP connection failure |
  | `BB.Error.Protocol.Ufactory.CommandRejected` | Non-zero status byte in a command response |

  ## Protocol Notes

  - **Angles on the wire are always radians.** BB also uses radians, so no conversion is needed.
  - **Mixed endianness:** u16 header fields are big-endian; fp32 payload fields are little-endian.
    The only exception is the linear track position, which is int32 big-endian.
  - **Heartbeat:** The controller sends `<<0, 0, 0, 1, 0, 2, 0, 0>>` every second on the command
    socket to keep the connection alive.
  - **Modbus-TCP variant:** The protocol identifier in the header is `0x0002` (not the standard
    Modbus `0x0000`). The developer manual is the authoritative spec.
  """
end
