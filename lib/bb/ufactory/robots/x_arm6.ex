# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Robots.XArm6 do
  @moduledoc """
  BB robot definition for the UFactory xArm6.

  6-DOF serial manipulator with revolute joints J1–J6. All joints rotate about
  their local Z axis. Joint limits and effort values are sourced from the
  xArm Developer Manual V1.10.0 and confirmed against the URDF in `tmp/urdf/xarm6/`.

  ## Joint Limits

  | Joint | Lower (rad) | Upper (rad) | Effort (N·m) | Max Velocity (rad/s) |
  |-------|------------|-------------|--------------|---------------------|
  | J1    | -2π        | +2π         | 50           | π                   |
  | J2    | -2.059     | +2.094      | 50           | π                   |
  | J3    | -3.927     | +0.192      | 32           | π                   |
  | J4    | -2π        | +2π         | 32           | π                   |
  | J5    | -1.693     | +π          | 32           | π                   |
  | J6    | -2π        | +2π         | 20           | π                   |

  ## Usage

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

  Or copy the topology into an existing robot module that also includes
  accessories (gripper, F/T sensor, linear track).
  """

  use BB
  import BB.Unit

  controllers do
    controller(
      :xarm,
      {BB.Ufactory.Controller, host: "192.168.1.111", model: :xarm6, loop_hz: 100},
      simulation: :mock
    )
  end

  topology do
    link :base do
      # J1 — base rotation. ±360°, 50 N·m, 180°/s
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
          # J2 — shoulder. -118° / +120° (-2.059 / +2.094 rad), 50 N·m
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
              # J3 — elbow. -225° / +11° (-3.927 / +0.192 rad), 32 N·m
              joint :j3 do
                type(:revolute)

                limit do
                  lower(~u(-225 degree))
                  upper(~u(11 degree))
                  effort(~u(32 newton_meter))
                  velocity(~u(180 degree_per_second))
                end

                actuator(:j3_motor, {BB.Ufactory.Actuator.Joint, joint: 3, controller: :xarm})

                link :link3 do
                  # J4 — forearm roll. ±360°, 32 N·m
                  joint :j4 do
                    type(:revolute)

                    limit do
                      lower(~u(-360 degree))
                      upper(~u(360 degree))
                      effort(~u(32 newton_meter))
                      velocity(~u(180 degree_per_second))
                    end

                    actuator(:j4_motor, {BB.Ufactory.Actuator.Joint, joint: 4, controller: :xarm})

                    link :link4 do
                      # J5 — wrist pitch. -97° / +180° (-1.693 / +π rad), 32 N·m
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
                          # J6 — wrist roll / TCP. ±360°, 20 N·m
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
    end
  end
end
