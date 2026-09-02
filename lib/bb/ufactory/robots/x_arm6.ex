# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Robots.XArm6.Definition do
  @moduledoc false

  # Single source of the xArm6 robot DSL, shared by the concrete
  # BB.Ufactory.Robots.XArm6 module and by `use BB.Ufactory.Robots.XArm6`.
  # Every option must be resolvable at compile time.
  defmacro define(opts \\ []) do
    opts =
      Keyword.validate!(opts,
        host: "192.168.1.111",
        loop_hz: 100,
        simulation: :mock,
        controller: [],
        cartesian: true,
        gripper: false,
        linear_track: false
      )

    controller_opts =
      Keyword.merge(
        [host: opts[:host], model: :xarm6, loop_hz: opts[:loop_hz]],
        opts[:controller]
      )

    cartesian_mount =
      mount_ast(
        :cartesian_mount,
        :cartesian_body,
        :cartesian,
        BB.Ufactory.Actuator.Cartesian,
        opts[:cartesian]
      )

    gripper_mount =
      mount_ast(
        :gripper_mount,
        :gripper_body,
        :gripper,
        BB.Ufactory.Actuator.Gripper,
        opts[:gripper]
      )

    track_mount =
      mount_ast(
        :track_mount,
        :track_body,
        :track,
        BB.Ufactory.Actuator.LinearTrack,
        opts[:linear_track]
      )

    quote do
      use BB
      import BB.Unit

      controllers do
        controller(
          :xarm,
          {BB.Ufactory.Controller, unquote(controller_opts)},
          simulation: unquote(opts[:simulation])
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

                        actuator(
                          :j4_motor,
                          {BB.Ufactory.Actuator.Joint, joint: 4, controller: :xarm}
                        )

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
                                  (unquote_splicing(cartesian_mount ++ gripper_mount))
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

          unquote_splicing(track_mount)
        end
      end
    end
  end

  # Accessory actuators hang off fixed mount joints: bb's DSL only allows
  # actuators under joints, and a fixed joint carries no limits, so the
  # mount neither warns about missing position feedback nor implies the
  # accessory's pose is tracked in the kinematic tree (gripper and track
  # positions are commanded, not streamed back).
  defp mount_ast(_joint, _link, _name, _module, disabled) when disabled in [false, nil], do: []

  defp mount_ast(joint, link, name, module, true), do: mount_ast(joint, link, name, module, [])

  defp mount_ast(joint, link, name, module, extra) when is_list(extra) do
    opts = Keyword.merge([controller: :xarm], extra)

    [
      quote do
        joint unquote(joint) do
          type(:fixed)

          actuator(unquote(name), {unquote(module), unquote(opts)})

          link unquote(link) do
          end
        end
      end
    ]
  end
end

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

  Build your own robot module from this definition, overriding what you need:

      defmodule MyRobot do
        use BB.Ufactory.Robots.XArm6, host: "192.168.1.111"
      end

  ### Options

  * `:host` — the arm's IP address (default `"192.168.1.111"`)
  * `:loop_hz` — motion-loop rate for the `:xarm` controller (default `100`)
  * `:simulation` — the controller's simulation strategy (default `:mock`)
  * `:controller` — extra `BB.Ufactory.Controller` options merged into the
    child spec, e.g. `[tcp_offset: {0.0, 0.0, 172.0, 0.0, 0.0, 0.0},
    reduced_mode: true]`
  * `:cartesian` — a `:cartesian` actuator (`BB.Ufactory.Actuator.Cartesian`)
    on a fixed mount at `:link6`, for `CartesianMove` commands. **Enabled by
    default** (it needs no extra hardware); pass `false` to omit, or a
    keyword list of actuator options (e.g. `[speed: 150.0]`)
  * `:gripper` — `true` (or a keyword list of `BB.Ufactory.Actuator.Gripper`
    options, e.g. `[speed: 1500]`) adds a `:gripper` actuator on a fixed
    mount joint at `:link6` (the TCP)
  * `:linear_track` — `true` (or `BB.Ufactory.Actuator.LinearTrack` options,
    e.g. `[speed: 200]`) adds a `:track` actuator on a fixed mount joint at
    the base link

  Sensors compose without options — declare your own `sensors do ... end`
  block alongside the `use`:

      defmodule MyRobot do
        use BB.Ufactory.Robots.XArm6, host: "192.168.1.111", gripper: true

        sensors do
          sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm}
        end
      end

  For a topology this definition cannot express (renamed joints, the arm
  mounted on a larger robot), copy the DSL from this module's source into
  your own `use BB` robot.

  This module is itself a complete robot (host `192.168.1.111`) and can be
  started directly, which the library's own simulator tests do.
  """

  require BB.Ufactory.Robots.XArm6.Definition

  BB.Ufactory.Robots.XArm6.Definition.define()

  defmacro __using__(opts) do
    quote do
      require BB.Ufactory.Robots.XArm6.Definition
      BB.Ufactory.Robots.XArm6.Definition.define(unquote(opts))
    end
  end
end
