<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Getting Started with BB.Ufactory

This tutorial walks you through connecting a UFactory xArm6 to the
[BB robotics framework](https://hex.pm/packages/bb) using `bb_ufactory`.

## Prerequisites

- Elixir 1.19+ and Mix
- A UFactory xArm arm reachable on your network (or just a machine to run simulation)
- The arm's IP address (check UFactory's xArm Studio or your router's DHCP table)

## Installation

Add `bb_ufactory` to your `mix.exs` dependencies:

```elixir
def deps do
  [
    {:bb, "~> 0.33.2"},
    {:bb_ufactory, "~> 0.2"}
  ]
end
```

Then fetch:

```bash
mix deps.get
```

## Network Check (Hardware Only)

Before writing any code, confirm the arm is reachable. The command socket
lives on port 502:

```bash
# Confirm port 502 is open (replace with your arm's IP)
nc -zv 192.168.1.111 502
```

If the connection is refused, check:
1. The arm is powered on and the LED is blue (ready).
2. Your machine and the arm are on the same subnet.
3. No firewall is blocking port 502.

The report socket on port 30003 does not require an explicit check — the arm
starts pushing data automatically once the command socket handshake succeeds.

## Defining a Robot

### The Fast Path — `use BB.Ufactory.Robots.XArm6`

`BB.Ufactory.Robots.XArm6` is a pre-built robot module with all six joints
wired up. Supply the arm's IP address and you are done:

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6, host: "192.168.1.111"
end
```

Options: `:host`, `:loop_hz` (default 100), `:simulation` (default `:mock`),
plus `:gripper` and `:linear_track` for accessories (see the
[accessories tutorial](02-accessories.md)).

### From Scratch — `use BB`

If you need to customise joint names, add accessories, or compose the xArm
into a larger robot, define it directly with `use BB`:

```elixir
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
    end
  end
end
```

## Running in Simulation

Before connecting real hardware, verify the robot definition is correct by
starting in kinematic simulation mode. No arm required:

```elixir
{:ok, _pid} = MyRobot.start_link(simulation: :kinematic)
```

The controller, all actuators, and any sensors will start. Commands are accepted
and `BeginMotion` messages are published, but no TCP connections are opened.

## Arming and Moving a Joint

```elixir
# 1. Arm the robot (enables the joint motors on hardware)
:ok = BB.Safety.arm(MyRobot)

# 2. Move joint 1 to 45° (in radians: π/4 ≈ 0.785 rad)
:ok = BB.Actuator.set_position(MyRobot, :j1_motor, :math.pi() / 4)

# 3. Disarm when done
:ok = BB.Safety.disarm(MyRobot)
```

On hardware, `BB.Safety.arm/1` sends the enable command over port 502 and the arm
transitions from idle to ready. `BB.Safety.disarm/1` requests the configured stop
or motor disable.

If BB reports `{:error, {:disarm_failed, failures}}`, the driver could not
confirm disarm. Use the reported failure to diagnose the command connection
or firmware state. After verifying the hardware is safe and restoring
connectivity, an operator can call `BB.Safety.force_disarm/1` to acknowledge
the fault. BB 0.33.2 restarts a stopped topology and returns it to
`:disarmed` when reconnect succeeds.

## Subscribing to Joint State

The controller publishes a `BB.Message.Sensor.JointState` with all joint
positions on every report frame (~100 Hz on port 30003). Subscribe from any process:

```elixir
BB.subscribe(MyRobot, [:sensor, :xarm])

receive do
  {:bb, [:sensor, :xarm], %BB.Message{payload: %BB.Message.Sensor.JointState{} = js}} ->
    IO.inspect(hd(js.positions), label: "J1 position (rad)")
end
```

The Cartesian TCP pose is published to `[:sensor, :xarm, :tcp_pose]` as a
`BB.Ufactory.Message.CartesianPose` message.

## Next Steps

- Add accessories (gripper, F/T sensor, linear track): see [Accessories](02-accessories.md)
- Command the arm in Cartesian space: see [Cartesian Motion](03-cartesian-motion.md)
