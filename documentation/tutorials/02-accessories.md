<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Accessories — Gripper, Force/Torque Sensor, and Linear Track

This tutorial covers the three accessories supported by `bb_ufactory`:

- **Gripper G2** — pneumatic/electric gripper via RS485 proxy (register 0x7C)
- **Force/Torque sensor** — 6-axis wrench via register 0xC8
- **Linear track** — motorised rail via RS485 proxy (int32 big-endian encoding)

Each accessory is an additional actuator or sensor declared in your robot module.
They all share the same `BB.Ufactory.Controller` instance.

## Gripper G2

The UFactory Gripper G2 connects to the arm's RS485 tool port. Commands are
proxied through the main TCP command socket via register 0x7C.

**Position units:** pulse units in the range **0–840**. The relationship to
physical jaw opening depends on the gripper model, but the full range spans from
fully closed (0) to fully open (840).

### Adding the Gripper

Declare the gripper actuator in your robot's topology:

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6

  controllers do
    controller :xarm, {BB.Ufactory.Controller, host: "192.168.1.111", model: :xarm6}
  end

  # Add at the base link level (outside the joint chain)
  topology do
    link :base do
      # ... joints j1–j6 ...

      actuator :gripper, {BB.Ufactory.Actuator.Gripper,
        controller: :xarm,
        speed: 1500         # pulse units per second; default: 1500
      }
    end
  end
end
```

On `init`, the gripper actuator automatically sends `cmd_gripper_enable(true)` to
the controller so the gripper is energised and ready before any position commands
arrive.

### Commanding the Gripper

Send a `%BB.Message.Actuator.Command.Position{}` with the target position in
pulse units:

```elixir
# Open gripper (840 = fully open)
BB.Process.cast(robot, :gripper, {:command,
  BB.Message.new!(BB.Message.Actuator.Command.Position, :gripper,
    position: 840.0
  )
})

# Close gripper (0 = fully closed)
BB.Process.cast(robot, :gripper, {:command,
  BB.Message.new!(BB.Message.Actuator.Command.Position, :gripper,
    position: 0.0
  )
})
```

Positions outside 0–840 are automatically clamped by the actuator.

### Disarm Behaviour

When the robot is disarmed, `Actuator.Gripper` opens a **fresh TCP connection**
directly to the arm (bypassing the controller GenServer) and sends
`cmd_gripper_enable(false)`. This ensures the gripper releases reliably even
if the controller has crashed.

---

## Force/Torque Sensor

The UFactory F/T sensor attaches to the tool flange and reports six-axis wrench
data: forces Fx/Fy/Fz (Newtons) and torques Tx/Ty/Tz (Newton-metres).

The sensor is polled at a configurable rate (default 50 Hz) by sending
`cmd_get_ft_data()` (register 0xC8) via the controller.

### Adding the Sensor

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6

  controllers do
    controller :xarm, {BB.Ufactory.Controller, host: "192.168.1.111", model: :xarm6}
  end

  sensors do
    sensor :wrench, {BB.Ufactory.Sensor.ForceTorque,
      controller: :xarm,
      poll_interval_ms: 20    # 50 Hz; default: 20 ms
    }
  end
end
```

On `init`, the sensor sends `cmd_ft_sensor_enable(true)` to activate the hardware.

### Subscribing to Wrench Messages

The sensor publishes `BB.Ufactory.Message.Wrench` to the sensor's pubsub path:

```elixir
BB.subscribe(robot, [:sensor, :wrench])

receive do
  {:bb, [:sensor, :wrench], %BB.Message{payload: %BB.Ufactory.Message.Wrench{} = w}} ->
    IO.puts("Fx: #{w.fx} N, Fy: #{w.fy} N, Fz: #{w.fz} N")
    IO.puts("Tx: #{w.tx} Nm, Ty: #{w.ty} Nm, Tz: #{w.tz} Nm")
end
```

### Disarm Behaviour

On disarm, the sensor sends `cmd_ft_sensor_enable(false)` via a fresh TCP
connection to deactivate the hardware.

---

## Linear Track

The UFactory linear track is a motorised rail that the arm base slides along.
It connects via the arm's RS485 bus and is controlled through the same TCP
command socket.

**Position units:** millimetres. The protocol encodes position as an `int32`
big-endian value where `raw = round(mm * 2000)`. This is the only place in the
UFactory protocol where a position is not little-endian fp32 — it is handled
transparently by `BB.Ufactory.Protocol.cmd_linear_track_move/3`.

### Adding the Linear Track

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6

  controllers do
    controller :xarm, {BB.Ufactory.Controller, host: "192.168.1.111", model: :xarm6}
  end

  topology do
    link :base do
      # ... joints j1–j6 ...

      actuator :track, {BB.Ufactory.Actuator.LinearTrack,
        controller: :xarm,
        speed: 200          # mm/s; default: 200
      }
    end
  end
end
```

### Commanding the Linear Track

Position is given in millimetres:

```elixir
# Move track to 500 mm from the home position
BB.Process.cast(robot, :track, {:command,
  BB.Message.new!(BB.Message.Actuator.Command.Position, :track,
    position: 500.0
  )
})
```

Under the hood, the actuator sends two sequential frames: a speed-set frame
followed by a position-set frame. Speed must arrive first so the arm uses the
updated speed for the move.

Unlike joint actuators, linear track commands **bypass the ETS batch loop**
and are forwarded immediately via `BB.Process.call/3`.

---

## Combined Robot With All Accessories

The following is a complete robot definition that includes the xArm6 joints,
gripper, F/T sensor, and linear track:

```elixir
defmodule BaristaBotRobot do
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
          lower ~u(-360 degree); upper ~u(360 degree)
          effort ~u(50 newton_meter); velocity ~u(180 degree_per_second)
        end
        actuator :j1_motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm}

        link :link1 do
          joint :j2 do
            type :revolute
            limit do
              lower ~u(-118 degree); upper ~u(120 degree)
              effort ~u(50 newton_meter); velocity ~u(180 degree_per_second)
            end
            actuator :j2_motor, {BB.Ufactory.Actuator.Joint, joint: 2, controller: :xarm}

            link :link2 do
              joint :j3 do
                type :revolute
                limit do
                  lower ~u(-225 degree); upper ~u(11 degree)
                  effort ~u(32 newton_meter); velocity ~u(180 degree_per_second)
                end
                actuator :j3_motor, {BB.Ufactory.Actuator.Joint, joint: 3, controller: :xarm}

                link :link3 do
                  joint :j4 do
                    type :revolute
                    limit do
                      lower ~u(-360 degree); upper ~u(360 degree)
                      effort ~u(32 newton_meter); velocity ~u(180 degree_per_second)
                    end
                    actuator :j4_motor, {BB.Ufactory.Actuator.Joint, joint: 4, controller: :xarm}

                    link :link4 do
                      joint :j5 do
                        type :revolute
                        limit do
                          lower ~u(-97 degree); upper ~u(180 degree)
                          effort ~u(32 newton_meter); velocity ~u(180 degree_per_second)
                        end
                        actuator :j5_motor, {BB.Ufactory.Actuator.Joint, joint: 5, controller: :xarm}

                        link :link5 do
                          joint :j6 do
                            type :revolute
                            limit do
                              lower ~u(-360 degree); upper ~u(360 degree)
                              effort ~u(20 newton_meter); velocity ~u(180 degree_per_second)
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

      actuator :gripper, {BB.Ufactory.Actuator.Gripper, controller: :xarm, speed: 1500}
      actuator :track,   {BB.Ufactory.Actuator.LinearTrack, controller: :xarm, speed: 200}
    end
  end

  sensors do
    sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm, poll_interval_ms: 20}
  end
end
```

## Next Steps

- Command the arm in Cartesian space instead of joint-space: see [Cartesian Motion](03-cartesian-motion.md)
