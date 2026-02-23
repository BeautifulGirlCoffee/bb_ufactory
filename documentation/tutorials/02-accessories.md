<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Accessories — Gripper, Force/Torque Sensor, Linear Track, Collision Detection, and Tool Configuration

This tutorial covers the accessories and hardware configuration options supported
by `bb_ufactory`:

- **Gripper G2** — pneumatic/electric gripper via RS485 proxy (register 0x7C)
- **Force/Torque sensor** — 6-axis wrench via register 0xC8
- **Linear track** — motorised rail via RS485 proxy (int32 big-endian encoding)
- **Collision detection** — firmware collision sensitivity and event subscription
- **TCP tool configuration** — tool offset, payload, reduced mode, workspace fence

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

## Collision Detection

`BB.Ufactory.Sensor.Collision` configures the arm's firmware collision detection
sensitivity and subscribes to `ArmStatus` events from the controller. When a
collision is detected (error codes 22, 31, or 35), the sensor re-publishes the
`ArmStatus` message to its own sensor path so application code can respond.

### Detected Error Codes

| Code | Meaning |
|------|---------|
| 22 | Self-collision (arm would intersect itself) |
| 31 | Collision caused abnormal current (external contact) |
| 35 | Safety boundary limit (TCP exited workspace fence) |

### Adding the Collision Sensor

```elixir
sensors do
  sensor :collision, {BB.Ufactory.Sensor.Collision,
    controller: :xarm,
    sensitivity: 3,              # 0 = disabled, 5 = most sensitive; nil = leave unchanged
    rebound: false,              # whether arm reverses after collision; nil = leave unchanged
    self_collision_check: true   # geometric self-collision model; nil = leave unchanged
  }
end
```

On `init`, the sensor sends the collision configuration commands to the arm via
the controller. Settings are persisted in the arm's NVRAM and survive a reboot.

### Subscribing to Collision Events

```elixir
BB.subscribe(MyRobot, [:sensor, :collision])

receive do
  {:bb, [:sensor, :collision],
   %BB.Message{payload: %BB.Ufactory.Message.ArmStatus{error_code: code}}} ->
    IO.puts("Collision event, error_code: #{code}")
end
```

### Sensitivity Scale

| Level | Behaviour |
|-------|-----------|
| `0` | Collision detection disabled |
| `1` | Lowest (very hard to trigger; tolerates heavy contact) |
| `3` | Balanced — recommended for most applications |
| `5` | Highest (easiest to trigger; stops on light contact) |

Tune sensitivity based on your payload weight and environment. A heavier tool
exerts more force on the arm's joints during motion, so lower sensitivity
reduces false positives.

### Disarm Behaviour

Collision detection settings are persistent firmware state. No hardware action
is taken on disarm.

---

## TCP Tool Configuration

The controller accepts optional options that configure the arm's tool geometry
and payload. These are sent once to the arm during `init`, immediately after
the TCP connections are established. All values are persisted in NVRAM.

### Tool Center Point Offset (`tcp_offset`)

Tells the arm where the tool tip is relative to the flange. Accurate TCP
configuration is required for Cartesian motion and force/torque readings in
tool coordinates.

```elixir
controller :xarm, {BB.Ufactory.Controller,
  host: "192.168.1.111",
  model: :xarm6,
  tcp_offset: {0.0, 0.0, 172.0, 0.0, 0.0, 0.0}
  # {x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad}
}
```

Omit `tcp_offset` (or pass `nil`) to leave the arm's current setting unchanged.

### Tool Payload (`tcp_load`)

Accurate payload configuration improves the arm's motion planning, collision
detection thresholds, and force/torque readings:

```elixir
controller :xarm, {BB.Ufactory.Controller,
  host: "192.168.1.111",
  model: :xarm6,
  tcp_load: {0.82, 0.0, 0.0, 48.0}
  # {mass_kg, com_x_mm, com_y_mm, com_z_mm}
  # com = center of mass relative to flange
}
```

---

## Reduced Mode and Workspace Fence

The xArm firmware supports a "reduced mode" that enforces lower speed limits
and an optional Cartesian workspace fence. This is useful for human-collaborative
applications or when the arm operates near obstacles.

### Enabling Reduced Mode

```elixir
controller :xarm, {BB.Ufactory.Controller,
  host: "192.168.1.111",
  model: :xarm6,
  # Speed limits applied in reduced mode
  reduced_tcp_speed: 250.0,      # max TCP linear speed in mm/s
  reduced_joint_speed: 1.0,      # max joint speed in rad/s
  # Enable reduced mode (applies the above limits)
  reduced_mode: true
}
```

Limits are sent before reduced mode is enabled, ensuring the firmware applies
the correct values when it enters reduced mode.

### Joint Range Limits

Optionally restrict joint travel to a narrower range in reduced mode (7 joints,
even for models with fewer — unused joints are ignored by firmware):

```elixir
reduced_joint_ranges: [
  {-3.14, 3.14},   # J1 ±180°
  {-2.059, 2.094}, # J2 standard limits
  {-3.927, 0.192}, # J3 standard limits
  {-3.14, 3.14},   # J4 ±180°
  {-1.693, 3.142}, # J5 standard limits
  {-3.14, 3.14},   # J6 ±180°
  {-3.14, 3.14}    # J7 (not used on xArm6)
]
```

### Workspace Fence (`tcp_boundary` + `fence_on`)

Reject any motion that would move the TCP outside a Cartesian box:

```elixir
controller :xarm, {BB.Ufactory.Controller,
  host: "192.168.1.111",
  model: :xarm6,
  tcp_boundary: {-400, 400, -400, 400, 0, 800},
  # {x_min, x_max, y_min, y_max, z_min, z_max} in mm
  fence_on: true
}
```

The firmware rejects the motion before it executes, which triggers error code 35.
The `Sensor.Collision` module re-publishes such events if it is declared alongside
the controller.

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
    loop_hz: 100,
    # Tool geometry — gripper G2 adds ~172mm to the flange along Z
    tcp_offset: {0.0, 0.0, 172.0, 0.0, 0.0, 0.0},
    tcp_load: {0.82, 0.0, 0.0, 48.0},
    # Workspace fence
    tcp_boundary: {-600, 600, -600, 600, 0, 900},
    fence_on: true,
    # Reduced mode for safe co-existence with humans
    reduced_tcp_speed: 250.0,
    reduced_mode: true
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
    sensor :collision, {BB.Ufactory.Sensor.Collision,
      controller: :xarm,
      sensitivity: 3,
      rebound: false,
      self_collision_check: true
    }
  end
end
```

## Next Steps

- Command the arm in Cartesian space instead of joint-space: see [Cartesian Motion](03-cartesian-motion.md)
