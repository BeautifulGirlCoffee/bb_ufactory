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

**Position units:** pulse units in the range **0–850**. The relationship to
physical jaw opening depends on the gripper model, but the full range spans from
fully closed (0) to fully open (850).

### Adding the Gripper

Enable the gripper with the `:gripper` option — it mounts a `:gripper`
actuator on a fixed joint at `:link6` (the TCP):

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6,
    host: "192.168.1.111",
    gripper: [speed: 1500]   # pulse units per second; `gripper: true` for defaults
end
```

On `init`, the gripper actuator automatically sends `cmd_gripper_enable(true)` to
the controller so the gripper is energised and ready before any position commands
arrive.

### Commanding the Gripper

Command the target position in pulse units — `BB.Actuator.set_position/4`
is synchronous, so a refusal (robot disarmed, controller unreachable)
comes back as `{:error, reason}` instead of vanishing:

```elixir
# Open gripper (850 = fully open)
:ok = BB.Actuator.set_position(MyRobot, :gripper, 850.0)

# Close gripper (0 = fully closed)
:ok = BB.Actuator.set_position(MyRobot, :gripper, 0.0)
```

Positions outside 0–850 are automatically clamped by the actuator.

### Disarm Behaviour

When the robot is disarmed, `Actuator.Gripper` sends
`cmd_gripper_enable(false)` through the controller, best-effort: if the
controller is down the attempt is swallowed. (The controller's own `disarm/1`
is the layer that opens a fresh TCP connection to stop the *arm* even after a
crash — the gripper has no independent connection of its own.)

---

## Force/Torque Sensor

The UFactory F/T sensor attaches to the tool flange and reports six-axis wrench
data: forces Fx/Fy/Fz (Newtons) and torques Tx/Ty/Tz (Newton-metres).

Wrench data is **push-based**: once the sensor is enabled (register 0xC9),
the arm includes `ft_filtered` values in its 135+ byte real-time report
frames, and the controller publishes a `BB.Ufactory.Message.Wrench` for each
one — no polling is involved.

### Adding the Sensor

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6, host: "192.168.1.111"

  sensors do
    sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm}
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

On disarm, the sensor sends `cmd_ft_sensor_enable(false)` through the
controller, best-effort: if the controller is already down, the attempt is
silently ignored.

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

Enable it with the `:linear_track` option — it mounts a `:track` actuator on
a fixed joint at the base link:

```elixir
defmodule MyRobot do
  use BB.Ufactory.Robots.XArm6,
    host: "192.168.1.111",
    linear_track: [speed: 200]   # mm/s; `linear_track: true` for defaults
end
```

### Commanding the Linear Track

Position is given in millimetres:

```elixir
# Move track to 500 mm from the home position (synchronous)
:ok = BB.Actuator.set_position(MyRobot, :track, 500.0)
```

`BB.Actuator.stop(MyRobot, :track)` brakes the carriage by re-targeting its
current position (read over RS485); the stop is refused with
`{:error, :position_unknown}` if the read fails.

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
use BB.Ufactory.Robots.XArm6,
  host: "192.168.1.111",
  controller: [
    tcp_offset: {0.0, 0.0, 172.0, 0.0, 0.0, 0.0}
    # {x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad}
  ]
```

Omit `tcp_offset` (or pass `nil`) to leave the arm's current setting unchanged.

### Tool Payload (`tcp_load`)

Accurate payload configuration improves the arm's motion planning, collision
detection thresholds, and force/torque readings:

```elixir
use BB.Ufactory.Robots.XArm6,
  host: "192.168.1.111",
  controller: [
    tcp_load: {0.82, 0.0, 0.0, 48.0}
    # {mass_kg, com_x_mm, com_y_mm, com_z_mm}
    # com = center of mass relative to flange
  ]
```

---

## Reduced Mode and Workspace Fence

The xArm firmware supports a "reduced mode" that enforces lower speed limits
and an optional Cartesian workspace fence. This is useful for human-collaborative
applications or when the arm operates near obstacles.

### Enabling Reduced Mode

```elixir
use BB.Ufactory.Robots.XArm6,
  host: "192.168.1.111",
  controller: [
    # Speed limits applied in reduced mode
    reduced_tcp_speed: 250.0,      # max TCP linear speed in mm/s
    reduced_joint_speed: 1.0,      # max joint speed in rad/s
    # Enable reduced mode (applies the above limits)
    reduced_mode: true
  ]
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
use BB.Ufactory.Robots.XArm6,
  host: "192.168.1.111",
  controller: [
    tcp_boundary: {-400, 400, -400, 400, 0, 800},
    # {x_min, x_max, y_min, y_max, z_min, z_max} in mm
    fence_on: true
  ]
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
  use BB.Ufactory.Robots.XArm6,
    host: "192.168.1.111",
    gripper: [speed: 1500],
    linear_track: [speed: 200],
    controller: [
      # Tool geometry — gripper G2 adds ~172mm to the flange along Z
      tcp_offset: {0.0, 0.0, 172.0, 0.0, 0.0, 0.0},
      tcp_load: {0.82, 0.0, 0.0, 48.0},
      # Workspace fence
      tcp_boundary: {-600, 600, -600, 600, 0, 900},
      fence_on: true,
      # Reduced mode for safe co-existence with humans
      reduced_tcp_speed: 250.0,
      reduced_mode: true
    ]

  sensors do
    sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm}

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
