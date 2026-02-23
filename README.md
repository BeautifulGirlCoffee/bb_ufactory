<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots UFactory xArm control

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![Hex version badge](https://img.shields.io/hexpm/v/bb_ufactory.svg)](https://hex.pm/packages/bb_ufactory)

# BB.Ufactory

BB integration for UFactory xArm robotic arms.

This library provides controller, actuator, and sensor modules for integrating UFactory
xArm arms with the Beam Bots robotics framework. The arm is commanded over two TCP
connections using UFactory's custom Modbus-TCP protocol: a command socket (port 502) and
a real-time report socket (port 30003) that pushes joint state at ~100Hz.

## Features

- **Joint-space motion** — Command individual joints; batched at 100Hz via ETS
- **Cartesian-space motion** — Command end-effector pose directly; IK solved on-arm
- **Gripper support** — Gripper G2 position control (0–840 mm)
- **Force/torque sensor** — F/T data streamed from 135-byte report frames (Fx/Fy/Fz/Tx/Ty/Tz)
- **Collision detection** — Configurable sensitivity, rebound, and self-collision check; publishes collision events
- **Linear track** — RS485-proxied linear axis position control
- **Safety integration** — Arm is stopped/disabled automatically on disarm; `disarm/1` runs on a fresh TCP connection even if the GenServer has crashed
- **100Hz ETS control loop** — Low-latency batched joint command dispatch
- **Push-based state** — Joint angles, Cartesian pose, and torques arrive via the report socket
- **Hardware configuration** — TCP offset, payload, reduced mode, workspace fence configured on init
- **Automatic reconnection** — Report socket reconnects with exponential backoff on disconnect

## Supported Arms

- xArm5 (5 joints)
- xArm6 (6 joints)
- xArm7 (7 joints)
- Lite6 (6 joints)
- xArm850 (6 joints)

## Installation

Add `bb_ufactory` to your list of dependencies in `mix.exs`:

```elixir
def deps do
  [
    {:bb_ufactory, "~> 0.1.0"}
  ]
end
```

## Requirements

- UFactory xArm arm connected via Ethernet
- BB framework (`~> 0.15`)

## Usage

### Quick Start with the Pre-Built xArm6 Robot

`BB.Ufactory.Robots.XArm6` provides a ready-made robot definition with correct joint
limits, effort values, and actuator wiring. Use it as a base and override the controller
host:

```elixir
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
```

### Custom Robot Definition

For custom topologies or additional accessories, define the robot manually in your BB DSL:

```elixir
defmodule MyRobot do
  use BB

  controllers do
    controller :xarm, {BB.Ufactory.Controller,
      host: "192.168.1.111",
      model: :xarm6,
      loop_hz: 100
    }
  end

  topology do
    link :base do
      joint :j1 do
        type :revolute
        limit do
          lower ~u(-360 degree)
          upper ~u(360 degree)
          velocity ~u(180 degree_per_second)
        end
        actuator :j1_motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm}

        # ... links and joints j2–j6 ...
      end
    end
  end

  sensors do
    sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm}
    sensor :collision, {BB.Ufactory.Sensor.Collision, controller: :xarm, sensitivity: 3}
  end
end
```

## Sending Commands

### Joint-Space Motion

```elixir
BB.Actuator.set_position(MyRobot, [:base, :j1, :j1_motor], 0.5)
```

### Cartesian Motion

```elixir
BB.Actuator.set_position(MyRobot, [:cartesian, :tcp], {300.0, 0.0, 400.0, 0.0, 0.0, 0.0})
```

### Subscribing to State

```elixir
# Joint angles + torques from every report frame
BB.subscribe(MyRobot, [:sensor, :xarm])

# TCP pose from every report frame
BB.subscribe(MyRobot, [:sensor, :xarm, :tcp_pose])

# Arm state/mode/fault changes
BB.subscribe(MyRobot, [:sensor, :xarm, :arm_status])

# F/T readings (when sensor is enabled)
BB.subscribe(MyRobot, [:sensor, :wrench])

# Collision events
BB.subscribe(MyRobot, [:sensor, :collision])
```

## Components

### Controller

`BB.Ufactory.Controller` manages both TCP connections and the 100Hz control loop.

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `host` | string | required | Arm IP address or hostname |
| `port` | integer | `502` | Command socket port |
| `report_port` | integer | `30003` | Real-time report socket port |
| `model` | atom | `:xarm6` | Arm model (`:xarm5`, `:xarm6`, `:xarm7`, `:lite6`, `:xarm850`) |
| `loop_hz` | integer | `100` | Control loop frequency in Hz |
| `heartbeat_interval_ms` | integer | `1000` | Heartbeat interval in ms |
| `disarm_action` | atom | `:stop` | Action on disarm: `:stop` clears motion, `:hold` holds position |
| `tcp_offset` | tuple or `nil` | `nil` | TCP offset `{x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad}` |
| `tcp_load` | tuple or `nil` | `nil` | Tool payload `{mass_kg, com_x_mm, com_y_mm, com_z_mm}` |
| `reduced_mode` | boolean | `false` | Enable firmware reduced mode (lower speed limits + optional fence) |
| `reduced_tcp_speed` | float or `nil` | `nil` | Max TCP linear speed in mm/s (reduced mode) |
| `reduced_joint_speed` | float or `nil` | `nil` | Max joint speed in rad/s (reduced mode) |
| `reduced_joint_ranges` | list or `nil` | `nil` | Per-joint angle limits `[{lower, upper}]` × 7 (reduced mode) |
| `tcp_boundary` | tuple or `nil` | `nil` | Cartesian workspace fence `{x_min, x_max, y_min, y_max, z_min, z_max}` in mm |
| `fence_on` | boolean | `false` | Activate the TCP boundary fence |

### Actuators

| Module | Description |
|--------|-------------|
| `BB.Ufactory.Actuator.Joint` | Joint-space position (radians), via ETS + 100Hz loop |
| `BB.Ufactory.Actuator.Cartesian` | Cartesian pose (mm + radians), direct command |
| `BB.Ufactory.Actuator.Gripper` | Gripper G2 position (0–840 mm) |
| `BB.Ufactory.Actuator.LinearTrack` | Linear track position (mm), RS485-proxied |

### Sensors

| Module | Description |
|--------|-------------|
| `BB.Ufactory.Sensor.ForceTorque` | 6-axis F/T streamed from report socket (Fx/Fy/Fz/Tx/Ty/Tz) |
| `BB.Ufactory.Sensor.Collision` | Collision detection with configurable sensitivity (0–5); publishes on error codes 22, 31, 35 |

### Messages

| Message | Published to | Description |
|---------|-------------|-------------|
| `BB.Message.Sensor.JointState` | `[:sensor, controller_name]` | Joint angles + torques, every report frame |
| `BB.Ufactory.Message.CartesianPose` | `[:sensor, controller_name, :tcp_pose]` | TCP position (mm) + orientation (rad RPY), every report frame |
| `BB.Ufactory.Message.ArmStatus` | `[:sensor, controller_name, :arm_status]` | Arm state, mode, error/warn codes; published on change |
| `BB.Ufactory.Message.Wrench` | `[:sensor, controller_name, :wrench]` | F/T reading; present only when F/T sensor is enabled |

## Protocol

UFactory uses a custom Modbus-TCP variant:

- **Header:** Transaction ID (u16 BE) + Protocol 0x0002 (u16 BE) + Length (u16 BE)
- **Body:** Register (u8) + Parameters (fp32 fields, **little-endian**)
- **Angles:** Always radians on the wire
- **Exception:** Linear track position uses int32 big-endian (not fp32 LE)

The real-time report socket (port 30003) pushes frames at ~100Hz containing joint angles,
Cartesian pose, and joint torques. Force/torque data (`ft_filtered`) is present only in
frames that are 135+ bytes, which requires the F/T sensor to be enabled via
`cmd_ft_sensor_enable`.

Error codes are not included in real-time report frames; they are polled from the command
socket (register 0x0F) once per second alongside the heartbeat.

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_ufactory).
