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
- **Force/torque sensor** — Read Fx/Fy/Fz/Tx/Ty/Tz at 50Hz
- **Linear track** — RS485-proxied linear axis position control
- **Safety integration** — Arm is stopped/disabled automatically on disarm
- **100Hz ETS control loop** — Low-latency batched joint command dispatch
- **Push-based state** — Joint angles, Cartesian pose, and torques arrive via report socket

## Supported Arms

- xArm6 (6 joints)

## Support Coming Soon

- xArm5 (5 joints)
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

Define a controller and robot topology in your BB DSL:

```elixir
defmodule MyRobot do
  use BB

  controller :xarm, {BB.Ufactory.Controller,
    host: "192.168.1.111",
    model: :xarm6,
    loop_hz: 100
  }

  topology do
    link :base do
      joint :j1, type: :revolute do
        limit lower: ~u(-360 degree), upper: ~u(360 degree), velocity: ~u(180 degree_per_second)
        actuator :motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm}

        link :upper_arm do
          joint :j2, type: :revolute do
            limit lower: ~u(-118 degree), upper: ~u(120 degree), velocity: ~u(180 degree_per_second)
            actuator :motor, {BB.Ufactory.Actuator.Joint, joint: 2, controller: :xarm}

            # ... j3–j6 ...
          end
        end
      end
    end
  end

  sensors do
    sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm}
  end
end
```

## Sending Commands

### Joint-Space Motion

```elixir
BB.Actuator.set_position(MyRobot, [:base, :j1, :motor], 0.5)
BB.Actuator.set_position!(MyRobot, :motor, 0.5)
```

### Cartesian Motion

```elixir
BB.Actuator.set_position(MyRobot, [:cartesian, :tcp], {300.0, 0.0, 400.0, 0.0, 0.0, 0.0})
```

## Components

### Controller

`BB.Ufactory.Controller` manages both TCP connections and the 100Hz control loop.

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `host` | string | required | Arm IP address |
| `port` | integer | 502 | Command socket port |
| `report_port` | integer | 30003 | Real-time report socket port |
| `model` | atom | `:xarm6` | Arm model (`:xarm5`, `:xarm6`, `:xarm7`, `:lite6`, `:xarm850`) |
| `loop_hz` | integer | 100 | Control loop frequency |
| `heartbeat_interval_ms` | integer | 1000 | Heartbeat interval |
| `disarm_action` | atom | `:stop` | Action on disarm (`:stop` or `:hold`) |

### Actuators

| Module | Description |
|--------|-------------|
| `BB.Ufactory.Actuator.Joint` | Joint-space position (radians), via ETS + 100Hz loop |
| `BB.Ufactory.Actuator.Cartesian` | Cartesian pose (mm + radians), direct command |
| `BB.Ufactory.Actuator.Gripper` | Gripper G2 position (0–840 mm) |
| `BB.Ufactory.Actuator.LinearTrack` | Linear track position (mm), RS485-proxied |

### Sensor

| Module | Description |
|--------|-------------|
| `BB.Ufactory.Sensor.ForceTorque` | 6-axis force/torque at 50Hz (Fx/Fy/Fz/Tx/Ty/Tz) |

## Protocol

UFactory uses a custom Modbus-TCP variant:

- **Header:** Transaction ID (u16 BE) + Protocol 0x0002 (u16 BE) + Length (u16 BE)
- **Body:** Register (u8) + Parameters (fp32 fields, **little-endian**)
- **Angles:** Always radians on the wire

The real-time report socket pushes frames containing joint angles, Cartesian pose, and
joint torques. Force/torque data is included in 135-byte frames.

## Documentation

Full documentation is available at [HexDocs](https://hexdocs.pm/bb_ufactory).
