<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# AGENTS.md

This file provides guidance to AI coding agents when working with code in this repository.

## Project Overview

`BB.Ufactory` is a Beam Bots integration library for UFactory xArm robotic arms
(xArm5, xArm6, xArm7, Lite6, xArm850). It provides controller, actuator, and sensor
modules that plug into the BB robotics framework's DSL.

Communication uses UFactory's custom Modbus-TCP variant over two TCP connections:
- **Port 502** — command socket (send register commands, receive responses)
- **Port 30003** — real-time report socket (arm pushes joint state at ~100Hz)

The protocol uses big-endian headers with little-endian float payloads. The arm solves
IK internally, so both joint-space and Cartesian-space commands are supported natively.

## Build and Test Commands

```bash
mix check --no-retry    # Run all checks (compile, test, format, credo, dialyzer, reuse)
mix test                # Run tests
mix test path/to/test.exs:42  # Run single test at line
mix format              # Format code
mix credo --strict      # Linting
```

The project uses `ex_check` — always prefer `mix check --no-retry` over running individual tools.

## Architecture

### Component Hierarchy

```
Controller (GenServer)
    |
    +-- command socket (port 502) — sends frames, heartbeat
    +-- report socket (port 30003) — receives pushed state at 100Hz
    +-- ETS table — shared state between controller and actuators
    |
    v
Actuator.Joint     — writes set_position to ETS (joint-space)
Actuator.Cartesian — sends cmd_move_cartesian via controller call
Actuator.Gripper   — sends gripper commands via controller call
Actuator.LinearTrack — RS485-proxied track position via controller call
    |
    v publishes
BB.Message.Actuator.BeginMotion
BB.Message.Sensor.JointState  (published by controller from report data)

Sensor.ForceTorque — polls register 0xC8, publishes BB.Ufactory.Message.Wrench
```

### Key Modules (planned)

- **`BB.Ufactory.Protocol`** (`lib/bb/ufactory/protocol.ex`) — Frame builder/parser for
  port 502 commands. Handles the Modbus-TCP variant header (transaction ID u16 BE,
  protocol 0x0002 u16 BE, length u16 BE, register u8, params as little-endian fp32s).

- **`BB.Ufactory.Report`** (`lib/bb/ufactory/report.ex`) — Parser for auto-push report
  frames from port 30003. Each frame: 4-byte length, state/mode byte, cmd count u16,
  7× joint angles fp32 LE, 6× Cartesian pose fp32 LE, 7× torques fp32 LE, optional
  force-torque data.

- **`BB.Ufactory.Registers`** (`lib/bb/ufactory/registers.ex`) — Module-attribute
  constants for all register addresses (0x0B enable, 0x0C set_state, 0x15 move_cart,
  0x17 move_joints, 0x2A get_joints, 0x7C gripper, 0xC8 ft_get, etc.).

- **`BB.Ufactory.Model`** (`lib/bb/ufactory/model.ex`) — Per-model joint counts and
  limits for xArm5/6/7, Lite6, xArm850.

- **`BB.Ufactory.Controller`** (`lib/bb/ufactory/controller.ex`) — `BB.Controller`
  GenServer. Manages both TCP sockets, ETS table, 100Hz control loop, heartbeat (every
  1s), and state machine pubsub. Publishes `JointState` from report data.

- **`BB.Ufactory.Actuator.Joint`** — Writes `set_position` to ETS; controller loop
  batches all joints into a single `cmd_move_joints` frame at 100Hz.

- **`BB.Ufactory.Actuator.Cartesian`** — Sends `cmd_move_cartesian` directly via
  controller `handle_call({:send_command, frame}, ...)`.

- **`BB.Ufactory.Actuator.Gripper`** — Gripper G2 position (0–840 mm) via register 0x7C.

- **`BB.Ufactory.Actuator.LinearTrack`** — Linear track (RS485 proxied; int32 BE / 2000 = mm).

- **`BB.Ufactory.Sensor.ForceTorque`** — Polls register 0xC8 at 50Hz, publishes
  `BB.Ufactory.Message.Wrench`.

### BB Framework Integration

The library uses BB's:
- `BB.Controller` behaviour for controller lifecycle
- `BB.Actuator` behaviour for actuator lifecycle
- `BB.Sensor` behaviour for sensor lifecycle
- `BB.Message` for typed message payloads
- `BB.Safety` for arm/disarm handling
- `BB.publish`/`BB.subscribe` for hierarchical PubSub by path
- `BB.Process.call`/`BB.Process.cast` to communicate with sibling processes
- `Spark.Options` for configuration validation

### Protocol Notes

- All wire angles are **radians**. No degree conversion needed.
- `fp32` fields are **little-endian**; u16 header fields are **big-endian**.
- Linear track position is `int32` **big-endian** (exception to the fp32 LE rule).
- Heartbeat frame: `<<0, 0, 0, 1, 0, 2, 0, 0>>` — send every 1s on command socket.
- Register 0x15 = `MOVE_LINE` (Cartesian linear). Register 0x17 = `MOVE_JOINT`.
  Do not confuse with 0x18 (`MOVE_LINE_AA`, arc blending).

### Testing

Tests use Mimic to mock `BB`, `BB.Process`, `BB.Robot`, and `BB.Safety`. Hardware tests
are tagged `@tag :hardware` and excluded from `mix test` by default. Test support modules
live in `test/support/`.

## Dependencies

- `bb ~> 0.13` — The Beam Bots robotics framework

## Reference Material

| Resource | Location |
|----------|----------|
| xArm Developer Manual V1.10.0 | `ref_repos/xArm-Python-SDK/doc/UF_ModbusTCP_Manual.md` |
| xArm Python SDK | `ref_repos/xArm-Python-SDK/` |
| bb_servo_feetech (structural reference) | `tmp/bb_servo_feetech/` |
| Implementation plan | `tmp/bb_ufactory_plan.md` |
