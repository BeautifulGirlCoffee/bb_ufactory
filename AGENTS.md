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
    +-- command socket (port 502) — sends frames, heartbeat, polls error code (0x0F)
    +-- report socket (port 30003) — active: true, receives pushed state at ~100Hz
    +-- ETS table — shared state between controller and actuators
    |
    v
Actuator.Joint       — writes set_position to ETS (joint-space)
Actuator.Cartesian   — sends cmd_move_cartesian via controller call
Actuator.Gripper     — sends gripper commands via controller call
Actuator.LinearTrack — RS485-proxied track position via controller call
    |
    v publishes (from report frames)
BB.Message.Sensor.JointState         → [:sensor, controller_name]
BB.Ufactory.Message.CartesianPose    → [:sensor, controller_name, :tcp_pose]
BB.Ufactory.Message.ArmStatus        → [:sensor, controller_name, :arm_status] (on change)
BB.Ufactory.Message.Wrench           → [:sensor, controller_name, :wrench] (135+ byte frames)

Sensor.ForceTorque — enables F/T on init, forwards Wrench from controller pubsub
Sensor.Collision   — configures collision detection on init, forwards ArmStatus on fault

BB.Error.Protocol.Ufactory.HardwareFault  — raised via BB.Safety.report_error
BB.Error.Protocol.Ufactory.CommandRejected
BB.Error.Protocol.Ufactory.ConnectionError
```

### Key Modules

- **`BB.Ufactory.Protocol`** (`lib/bb/ufactory/protocol.ex`) — Frame builder/parser for
  port 502 commands. Handles the Modbus-TCP variant header (transaction ID u16 BE,
  protocol 0x0002 u16 BE, length u16 BE, register u8, params as little-endian fp32s).
  Includes builders for: `cmd_enable`, `cmd_set_state`, `cmd_stop`, `cmd_move_joints`,
  `cmd_move_cartesian`, `cmd_gripper_move`, `cmd_linear_track_set_pos`,
  `cmd_ft_sensor_enable`, `cmd_set_tcp_offset`, `cmd_set_tcp_load`,
  `cmd_set_reduced_mode`, `cmd_set_tcp_boundary`, `cmd_set_fence_on`,
  `cmd_set_collision_sensitivity`, `cmd_set_collision_rebound`,
  `cmd_set_self_collision_check`, `cmd_get_error`, and `heartbeat`.

- **`BB.Ufactory.Report`** (`lib/bb/ufactory/report.ex`) — Parser for auto-push report
  frames from port 30003. Each frame: 4-byte length, state/mode byte, cmd count u16,
  7× joint angles fp32 LE, 6× Cartesian pose fp32 LE, 7× torques fp32 LE, optional
  force-torque data (in 135+ byte frames).

- **`BB.Ufactory.Registers`** (`lib/bb/ufactory/registers.ex`) — Module-attribute
  constants for all register addresses (0x0B enable, 0x0C set_state, 0x15 move_cart,
  0x17 move_joints, 0x2A get_joints, 0x7C gripper, 0xC8 ft_get, etc.).

- **`BB.Ufactory.Model`** (`lib/bb/ufactory/model.ex`) — Per-model joint counts,
  max speed, and per-joint radian limits for all five supported variants:
  xArm5, xArm6, xArm7, Lite6, xArm850.

- **`BB.Ufactory.Controller`** (`lib/bb/ufactory/controller.ex`) — `BB.Controller`
  GenServer. Manages both TCP sockets, ETS table, 100Hz control loop, 1s heartbeat,
  error code polling, report socket reconnection with exponential backoff, and state
  machine pubsub. Publishes `JointState`, `CartesianPose`, `ArmStatus`, and `Wrench`
  from report data. Applies hardware configuration on init (TCP offset, payload,
  reduced mode, workspace fence, collision settings).

- **`BB.Ufactory.Actuator.Joint`** — Writes `set_position` to ETS; controller loop
  batches all joints into a single `cmd_move_joints` frame at 100Hz.

- **`BB.Ufactory.Actuator.Cartesian`** — Sends `cmd_move_cartesian` directly via
  controller `handle_call({:send_command, frame}, ...)`.

- **`BB.Ufactory.Actuator.Gripper`** — Gripper G2 position (0–840 mm) via register 0x7C.

- **`BB.Ufactory.Actuator.LinearTrack`** — Linear track (RS485 proxied; int32 BE / 2000 = mm).

- **`BB.Ufactory.Sensor.ForceTorque`** — Enables the arm's built-in F/T sensor on
  init via `cmd_ft_sensor_enable`. Subscribes to `[:sensor, controller_name, :wrench]`
  and re-publishes to its own sensor-scoped pubsub path. Disarms by sending
  `cmd_ft_sensor_enable(false)`.

- **`BB.Ufactory.Sensor.Collision`** — Configures collision detection sensitivity,
  rebound behaviour, and self-collision check on init. Subscribes to
  `[:sensor, controller_name, :arm_status]` and re-publishes events where
  `error_code` is 22 (self-collision), 31 (external contact), or 35 (fence violation)
  to its own sensor-scoped pubsub path.

- **`BB.Ufactory.Message.ArmStatus`** — Arm state/mode/error_code/warn_code snapshot,
  published on change.

- **`BB.Ufactory.Message.CartesianPose`** — TCP position (mm) and orientation (radians
  RPY), published on every report frame.

- **`BB.Ufactory.Message.Wrench`** — 6-axis F/T reading (Fx/Fy/Fz/Tx/Ty/Tz), present
  only in 135+ byte report frames when F/T sensor is enabled.

- **`BB.Ufactory.Robots.XArm6`** (`lib/bb/ufactory/robots/x_arm6.ex`) — Ready-to-use
  BB robot definition for the xArm6 with correct joint limits, effort values, and
  actuator wiring. Use with `use BB.Ufactory.Robots.XArm6` as a starting point.

- **`BB.Error.Protocol.Ufactory.HardwareFault`** — Structured exception for arm
  hardware error codes (full code table included). Raised via `BB.Safety.report_error`.

- **`BB.Error.Protocol.Ufactory.CommandRejected`** — Raised when a response frame
  carries a non-zero status byte.

- **`BB.Error.Protocol.Ufactory.ConnectionError`** — Raised on TCP connect failure
  during `init/1`.

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

Test files mirror the `lib/` structure:
- `test/bb/ufactory/controller_test.exs` — controller lifecycle, ETS, pubsub, error handling
- `test/bb/ufactory/protocol_test.exs` — frame encoding/parsing
- `test/bb/ufactory/report_test.exs` — report frame parsing
- `test/bb/ufactory/model_test.exs` — model config
- `test/bb/ufactory/error_test.exs` — error structs
- `test/bb/ufactory/actuator/` — one file per actuator
- `test/bb/ufactory/sensor/` — one file per sensor
- `test/bb/ufactory/robots/x_arm6_test.exs` — robot definition
- `test/bb/ufactory/hardware_test.exs` — `@tag :hardware` integration tests
- `test/bb/ufactory/sim_test.exs` — simulation mode tests

## Dependencies

- `bb ~> 0.15` — The Beam Bots robotics framework

## Reference Material

| Resource | Location |
|----------|----------|
| xArm Developer Manual V1.10.0 | `ref_repos/xArm-Python-SDK/doc/UF_ModbusTCP_Manual.md` |
| xArm Python SDK | `ref_repos/xArm-Python-SDK/` |
| bb_servo_feetech (structural reference) | `tmp/bb_servo_feetech/` |
| Implementation plan | `tmp/bb_ufactory_plan.md` |
