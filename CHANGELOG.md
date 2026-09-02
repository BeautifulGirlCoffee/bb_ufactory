<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- changelog -->

## v0.2.0 (2026-09-01)

### Breaking Changes

- Requires `bb ~> 0.31` (was `~> 0.22`). Actuator command delivery now goes
  through `c:BB.Actuator.handle_command/2` (bb 0.23+): the framework
  subscribes each actuator to its command topic, enforces the armed check,
  and gates payload types before the driver sees them. Code that called the
  actuators' old `handle_cast({:command, ...})`/`handle_info` clauses
  directly must use the command pipeline (`BB.Actuator.set_position/4`,
  `BB.call(robot, name, {:command, msg})`).
- `BB.Actuator.set_position/4` is synchronous under bb 0.30+: it returns
  `:ok` or `{:error, reason}`, and the Gripper, LinearTrack, and Cartesian
  actuators now reply with the error when the controller cannot deliver a
  frame.
- Cartesian moves have a first-class payload:
  `BB.Ufactory.Message.Command.CartesianMove` (6-DOF pose + optional
  speed/acceleration), delivered through the gated pipeline. The raw
  `{:move_cartesian, pose}` cast is still accepted but bypasses the armed
  check; prefer the payload.

### Bug Fixes

- **`Protocol.cmd_stop/1` now sends SET_STATE 4 (stop) instead of
  SET_STATE 0 (motion state).** State 0 is the firmware's motion-ready
  ("sport") state, so the `:stop` disarm action — including the
  fresh-connection safety disarm — was commanding the arm *into* motion
  state rather than terminating motion and clearing queued commands.

### Features

- `Actuator.Joint` accepts `Command.Stop` and `Command.Hold`, braking the
  joint by latching its current reported position as the 100 Hz loop
  target. It also declares `capabilities: [:position_feedback,
  :effort_feedback]`, so joints it drives no longer warn about missing
  position sensors (the controller publishes `JointState` from every
  report frame).
- `Actuator.LinearTrack` accepts `Command.Stop`: it reads the carriage's
  current position over RS485 and re-targets it, and refuses the stop when
  the read fails rather than commanding an assumed position.
- The 100 Hz control loop runs on `BB.Loop` (bb 0.26+): ticks are
  scheduled against absolute monotonic deadlines, so the loop no longer
  drifts by per-tick processing time, and overruns are reported via
  `[:bb, :loop, :tick]` telemetry instead of accumulating silently.

### Improvements

- `.formatter.exs` imports the Spark DSL locals from bb's exported
  formatter config instead of a hand-maintained copy.
- Added `reach` to the dev/test toolchain: `.reach.exs` machine-checks the
  wire-layer purity invariant (Protocol/Report/Registers/Model must not
  depend on runtime components, message structs, or sockets), and
  `mix check` now runs `mix reach.check --arch --smells --strict`.
- CI runs on Elixir 1.20.4 (was 1.20.2); the library now requires
  Elixir ~> 1.20.

## v0.1.0 (2026-07-19)

Initial release.

### Features

- `BB.Ufactory.Controller` — manages the command (502) and real-time report
  (30003) sockets, the 100 Hz batched joint-motion loop, heartbeat and
  error-code polling, the arm/disarm safety lifecycle, and hardware
  configuration (TCP offset/payload, reduced mode, workspace fence).
- Actuators: per-joint position (`Actuator.Joint`), Cartesian moves
  (`Actuator.Cartesian`), Gripper G2 (`Actuator.Gripper`), and linear track
  (`Actuator.LinearTrack`, with stroke clamping).
- Sensors: force-torque wrench forwarding (`Sensor.ForceTorque`) and
  firmware collision events (`Sensor.Collision`).
- `BB.Ufactory.Protocol` — pure encode/decode for the full wire protocol,
  including firmware kinematics and workspace queries (`cmd_get_fk`,
  `cmd_get_ik`, `cmd_tcp_limit_check`, `cmd_joint_limit_check`).
- Per-model configuration for xArm5/6/7, Lite6, and UF850 with joint-limit
  tables verified against the official URDFs **and** against UFACTORY's
  firmware limit enforcement in simulation.
- Simulator testing support for library consumers: `mix bb_ufactory.sim`
  (container lifecycle), `BB.Ufactory.SimulatorCase` (ExUnit case
  template), `BB.Ufactory.Simulator` (protocol-level helpers that also work
  against physical arms, including the `reachable?/3` workspace probe), and
  the *Testing against the UFACTORY simulator* tutorial.
- Safety hardening throughout: command-socket failures stop the controller
  and report to `BB.Safety`, malformed report streams force a reconnect,
  motion ticks never substitute defaults for unknown joint positions, and
  accessory enable frames are sequenced after the controller's arm
  sequence.

CI validates every commit against UFACTORY's firmware simulator for all
five supported arm models.
