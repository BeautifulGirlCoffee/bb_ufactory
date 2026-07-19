<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- changelog -->

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
