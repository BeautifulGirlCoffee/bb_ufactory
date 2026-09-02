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
  speed/acceleration; integer fields accepted), delivered through the gated
  pipeline. The raw `{:move_cartesian, pose}` cast is deprecated: it logs a
  warning and is **dropped when the robot is not armed** (it previously
  moved a disarmed robot — the firmware stays motion-ready under
  `disarm_action: :hold`).
- `use BB.Ufactory.Robots.XArm6` takes options (`:host`, `:loop_hz`,
  `:simulation`, `:controller`, `:cartesian`, `:gripper`, `:linear_track`)
  instead of the documented-but-never-compiling `controllers`-override
  pattern; a `:cartesian` actuator is included by default. Accessory
  actuators mount on fixed joints (bb's DSL allows actuators only under
  joints — the old link-level examples never compiled either).
- The controller's shared ETS joint rows are now 5-tuples
  (`{joint, current_position, current_torque, set_position, set_velocity}`).
- `{:send_and_recv, frame}` defaults to a 500 ms timeout (was 5 s — it
  blocks the controller GenServer mid-stream) and can return
  `{:error, :desync}` on an unrecoverable stream.
- `Sensor.ForceTorque`'s inert `poll_interval_ms` option is removed (wrench
  data is push-based from report frames; the option was ignored).
- `Protocol.cmd_move_joints/4` raises for more than 7 angles instead of
  silently dropping extras; `cmd_set_mode/2` rejects modes outside `0..7`.
- `Actuator.Joint` refuses a topology transmission at init
  (`{:stop, {:unsupported_transmission, name}}`): bb hands the driver
  motor-space values but the firmware receives joint angles, so a
  non-identity transmission would silently skew every command.

### Bug Fixes

- **`Protocol.cmd_stop/1` now sends SET_STATE 4 (stop) instead of
  SET_STATE 0 (motion state).** State 0 is the firmware's motion-ready
  ("sport") state, so the `:stop` disarm action — including the
  fresh-connection safety disarm — was commanding the arm *into* motion
  state rather than terminating motion and clearing queued commands.
  Firmware-tier simulator tests now pin the semantics (a byte-level test
  cannot), and `Registers.set_state/0`'s doc table is corrected too.
- **Fixed an ETS lost-update race** between report ingestion and joint
  commands: both sides did read-whole-row-then-insert on the same public
  rows, so a report frame interleaving with a command could silently drop a
  fresh target — or resurrect the pre-Stop target after a brake latch. Each
  side now writes only its own columns via `:ets.update_element/3`.
- **`send_and_recv` matches replies by register**: while armed, 100 Hz
  move-acks are in flight that the pre-send drain cannot clear, and the
  first-frame-wins behavior could hand an RS485 caller a move-ack (refusing
  a healthy linear-track Stop as `:position_unknown`). Foreign frames are
  skipped and split frames reassembled until the deadline.
- **The 100 Hz loop no longer streams motion on stale feedback**: with the
  report socket down it kept commanding joints to frozen last-known angles
  indefinitely. Ticks are skipped when the last report frame is older than
  `report_stale_ms` (new option, default 250 ms), and after
  `feedback_loss_fatal_ms` (default 3 s) of blindness with motion pending
  the controller stops and reports to `BB.Safety`.
- Sensors no longer crash the whole robot supervision tree when the
  controller is unreachable at init (mock simulation, or a crash-restart
  race on hardware) — `ForceTorque` and `Collision` degrade to a warning.
- Gripper clamp raised to the SDK's true fully-open value (850 pulses, was
  840 — fully-open commands stopped ~0.9 mm short).
- `parse_linear_track_position/1` requires Modbus function 0x03, so RS485
  exception replies no longer decode as garbage positions; u16 speed fields
  clamp at 0xFFFF instead of bit-wrapping to a crawl.
- `{:register_arm_frames, ...}` replies `{:error, reason}` (not `:ok`) when
  the armed-time send fails; the controller ETS handle is always the table
  name (a cached tid raised after a controller crash-restart);
  `Simulator.command/3` reassembles split responses instead of leaking
  `{:more}` through `reachable?/3`.
- `mix bb_ufactory.sim start <unknown-model>` prints the usage error again
  instead of raising `ArgumentError`.

### Features

- `Actuator.Joint` accepts `Command.Stop` and `Command.Hold`, braking the
  joint by latching its current reported position as the 100 Hz loop
  target. It also declares `capabilities: [:position_feedback,
  :effort_feedback]`, so joints it drives no longer warn about missing
  position sensors (the controller publishes `JointState` from every
  report frame).
- `Command.Position` **velocity hints are honored**: the loop dispatches
  each batch at the slowest pending hint (clamped to the model maximum),
  with acceleration scaling to match; brake latches clear the hint so
  stopping always happens at full speed.
- `Actuator.LinearTrack` accepts `Command.Stop`: it reads the carriage's
  current position over RS485 (one round-trip — the read doubles as the
  BeginMotion estimate) and re-targets it, refusing the stop with a logged
  warning when the read fails rather than commanding an assumed position.
  Stop/Hold refusals are logged everywhere, since fire-and-forget
  transports discard the error reply.
- The 100 Hz control loop runs on `BB.Loop` (bb 0.26+): ticks are
  scheduled against absolute monotonic deadlines, so the loop no longer
  drifts by per-tick processing time, and overruns are reported via
  `[:bb, :loop, :tick]` telemetry instead of accumulating silently.
- The out-of-band safety disarm now waits briefly for the firmware's
  response and logs when the stop went unconfirmed.

### Improvements

- `.formatter.exs` imports the Spark DSL locals from bb's exported
  formatter config instead of a hand-maintained copy.
- Added `reach` to the dev/test toolchain: `.reach.exs` machine-checks the
  wire-layer purity invariant (Protocol/Report/Registers/Model must not
  depend on runtime components, message structs, or sockets), and
  `mix check` now runs `mix reach.check --arch --smells --strict`.
- Test suite hardening: byte-exact RS485 frame pins (host/device/function/
  register bytes — self-referential assertions could not catch a swapped
  bus), `stream_data` properties for fp32 round-trips and frame parsing
  across arbitrary TCP split points, firmware-tier stop-semantics tests,
  and an interleaving stress test for the ETS race. CI compiles test files
  with `--warnings-as-errors` (a tag-excluded simulator test calling a
  removed API previously surfaced only as a warning) and refuses to publish
  a version without a CHANGELOG entry.
- CI runs on Elixir 1.20.4 (was 1.20.2); the library now requires
  Elixir ~> 1.20.
- Docs corrected end to end: quick-start and accessory examples compile
  (verified by test), disarm behaviour and F/T data flow described
  truthfully, gripper units fixed, tutorials standardized on the
  synchronous `BB.Actuator` API.

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
