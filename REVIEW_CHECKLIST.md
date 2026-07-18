<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Code Review Checklist — 2026-07-18

Findings from the dependency-update + full-codebase review pass (bb 0.15.1 → 0.22.1).
Temporary tracking file — delete once everything is resolved or ticketed.

Status legend: `[ ]` open · `[x]` fixed · `[~]` needs hardware verification / deferred (see note)

## Before the next release — flag for hardware testing / changelog

- **Heartbeat frame changed** (M1): now a well-formed GET_STATE request instead of the old 8-byte
  constant. Verify keep-alive behaviour against the physical arm.
- **Joint limits corrected for xarm7 / lite6 / xarm850** (C1): if either of those models is in use
  anywhere, previously-accepted out-of-range commands will now clamp differently.
- **API renames**: `Registers.move_cart_arc/0` → `Registers.move_line_aa/0`;
  `Report.parse_report/1` → `parse_report/2` (arity-1 still works, defaults to `:realtime`).
- **New controller behaviour**: command-socket failures now stop the controller (after
  `BB.Safety.report_error/3`) instead of logging and continuing; malformed report frames force a
  report-socket reconnect. Accessories (gripper/track/F-T) enable via
  `{:register_arm_frames, label, frames}` on the controller rather than their own
  `[:state_machine]` subscriptions.
- **New option**: `stroke_mm` on `BB.Ufactory.Actuator.LinearTrack` (default 700).

## Already fixed during the dependency update

- [x] `BB.Message` struct: `timestamp` → `monotonic_time`/`wall_time` (bb 0.16 breaking change) in
      `collision_test.exs` and `force_torque_test.exs`
- [x] Unpinned variables in `binary-size(...)` matches (`protocol.ex:148`, `report.ex:115`) — future compile errors
- [x] Removed 7 unused `ex_cldr_*` entries from `mix.lock` (bb moved to gettext)
- [x] Stale `auto_disarm_on_error` reference in `hardware_fault.ex` moduledoc (option removed in bb 0.16)

## Critical (safety-relevant)

- [x] **C1** `model.ex:57-93` — joint limits copy-pasted across models. xArm7 J4 range inverted
      (shipped `{-3.927, 0.192}`, real `(-0.192, 3.927)`); lite6 J2/J3/J5 and xarm850 (UF850) J2/J3/J5
      wrong — out-of-limit commands pass the clamp, valid ranges blocked. Verify against
      xArm-Python-SDK / xarm_ros2 URDFs before changing. Moduledoc also falsely claims limits can be
      overridden in robot config.
- [x] **C2** `actuator/{joint,gripper,linear_track,cartesian}.ex` — actuators implement
      `handle_info({:bb, [:actuator | path], ...})` but never `BB.subscribe` to that topic, so
      pubsub-delivered commands (incl. `BB.Motion.move_to/4` default `delivery: :pubsub`) go nowhere.
- [x] **C3** `controller.ex:362-377,826-838` — command-socket failure (`{:error, :closed}` from
      send/recv) is logged and discarded: no reconnect, no crash, no supervision escalation. Error
      polling (the only fault-detection path) silently dies while armed. Should stop the GenServer so
      bb's safety controller force-disarms.
- [x] **C4** `controller.ex:768-797` — post-arm grace period records suppressed code in
      `last_error_code`; after grace expiry the `!=` guard means a *persistent* fault is never
      reported. Re-evaluate suppressed faults at grace expiry.
- [x] **C5** `controller.ex:583-585` — `set_pos || cur_pos || 0.0`: motion tick can command joints
      with unknown current position to 0.0 (uncommanded full-arm sweep after controller restart
      before first report frame). Skip tick while any position is unknown.

## High

- [x] **H1** `report.ex:105-125` — `parse_report/1` crashes (MatchError) on `frame_length < 87` and
      buffers forever on absurd lengths (e.g. `0xFFFFFFFF`) with no resync. Add bounds → error return;
      controller must handle it (reconnect/resync). Existing test at `controller_test.exs:706-711`
      asserts the wedge behavior as passing — update it.
- [x] **H2** `protocol.ex:144-152` — `parse_response/1` crashes on wire `length < 2`. Add guard →
      `{:error, ...}`.
- [x] **H3** `sensor/force_torque.ex` — `disarm/1` disables the F/T sensor but nothing re-enables it
      on re-arm (no `[:state_machine]` subscription, unlike Gripper/LinearTrack). Sensor is
      permanently dead after first disarm→re-arm cycle.
- [x] **H4** `report.ex:151-188` — frame-size heuristic mis-parses normal-report frames > 134 bytes
      as force-torque data (real port-30001 normal reports are 187–245+ bytes); `error_code` decoded
      as garbage floats. Parser should take report type as an argument, not guess from size. Moduledoc
      contradicts code three ways (`>= 111`, `>= 133` claims).

## Medium

- [~] **M1** `protocol.ex` — heartbeat constant encoded protocol id 0x0001 but the parser requires
      0x0002, so every reply parsed as `{:error, {:bad_protocol_id, 1}}`. **Fixed in code**:
      `heartbeat/0` is now a well-formed `GET_STATE` (register 0x0D) request, and the error-poll
      scanner skips its reply. **Still needs a session against the physical xArm** to confirm the
      arm accepts it as keep-alive traffic (the old 8-byte constant was presumably hand-derived
      during hardware debugging).
- [x] **M2** `controller.ex:753-766` — error poll requires the *first* parsed frame to be register
      0x0F; any stale in-flight response at the head silently skips that fault-detection cycle. Walk
      frames until 0x0F or timeout.
- [x] **M3** `actuator/linear_track.ex:127-143` — no stroke clamping; any mm value (e.g. 5000) is
      encoded and sent to hardware. Add stroke option + clamp like Joint/Gripper.
- [x] **M4** `actuator/cartesian.ex:95-123` — `expected_arrival` computed from configured default
      speed even when the per-command override speed is used for the actual move (up to 4× wrong).
- [x] **M5** `actuator/cartesian.ex:126-131` — BeginMotion publishes scalar mm distance where bb's
      contract expects radians/metres; estimators get garbage. Convert to metres and document the
      distance semantic.
- [x] **M6** `controller.ex:242-244` — `init/1` connects with `timeout: :infinity` (OS default ~75s
      stall of the supervision tree on a black-holed host); reconnect path uses 2000 ms. Use the same
      timeout in init.
- [x] **M7** `protocol.ex:536-537` — `parse_linear_track_position/1` doctest wrong by 1000×
      (`2_000_000 / 2000 = 1000.0`, not `1.0`), and **no test file runs doctests at all**. Fix the
      example and add `doctest` calls for Protocol and Report.
- [x] **M8** `actuator/joint.ex:68,114` — `joint:` index beyond the model's joint count passes init
      (`Enum.at` → nil limits), then crash-loops with MatchError on the first command. Validate at
      init and `{:stop, ...}`.

## Low

- [x] **L1** `controller.ex:347` — `:math.pow(2, attempts)` overflows (ArithmeticError) at
      attempts ≥ 1024 (~8.5 h of failed reconnects). Cap the exponent, not just the product.
- [x] **L2** `controller.ex` — no catch-all `handle_info` (defining any clause removes bb's injected
      default); unexpected message shape kills an armed controller.
- [x] **L3** `controller.ex:319-332` — `:tcp_error` followed by `:tcp_closed` (legal inet sequence)
      starts two parallel reconnect timer chains.
- [x] **L4** `registers.ex:54-56` — `move_cart_arc/0` mislabels register 0x5C (it is MOVE_LINE_AA,
      axis-angle linear move; arc blends are `move_lineb`/`move_jointb`). Rename + fix doc.
- [x] **L5** `sensor/collision.ex:69-90` — config options typed `:any`; invalid values crash init
      with FunctionClauseError instead of a schema error. Use `{:in, 0..5}` / `:boolean`; make the
      `sensitivity: 0` nil-check explicit.
- [x] **L6** `actuator/{gripper,linear_track}.ex` — enable-on-`:armed` races the controller's own
      arm-sequence handling of the same `[:state_machine]` publish (unspecified dispatch order); an
      RS485 enable can reach the controller before mode/state setup.
- [x] **L7** `message/arm_status.ex:15-22` — documented state table wrong vs SDK (1=in motion,
      2=sleeping, 3=suspended, 4=stopping); mode table omits modes 4–7.
- [x] **L8** `actuator/joint.ex:66-68` + `robots/x_arm6.ex:59-64` — DSL `limit` blocks in the robot
      topology are ignored by the clamp (only `Model.get` limits used); tightened user limits have no
      effect on clamping.

## Test gaps to close alongside fixes

- [x] Malformed length-prefix tests for both parsers (crash + resync paths) — covers H1/H2
- [x] Grace-period test where a fault seen *during* grace persists past expiry — covers C4
- [x] Joint-move test decodes commanded positions (would have caught C5's 0.0s)
- [x] Cmd-socket failure paths: heartbeat/loop/transition with closed socket — covers C3
- [~] Pubsub end-to-end actuator delivery test — covers C2. Unit tests now assert each actuator
      subscribes to `[:actuator | path]` in init; a full end-to-end test (publish via
      `BB.Actuator.set_position/4` through a running sim robot and observe the frame) would be a
      good addition to `sim_test.exs` in a follow-up.
- [x] Model limit values asserted per model (not just tuple counts) — covers C1
- [x] ForceTorque disarm→re-arm re-enable test — covers H3
- [x] Out-of-range linear-track command test — covers M3
- [x] Invalid joint-index init test — covers M8
- [x] Doctests enabled for Protocol/Report — covers M7
- [x] Untested protocol builders: `cmd_linear_track_move/3`, `cmd_linear_track_read_position/1`,
      `parse_linear_track_position/1`, `cmd_gripper_speed/2`, `cmd_get_error/1`, `cmd_clean_error/1`,
      `cmd_set_mode/2`
