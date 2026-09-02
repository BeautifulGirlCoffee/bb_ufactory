<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Cartesian Motion

The xArm can accept commands in two distinct coordinate spaces:

- **Joint-space** — specify a target angle for each of the six joints
- **Cartesian space** — specify a target pose (x, y, z, roll, pitch, yaw) for the
  end-effector (TCP). The arm's on-board controller solves inverse kinematics internally.

`bb_ufactory` provides a separate actuator for each approach.

## Joint-Space Commands — `BB.Ufactory.Actuator.Joint`

Joint-space is the default and is what most robot programs use. One `Actuator.Joint`
instance is declared per joint in the topology, each identified by its hardware
joint index (1–6 for the xArm6).

### How it works

1. A position command arrives at the actuator (via `BB.Actuator.set_position/4`, pubsub, or cast).
2. The angle is clamped to the joint's configured limits.
3. The target angle is written into the controller's **ETS table** under the key
   for that joint.
4. On the next 100 Hz tick (≤10 ms later), the controller reads all pending
   positions from ETS and sends a single `MOVE_JOINT` frame (register 0x17)
   covering all joints simultaneously.

This batching means that sending commands to multiple joints in rapid succession
does **not** produce multiple wire frames — they all coalesce into one frame per
tick. The arm moves all joints concurrently, which is the expected behaviour for
smooth multi-joint motion.

### Sending a joint-space command

```elixir
# Move J1 to 45° and J2 to -30° (commands are independent; both land in ETS).
# set_position/4 is synchronous: :ok means the command was accepted.
:ok = BB.Actuator.set_position(MyRobot, :j1_motor, :math.pi() / 4)
:ok = BB.Actuator.set_position(MyRobot, :j2_motor, -:math.pi() / 6)

# An optional velocity hint caps the batch speed for the move (rad/s):
:ok = BB.Actuator.set_position(MyRobot, :j1_motor, 0.5, velocity: 0.2)
```

Both commands are picked up in the same or adjacent loop tick and sent as one
`MOVE_JOINT` frame. The arm executes them simultaneously.

### Reading joint positions back

The controller's report handler (port 30003) updates `current_position` in ETS
for each joint on every push frame (~100 Hz) and publishes
`BB.Message.Sensor.JointState`:

```elixir
BB.subscribe(robot, [:sensor, :xarm, :j1])

receive do
  {:bb, [:sensor, :xarm, :j1], %BB.Message{payload: js}} ->
    IO.inspect(js.position, label: "J1 (rad)")
end
```

---

## Cartesian-Space Commands — `BB.Ufactory.Actuator.Cartesian`

The Cartesian actuator lets you command the TCP pose directly without computing
joint angles yourself. The arm's on-board IK solver handles the conversion.

Declare it once at the base link level (it is not per-joint):

```elixir
topology do
  link :base do
    # ... joints j1–j6 ...

    actuator :cartesian, {BB.Ufactory.Actuator.Cartesian,
      controller: :xarm,
      speed: 100.0,         # mm/s; default: 100.0
      acceleration: 2000.0  # mm/s²; default: 2000.0
    }
  end
end
```

### How it works

1. A `BB.Ufactory.Message.Command.CartesianMove` command arrives at the
   actuator through BB's gated command pipeline (the actuator declares the
   payload via `c:BB.Actuator.command_payloads/1`, so the framework checks
   the robot is armed before the driver sees it).
2. A `MOVE_LINE` frame (register 0x15) is built immediately with the pose and
   the configured (or per-command) speed/acceleration.
3. The frame is forwarded **directly** to the controller via `BB.Process.call/3`
   and sent to the arm over port 502.

Cartesian commands **bypass the ETS batch loop**. Each command produces exactly
one wire frame, dispatched synchronously before the call returns.

### Sending a Cartesian command

```elixir
alias BB.Ufactory.Message.Command.CartesianMove

# Move TCP to x=300mm, y=0mm, z=400mm, no rotation
msg = BB.Message.new!(CartesianMove, :cartesian,
  x: 300.0, y: 0.0, z: 400.0, roll: 0.0, pitch: 0.0, yaw: 0.0)

# Synchronous: returns {:ok, :accepted} once the arm has taken the frame,
# or {:error, reason} — including a refusal while the robot is disarmed.
BB.call(robot, :cartesian, {:command, msg})

# Fire-and-forget:
BB.cast(robot, :cartesian, {:command, msg})
```

Field units:
- `x`, `y`, `z` are in **millimetres** relative to the arm base frame
- `roll`, `pitch`, `yaw` are in **radians**

### Overriding speed and acceleration per command

```elixir
# Slow move for a precision pick
msg = BB.Message.new!(CartesianMove, :cartesian,
  x: 200.0, y: 100.0, z: 250.0, roll: 0.0, pitch: 0.0, yaw: 0.0,
  speed: 50.0,          # mm/s
  acceleration: 500.0)  # mm/s²

BB.call(robot, :cartesian, {:command, msg})
```

### Legacy cast interface (deprecated)

The pre-0.2 raw cast still works but is deprecated: it logs a warning on
every use, and since 0.2.0 it is **dropped when the robot is not armed**
(previously it would move a disarmed robot). Prefer `CartesianMove`:

```elixir
BB.Process.cast(robot, :cartesian, {:move_cartesian, {300.0, 0.0, 400.0, 0.0, 0.0, 0.0}})
BB.Process.cast(robot, :cartesian, {:move_cartesian, pose, 50.0, 500.0})
```

### Reading the current Cartesian pose

The controller publishes `BB.Ufactory.Message.CartesianPose` from each report
frame. Subscribe to the TCP pose path:

```elixir
BB.subscribe(robot, [:sensor, :xarm, :tcp_pose])

receive do
  {:bb, [:sensor, :xarm, :tcp_pose], %BB.Message{payload: pose}} ->
    IO.puts("TCP position: x=#{pose.x}mm y=#{pose.y}mm z=#{pose.z}mm")
    IO.puts("TCP orientation: roll=#{pose.roll} pitch=#{pose.pitch} yaw=#{pose.yaw} (rad)")
end
```

---

## Joint-Space vs Cartesian — When to Use Each

| Consideration | Joint-Space | Cartesian |
|---------------|-------------|-----------|
| **Ease of programming** | Requires knowing target joint angles | Intuitive — specify where the hand should be |
| **IK singularities** | None — joints move directly | Arm may reject commands near singularities |
| **Multi-joint coordination** | Batched at 100 Hz — all joints move together | Single command moves TCP; arm handles coordination |
| **Trajectory smoothness** | Depends on your rate of ETS writes | Arm's motion planner handles interpolation |
| **Speed/accel control** | Set in the `MOVE_JOINT` frame (via controller options) | Per-actuator and per-command override |
| **Accessories** | Works alongside gripper, track | Works alongside gripper, track |
| **Best for** | Repeatable joint configurations, learning poses, simulation | Pick-and-place, task-space programming, reactive control |

### Mixing both approaches

You can declare both actuators in the same robot and switch between them freely.
The ETS table and the direct Cartesian path share the same controller — there is
no conflict as long as you do not send joint and Cartesian commands simultaneously,
since overlapping motion commands will cause the arm to reject or abort the earlier
motion.

```elixir
topology do
  link :base do
    # Joint actuators for precise joint-space control
    joint :j1 do
      # ...
      actuator :j1_motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm}
      # ...
    end
    # ... j2–j6 ...

    # Cartesian actuator for task-space commands
    actuator :cartesian, {BB.Ufactory.Actuator.Cartesian,
      controller: :xarm,
      speed: 100.0,
      acceleration: 2000.0
    }
  end
end
```

---

## Protocol Reference

| Actuator | Register | Encoding |
|----------|----------|----------|
| `Actuator.Joint` | 0x17 `MOVE_JOINT` | 10× fp32 LE: angles[7] + speed + accel + mvtime |
| `Actuator.Cartesian` | 0x15 `MOVE_LINE` | 9× fp32 LE: pose[6] + speed + accel + mvtime |

Both registers produce a response with a status byte. A non-zero status byte
raises `BB.Error.Protocol.Ufactory.CommandRejected`.

Do not confuse register 0x15 (`MOVE_LINE`, Cartesian linear) with register 0x18
(`MOVE_LINE_AA`, arc-blending variant). `Actuator.Cartesian` always uses 0x15.
