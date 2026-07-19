<!--
SPDX-FileCopyrightText: 2026 Holden Oullette

SPDX-License-Identifier: Apache-2.0
-->

# Testing against the UFACTORY simulator

UFACTORY publishes its controller firmware as a Docker image. The simulated
firmware speaks the **exact same wire protocol** as a physical arm — command
socket on port 502, real-time report stream on port 30003 — which means you
can run true end-to-end tests of your robot application without hardware:
your `BB.Ufactory.Controller` connects, arms, streams sensor data, and moves
joints against the same firmware code that runs in the arm's control box.

This library ships everything needed to build such a suite:

- `mix bb_ufactory.sim` — manages the simulator container
- `BB.Ufactory.SimulatorCase` — an ExUnit case template with readiness
  gating and helpers
- `BB.Ufactory.Simulator` — the underlying protocol-level helpers (which
  also work against a physical arm)

## Prerequisites

Docker. That's it. The simulator image
(`danielwang123321/uf-ubuntu-docker`, ~1.2 GB) is a community-published
image containing UFACTORY's firmware binaries and the Studio web UI; it is
amd64-only, and `mix bb_ufactory.sim` handles the platform flag so it runs
under emulation on Apple Silicon. If the image ever moves, pass
`--image` to the task.

## Quick start

```sh
# Start the simulator with the firmware for your arm model
mix bb_ufactory.sim start lite6     # xarm5 | xarm6 | xarm7 | lite6 | xarm850

# Run your simulator-tagged tests against it
SIM_MODEL=lite6 mix test --include simulator

# Visual verification: UFACTORY Studio runs at http://127.0.0.1:18333

mix bb_ufactory.sim stop
```

## Writing a test suite

First, exclude simulator tests from your default run in `test/test_helper.exs`
(they need the container):

```elixir
ExUnit.start(exclude: [:simulator])
```

Then build on `BB.Ufactory.SimulatorCase`. It tags the module `:simulator`,
applies a generous timeout, gates the whole module on firmware readiness, and
imports the `BB.Ufactory.Simulator` helpers:

```elixir
defmodule MyApp.PickPlaceSimulatorTest do
  use BB.Ufactory.SimulatorCase

  test "arm streams joint state" do
    {:ok, report} = current_state()
    assert length(Enum.take(report.angles, sim_joints())) == sim_joints()
  end

  test "all poses in my pick sequence are reachable" do
    {:ok, cmd} = connect_command()
    prepare_arm(cmd)

    for pose <- MyApp.PickSequence.poses() do
      assert {:ok, true} = reachable?(cmd, pose),
             "unreachable pose in pick sequence: #{inspect(pose)}"
    end
  end
end
```

### Testing your full robot

The most valuable tests start your actual robot supervision tree against the
simulator — real controller, real pubsub, real motion:

```elixir
defmodule MyApp.RobotSimulatorTest do
  use BB.Ufactory.SimulatorCase

  setup do
    # Your robot module, with the controller host pointed at the simulator.
    {:ok, _pid} = MyApp.Robot.start_link()
    on_exit(fn -> Supervisor.stop(MyApp.Robot) end)
    :ok
  end

  test "arms and executes the home motion" do
    BB.subscribe(MyApp.Robot, [:sensor, :xarm])
    assert :ok = BB.Safety.arm(MyApp.Robot)

    MyApp.Actions.home()

    assert_receive {:bb, _, %BB.Message{payload: %BB.Message.Sensor.JointState{}}}, 5_000
    # ... assert convergence, then disarm
  end
end
```

Make the controller host configurable so the same robot module works in
production and against the simulator — for example read it from application
config, and set `config :my_app, arm_host: "127.0.0.1"` in `config/test.exs`.

### Observing motion

The report stream is the ground truth for "did the arm actually move". Read
it with an aligned buffer (always thread `rest` through — starting fresh
mid-stream misaligns frame boundaries):

```elixir
{:ok, report_socket} = connect_report()
{:ok, first, rest} = read_report(report_socket)

# ... command a move ...

{:ok, _frame, _rest} =
  await_report(report_socket, rest, 20_000, fn r ->
    abs(hd(r.angles) - target) < 0.02
  end)
```

### Multi-model coverage

`sim_model/0` and `sim_joints/0` read the `SIM_MODEL` environment variable,
so one suite can run against every arm model you support. In CI, run the
matrix (see below); locally, start whichever model you're targeting.

## Firmware quirks you will hit

These are real behaviors of UFACTORY's firmware, learned the hard way and
handled by the helpers — but worth knowing when you write your own probes:

| Quirk | Consequence |
|---|---|
| Ports accept TCP seconds before services respond | Never gate on a bare connect; use `wait_until_ready/1` (the case template does this for you) |
| `MOTION_EN` (0x0B) may get no response | Send enable fire-and-forget (`send_frame/2`, `prepare_arm/2`) |
| Response status bytes carry flag bits (0x20 warning, 0x40 error) | Don't assert `status == 0` on otherwise-successful commands |
| `IS_TCP_LIMIT` checks the configured safety boundary, not reachability | Use `reachable?/3` (FK∘IK round-trip) for workspace probing |
| The self-collision model fires before joint-limit enforcement on folded poses | Pass `self_collision_check: false` to `prepare_arm/2` when probing limits (error 23); restore it afterwards |
| Out-of-limit IK returns unclamped garbage angles with a warning status | This is what makes the `reachable?/3` round-trip discriminative |

## Running the simulator in CI

GitHub Actions' Linux runners are amd64 with Docker preinstalled, so the
firmware runs natively. A minimal job (see this repository's
`.github/workflows/ci.yml` for the full five-model matrix version):

```yaml
simulator:
  runs-on: ubuntu-latest
  timeout-minutes: 20
  strategy:
    fail-fast: false
    matrix:
      model: [xarm6, lite6]   # whichever models you target
  steps:
    - uses: actions/checkout@v4
    - uses: erlef/setup-beam@v1
      with:
        elixir-version: "1.20.2"
        otp-version: "29"
    - run: mix deps.get
    - run: MIX_ENV=test mix compile
    - run: MIX_ENV=test mix bb_ufactory.sim start ${{ matrix.model }}
    - run: SIM_MODEL=${{ matrix.model }} mix test --only simulator
    - if: failure()
      run: docker logs --tail 200 bb_ufactory_sim || true
    - if: always()
      run: MIX_ENV=test mix bb_ufactory.sim stop
```

## What the simulator does and doesn't cover

**Covered** (validated against the real firmware): the full wire protocol,
heartbeat/keep-alive, report-stream parsing, arm/disarm sequences, joint
motion and trajectory execution, firmware kinematics (FK/IK), joint-limit
enforcement (error 23), error-code reporting, and your application logic on
top of all of it.

**Not covered**: real dynamics (payloads, momentum, torque), RS485
accessories (gripper, linear track — the RS485 proxy has no devices behind
it in the container), force-torque sensing, and network failure modes of
real hardware. Keep a small hardware test pass for those.

## Using the helpers against a physical arm

Every `BB.Ufactory.Simulator` function speaks the plain protocol — point it
at real hardware for bench verification:

```elixir
opts = [host: "192.168.1.224"]
:ok = BB.Ufactory.Simulator.wait_until_ready(opts ++ [timeout: 10_000])
{:ok, cmd} = BB.Ufactory.Simulator.connect_command(opts)
```

**Safety:** on a physical arm, `prepare_arm/2` enables motors and motion
commands move real steel. Clear the workspace first.
