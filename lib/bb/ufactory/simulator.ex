# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Simulator do
  @moduledoc """
  Helpers for integration-testing against a UFACTORY arm — simulated or real.

  UFACTORY ships its controller firmware as a Docker image, which speaks the
  exact same wire protocol as a physical arm (command socket on port 502,
  real-time report stream on port 30003). That makes it possible to run true
  end-to-end tests of robot applications built on this library without
  hardware. See the *Testing against the UFACTORY simulator* tutorial for
  the full workflow, and `mix bb_ufactory.sim` for managing the container.

  Despite the module name, every function here speaks the plain UFACTORY
  protocol and works identically against a physical arm — point `:host` at
  the arm's IP instead of the simulator's.

  ## Typical usage

  In an ExUnit suite, prefer `BB.Ufactory.SimulatorCase`, which wires these
  helpers up with readiness gating and tagging. Standalone:

      alias BB.Ufactory.{Protocol, Simulator}

      :ok = Simulator.wait_until_ready(timeout: 60_000)

      {:ok, cmd} = Simulator.connect_command()
      :ok = Simulator.prepare_arm(cmd)

      {:ok, {_reg, _status, params}, _rest} =
        Simulator.command(cmd, Protocol.cmd_get_error(0))

  ## Options

  All functions taking `opts` accept:

  - `:host` — arm/simulator address (default: the `SIM_HOST` environment
    variable, falling back to `"127.0.0.1"`)
  - `:command_port` — command socket port (default 502)
  - `:report_port` — real-time report port (default 30003)

  ## Firmware quirks worth knowing

  Observed against both the simulator and documented SDK behavior:

  - The firmware accepts TCP connections a few seconds before its services
    respond — always gate on `wait_until_ready/1`, never on a bare connect.
  - `MOTION_EN` (0x0B) may get no response; `prepare_arm/2` sends its
    sequence fire-and-forget and drains, which works on simulator and
    hardware alike.
  - Response status bytes carry flag bits (0x20 warning pending, 0x40 error
    pending), not plain zero.
  - `IS_TCP_LIMIT` checks the *configured* safety boundary, not
    reachability — use `reachable?/3` (an FK/IK round-trip through the
    firmware's own kinematics) to probe the workspace.
  """

  alias BB.Ufactory.Protocol
  alias BB.Ufactory.Report

  @default_command_port 502
  @default_report_port 30_003
  @connect_timeout 5_000

  @typedoc """
  Connection options. `:timeout` is honored only by `wait_until_ready/1`.
  """
  @type opts :: [
          host: String.t(),
          command_port: pos_integer(),
          report_port: pos_integer(),
          timeout: pos_integer()
        ]

  @typedoc "A parsed real-time report frame — see `BB.Ufactory.Report`."
  @type report :: map()

  defp host(opts), do: Keyword.get(opts, :host, System.get_env("SIM_HOST", "127.0.0.1"))
  defp command_port(opts), do: Keyword.get(opts, :command_port, @default_command_port)
  defp report_port(opts), do: Keyword.get(opts, :report_port, @default_report_port)

  # ── Connections ─────────────────────────────────────────────────────────────

  @doc """
  Opens a passive TCP connection to the command socket (port 502).
  """
  @spec connect_command(opts()) :: {:ok, :gen_tcp.socket()} | {:error, term()}
  def connect_command(opts \\ []) do
    :gen_tcp.connect(
      String.to_charlist(host(opts)),
      command_port(opts),
      [:binary, active: false, packet: :raw],
      @connect_timeout
    )
  end

  @doc """
  Opens a passive TCP connection to the real-time report stream (port 30003).

  Read frames with `read_report/2` or `await_report/4`.
  """
  @spec connect_report(opts()) :: {:ok, :gen_tcp.socket()} | {:error, term()}
  def connect_report(opts \\ []) do
    :gen_tcp.connect(
      String.to_charlist(host(opts)),
      report_port(opts),
      [:binary, active: false, packet: :raw],
      @connect_timeout
    )
  end

  # ── Readiness ───────────────────────────────────────────────────────────────

  @doc """
  Checks whether the firmware is actually ready — not merely accepting TCP.

  Ready means the command socket answers a `GET_STATE` request AND the
  report stream produces a parseable frame. Returns `false` on any failure.
  """
  @spec available?(opts()) :: boolean()
  def available?(opts \\ []) do
    command_responding?(opts) and report_streaming?(opts)
  rescue
    _ -> false
  catch
    _, _ -> false
  end

  @doc """
  Blocks until `available?/1` returns true, polling once per second.

  Accepts the standard connection opts plus `:timeout` in milliseconds
  (default 60000). Returns `:ok`, or `{:error, :timeout}` if the firmware
  never became ready.

  A freshly started simulator needs several seconds after its ports open
  before this succeeds; a physical arm is typically ready immediately.
  """
  @spec wait_until_ready(opts()) :: :ok | {:error, :timeout}
  def wait_until_ready(opts \\ []) do
    timeout = Keyword.get(opts, :timeout, 60_000)
    deadline = System.monotonic_time(:millisecond) + timeout
    poll_until_ready(opts, deadline)
  end

  defp poll_until_ready(opts, deadline) do
    cond do
      available?(opts) ->
        :ok

      System.monotonic_time(:millisecond) > deadline ->
        {:error, :timeout}

      true ->
        Process.sleep(1_000)
        poll_until_ready(opts, deadline)
    end
  end

  defp command_responding?(opts) do
    with {:ok, socket} <- connect_command(opts),
         :ok <- :gen_tcp.send(socket, Protocol.heartbeat()),
         {:ok, data} <- :gen_tcp.recv(socket, 0, 2_000) do
      :gen_tcp.close(socket)
      match?({:ok, {0x0D, _, _}, _}, Protocol.parse_response(data))
    else
      _ -> false
    end
  end

  defp report_streaming?(opts) do
    case connect_report(opts) do
      {:ok, socket} ->
        result = match?({:ok, _report, _rest}, read_report(socket, <<>>, 3_000))
        :gen_tcp.close(socket)
        result

      _ ->
        false
    end
  end

  # ── Command socket helpers ──────────────────────────────────────────────────

  @doc """
  Drains any unread responses from a command socket's receive buffer.

  Fire-and-forget frames (moves, RS485 writes, `prepare_arm/2`) generate
  responses nobody reads; drain before a request/response exchange so the
  next `recv` reads its own reply. `command/3` does this automatically.
  """
  @spec drain(:gen_tcp.socket()) :: :ok
  def drain(socket) do
    case :gen_tcp.recv(socket, 0, 200) do
      {:ok, _} -> drain(socket)
      {:error, _} -> :ok
    end
  end

  @doc """
  Sends a command frame and returns the parsed response.

  Drains stale responses first, then sends `frame` (built with
  `BB.Ufactory.Protocol`) and parses the reply. Returns
  `{:ok, {register, status, params}, rest}`, `{:more}` on a truncated
  response, or `{:error, reason}`.

  ## Examples

      {:ok, cmd} = Simulator.connect_command()

      {:ok, {0x0F, _status, <<error_code, _warn>>}, _rest} =
        Simulator.command(cmd, Protocol.cmd_get_error(0))
  """
  @spec command(:gen_tcp.socket(), binary(), timeout()) ::
          {:ok, {byte(), byte(), binary()}, binary()} | {:more} | {:error, term()}
  def command(socket, frame, timeout \\ 5_000) do
    drain(socket)

    with :ok <- :gen_tcp.send(socket, frame),
         {:ok, data} <- :gen_tcp.recv(socket, 0, timeout) do
      Protocol.parse_response(data)
    end
  end

  @doc """
  Sends a frame without waiting for a response, then drains.

  Some registers (notably `MOTION_EN`) may never get a reply from the
  simulator firmware; send those fire-and-forget.
  """
  @spec send_frame(:gen_tcp.socket(), binary()) :: :ok | {:error, term()}
  def send_frame(socket, frame) do
    with :ok <- :gen_tcp.send(socket, frame) do
      drain(socket)
    end
  end

  @doc """
  Brings the arm to a known-good, motion-ready state.

  Sends the standard recovery sequence fire-and-forget: clean error, clean
  warning, enable motors, position-control mode, ready state. Options:

  - `:self_collision_check` — when `false`, additionally disables the
    firmware's geometric self-collision model. Useful when probing joint
    limits (the collision model otherwise fires first on folded poses).
    Re-enable it afterwards by calling `prepare_arm(socket,
    self_collision_check: true)` or leaving the option out on a fresh
    simulator. Defaults to leaving the current setting untouched.
  """
  @spec prepare_arm(:gen_tcp.socket(), keyword()) :: :ok
  def prepare_arm(socket, opts \\ []) do
    send_frame(socket, Protocol.cmd_clean_error(0))
    send_frame(socket, Protocol.cmd_clean_warn(1))
    send_frame(socket, Protocol.cmd_enable(2, true))
    send_frame(socket, Protocol.cmd_set_mode(3, 0))

    case Keyword.fetch(opts, :self_collision_check) do
      {:ok, enabled?} -> send_frame(socket, Protocol.cmd_set_self_collision_check(4, enabled?))
      :error -> :ok
    end

    send_frame(socket, Protocol.cmd_set_state(5, 0))
    Process.sleep(300)
    drain(socket)
  end

  # ── Report stream helpers ───────────────────────────────────────────────────

  @doc """
  Reads one complete report frame from a report-stream socket.

  `buffer` must be the unconsumed remainder (`rest`) from the previous call
  — starting fresh mid-stream would misalign frame boundaries. Returns
  `{:ok, report, rest}` or `{:error, reason}` (including
  `{:error, {:bad_frame_length, n}}` on a desynchronized stream).
  """
  @spec read_report(:gen_tcp.socket(), binary(), timeout()) ::
          {:ok, report(), binary()} | {:error, term()}
  def read_report(socket, buffer \\ <<>>, timeout \\ 2_000) do
    case Report.parse_report(buffer) do
      {:ok, report, rest} ->
        {:ok, report, rest}

      {:more} ->
        case :gen_tcp.recv(socket, 0, timeout) do
          {:ok, data} -> read_report(socket, buffer <> data, timeout)
          {:error, reason} -> {:error, reason}
        end

      {:error, reason} ->
        {:error, reason}
    end
  end

  @doc """
  Reads report frames until `fun` returns a truthy value or `timeout`
  elapses.

  Useful for observing motion: "wait until J1 is within 0.02 rad of the
  target". Returns `{:ok, report, rest}` with the first matching frame, or
  `{:error, :timeout}` with the deadline exceeded.

  ## Examples

      {:ok, report_socket} = Simulator.connect_report()
      {:ok, _first, rest} = Simulator.read_report(report_socket)

      {:ok, _frame, _rest} =
        Simulator.await_report(report_socket, rest, 20_000, fn r ->
          abs(hd(r.angles) - target) < 0.02
        end)
  """
  @spec await_report(:gen_tcp.socket(), binary(), pos_integer(), (report() -> as_boolean(term()))) ::
          {:ok, report(), binary()} | {:error, term()}
  def await_report(socket, buffer, timeout, fun) do
    deadline = System.monotonic_time(:millisecond) + timeout
    do_await_report(socket, buffer, deadline, fun)
  end

  defp do_await_report(socket, buffer, deadline, fun) do
    with {:ok, report, rest} <- read_report(socket, buffer) do
      cond do
        fun.(report) -> {:ok, report, rest}
        System.monotonic_time(:millisecond) > deadline -> {:error, :timeout}
        true -> do_await_report(socket, rest, deadline, fun)
      end
    end
  end

  @doc """
  Snapshots the arm's current state from a fresh report connection.

  Opens a new connection so the frame reflects *now* rather than a stale
  buffer backlog. Returns `{:ok, report}` with the full report map
  (`:angles`, `:pose`, `:state`, `:mode`, …) or `{:error, reason}`.
  """
  @spec current_state(opts()) :: {:ok, report()} | {:error, term()}
  def current_state(opts \\ []) do
    with {:ok, socket} <- connect_report(opts),
         {:ok, report, _rest} <- read_report(socket) do
      :gen_tcp.close(socket)
      {:ok, report}
    end
  end

  # ── Workspace probing ───────────────────────────────────────────────────────

  @doc """
  Probes whether a Cartesian pose is reachable, using the firmware's own
  kinematics.

  Runs the pose through the firmware's IK, feeds the resulting joint
  configuration back through its FK, and measures the positional error: a
  reachable pose round-trips within a few millimetres, while an unreachable
  pose comes back wildly off (the firmware returns unclamped IK solutions).
  This is the reliable reachability probe — `IS_TCP_LIMIT` only checks the
  configured safety boundary.

  `pose` is `{x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad}`. Options:

  - `:tolerance_mm` — maximum round-trip error to count as reachable
    (default 25.0; observed errors are < 5 mm for reachable poses and
    > 100 mm for unreachable ones)

  Returns `{:ok, boolean}` or `{:error, reason}` on a protocol failure.

  Sweeping this over a grid maps the arm's true reachable workspace.
  """
  @spec reachable?(
          :gen_tcp.socket(),
          {number(), number(), number(), number(), number(), number()},
          keyword()
        ) :: {:ok, boolean()} | {:error, term()}
  def reachable?(socket, {x, y, z, _roll, _pitch, _yaw} = pose, opts \\ []) do
    tolerance = Keyword.get(opts, :tolerance_mm, 25.0)

    with {:ok, {_reg, _s1, ik_params}, _} <- command(socket, Protocol.cmd_get_ik(0, pose)),
         {:ok, angles} <- Protocol.parse_ik_response(ik_params),
         {:ok, {_reg2, _s2, fk_params}, _} <- command(socket, Protocol.cmd_get_fk(1, angles)),
         {:ok, {fx, fy, fz, _r, _p, _yw}} <- Protocol.parse_fk_response(fk_params) do
      error = :math.sqrt((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2)
      {:ok, error <= tolerance}
    end
  end
end
