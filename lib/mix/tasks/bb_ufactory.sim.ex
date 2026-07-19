# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule Mix.Tasks.BbUfactory.Sim do
  @shortdoc "Manages the UFACTORY firmware simulator Docker container"

  @moduledoc """
  Manages the UFACTORY firmware simulator Docker container for integration
  testing.

  The simulator runs UFACTORY's real controller firmware, speaking the same
  command (502) and report (30003) protocol as a physical arm, with the
  UFACTORY Studio web UI on http://127.0.0.1:18333 for visual verification.
  See the *Testing against the UFACTORY simulator* tutorial and
  `BB.Ufactory.SimulatorCase` for building test suites on top of it.

      mix bb_ufactory.sim start [MODEL]   # start container + firmware (default: xarm6)
      mix bb_ufactory.sim stop            # stop and remove the container
      mix bb_ufactory.sim status          # container / firmware readiness
      mix bb_ufactory.sim logs            # print recent container logs

  `MODEL` is one of `xarm5`, `xarm6`, `xarm7`, `lite6`, `xarm850` — the same
  atoms `BB.Ufactory.Model` uses. The container ships one firmware binary per
  model (Lite6 is UFACTORY device type 9, UF850 is type 12; both 6-axis).

  `start` waits for full firmware readiness (protocol-level, via
  `BB.Ufactory.Simulator.wait_until_ready/1`) — the firmware accepts TCP
  connections several seconds before its services respond, so a bare port
  probe is not enough.

  ## Options

  - `--image IMAGE` — override the Docker image (default:
    `#{"danielwang123321/uf-ubuntu-docker"}`, a community-published image;
    it is amd64-only and runs under emulation on Apple Silicon)
  - `--name NAME` — override the container name (default `bb_ufactory_sim`)

  A typical test loop:

      mix bb_ufactory.sim start lite6
      SIM_MODEL=lite6 mix test --include simulator
      mix bb_ufactory.sim stop
  """

  use Mix.Task

  alias BB.Ufactory.Simulator

  @default_image "danielwang123321/uf-ubuntu-docker"
  @default_name "bb_ufactory_sim"

  # Model → "<axes> <device_type>" arguments for /xarm_scripts/xarm_start.sh
  @models %{
    "xarm5" => ["5", "5"],
    "xarm6" => ["6", "6"],
    "xarm7" => ["7", "7"],
    "lite6" => ["6", "9"],
    "xarm850" => ["6", "12"]
  }

  @ports ~w(18333 502 503 504 30000 30001 30002 30003)

  @impl Mix.Task
  def run(argv) do
    {opts, args} = OptionParser.parse!(argv, strict: [image: :string, name: :string])
    image = opts[:image] || @default_image
    name = opts[:name] || @default_name

    # Compile and load this library's modules WITHOUT starting the host
    # application — a consumer's app may attempt hardware connections on
    # boot, which a tooling task must never trigger.
    Mix.Task.run("compile")

    case args do
      ["start"] -> start(image, name, "xarm6")
      ["start", model] -> start(image, name, model)
      ["stop"] -> stop(name)
      ["status"] -> status(name)
      ["logs"] -> logs(name)
      _ -> Mix.raise("Usage: mix bb_ufactory.sim start [MODEL] | stop | status | logs")
    end
  end

  defp start(image, name, model) do
    firmware_args =
      Map.get(@models, model) ||
        Mix.raise(
          "Unknown model #{inspect(model)} — expected one of: #{Enum.join(Map.keys(@models), ", ")}"
        )

    # Always recreate so a previously started firmware (possibly a different
    # model) is fully torn down.
    docker(["rm", "-f", name], ok_to_fail: true)

    port_flags = Enum.flat_map(@ports, fn p -> ["-p", "#{p}:#{p}"] end)

    # -it: the image's entrypoint is a bare shell that exits without a TTY.
    # --platform: the image is amd64-only; ARM hosts run it under emulation.
    {_, 0} =
      docker(
        ["run", "-dit", "--platform", "linux/amd64", "--name", name] ++
          port_flags ++ [image]
      )

    Mix.shell().info("Container #{name} started.")

    Mix.shell().info(
      "Starting #{model} firmware (xarm_start.sh #{Enum.join(firmware_args, " ")})..."
    )

    {_, 0} = docker(["exec", "-d", name, "/xarm_scripts/xarm_start.sh" | firmware_args])

    Mix.shell().info("Waiting for firmware readiness (protocol-level probe)...")

    case Simulator.wait_until_ready(timeout: 90_000) do
      :ok ->
        Mix.shell().info("Simulator ready (#{model}). Studio UI: http://127.0.0.1:18333")
        Mix.shell().info("Run tests with: SIM_MODEL=#{model} mix test --include simulator")

      {:error, :timeout} ->
        Mix.raise("Firmware did not become ready within 90s — check: mix bb_ufactory.sim logs")
    end
  end

  defp stop(name) do
    case docker(["rm", "-f", name], ok_to_fail: true) do
      {_, 0} -> Mix.shell().info("Container #{name} removed.")
      _ -> Mix.shell().info("Container #{name} was not running.")
    end
  end

  defp status(name) do
    case docker(["ps", "-q", "-f", "name=^#{name}$"], ok_to_fail: true) do
      {out, 0} when out != "" ->
        Mix.shell().info("Container: running")

        if Simulator.available?() do
          Mix.shell().info("Firmware: ready (command + report sockets answering)")
        else
          Mix.shell().info("Firmware: NOT ready (still booting, or not started)")
        end

      _ ->
        Mix.shell().info("Container: not running")
    end
  end

  defp logs(name) do
    {out, _} = docker(["logs", "--tail", "200", name], ok_to_fail: true)
    Mix.shell().info(out)
  end

  defp docker(args, opts \\ []) do
    {out, exit_code} = System.cmd("docker", args, stderr_to_stdout: true)

    if exit_code != 0 and not Keyword.get(opts, :ok_to_fail, false) do
      Mix.raise("docker #{Enum.join(args, " ")} failed (exit #{exit_code}):\n#{out}")
    end

    {String.trim(out), exit_code}
  rescue
    e in ErlangError ->
      if e.original == :enoent do
        Mix.raise("The docker CLI was not found on PATH — install Docker to use the simulator.")
      else
        reraise e, __STACKTRACE__
      end
  end
end
