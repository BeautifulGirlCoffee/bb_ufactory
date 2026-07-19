# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.SimulatorCase do
  @moduledoc """
  An ExUnit case template for integration tests against the UFACTORY
  firmware simulator (or a physical arm).

  Handles the boilerplate that firmware integration tests need:

  - Tags every test `:simulator` (exclude it by default in your
    `test_helper.exs`: `ExUnit.start(exclude: [:simulator])`) and applies a
    generous per-test timeout — firmware round-trips are slow compared to
    unit tests.
  - Gates the whole module on `BB.Ufactory.Simulator.wait_until_ready/1`
    in `setup_all`, so tests never run against a firmware that is still
    booting (the simulator accepts TCP several seconds before its services
    respond — every cold start hits this window).
  - Imports `BB.Ufactory.Simulator` and aliases `BB.Ufactory.Protocol` /
    `BB.Ufactory.Report`.
  - Provides `sim_model/0` and `sim_joints/0`, reading the `SIM_MODEL`
    environment variable so one suite can run against every arm model
    (matching `mix bb_ufactory.sim start <model>`).

  ## Usage

      # test/my_robot/simulator_test.exs
      defmodule MyRobot.SimulatorTest do
        use BB.Ufactory.SimulatorCase

        test "arm reports joint state" do
          {:ok, report} = current_state()
          assert length(Enum.take(report.angles, sim_joints())) == sim_joints()
        end

        test "my pick pose is reachable" do
          {:ok, cmd} = connect_command()
          prepare_arm(cmd)
          assert {:ok, true} = reachable?(cmd, {300.0, 0.0, 200.0, 3.14, 0.0, 0.0})
        end
      end

  Run against a started simulator (see `mix bb_ufactory.sim`):

      mix bb_ufactory.sim start lite6
      SIM_MODEL=lite6 mix test --include simulator

  ## Options

  - `:readiness_timeout` — milliseconds to wait for firmware readiness in
    `setup_all` before failing the module (default 60000):

        use BB.Ufactory.SimulatorCase, readiness_timeout: 120_000

  Tests run against one shared firmware instance, so suites using this
  template are implicitly `async: false`.
  """

  use ExUnit.CaseTemplate

  alias BB.Ufactory.Model

  using opts do
    quote do
      @moduletag :simulator
      @moduletag timeout: 120_000

      import BB.Ufactory.Simulator
      import BB.Ufactory.SimulatorCase, only: [sim_model: 0, sim_joints: 0]

      alias BB.Ufactory.Protocol
      alias BB.Ufactory.Report

      @simulator_readiness_timeout Keyword.get(unquote(opts), :readiness_timeout, 60_000)

      setup_all do
        case wait_until_ready(timeout: @simulator_readiness_timeout) do
          :ok ->
            :ok

          {:error, :timeout} ->
            raise "UFACTORY firmware did not become ready within " <>
                    "#{@simulator_readiness_timeout}ms — is the simulator running? " <>
                    "(mix bb_ufactory.sim status)"
        end
      end
    end
  end

  @doc """
  The arm model under test, from the `SIM_MODEL` environment variable
  (default `:xarm6`).

  Matches the model names accepted by `mix bb_ufactory.sim start` and
  `BB.Ufactory.Model`.
  """
  @spec sim_model() :: atom()
  def sim_model do
    "SIM_MODEL"
    |> System.get_env("xarm6")
    |> String.to_existing_atom()
  end

  @doc """
  The joint count of the model under test — see `sim_model/0`.
  """
  @spec sim_joints() :: pos_integer()
  def sim_joints, do: Model.joint_count(sim_model())
end
