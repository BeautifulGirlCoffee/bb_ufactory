# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule TestRobot do
  @moduledoc """
  Shared test module providing xArm6 model configuration constants.

  Used as the robot atom (`%{bb: %{robot: TestRobot, ...}}`) across all
  actuator, sensor, and controller tests. All interactions with BB framework
  modules are mocked via Mimic — this module simply provides a stable atom
  and centralised constants so individual test files don't duplicate them.
  """

  @pi :math.pi()
  @two_pi 2 * :math.pi()

  @xarm6_model_config %{
    joints: 6,
    max_speed_rads: @pi,
    limits: [
      {-@two_pi, @two_pi},
      {-2.059488, 2.094395},
      {-3.926990, 0.191986},
      {-@two_pi, @two_pi},
      {-1.692969, @pi},
      {-@two_pi, @two_pi}
    ]
  }

  @doc "Returns the xArm6 model config map (joints, limits, max_speed_rads)."
  @spec model_config() :: map()
  def model_config, do: @xarm6_model_config

  @doc "Returns the joint count for the xArm6."
  @spec joint_count() :: pos_integer()
  def joint_count, do: @xarm6_model_config.joints

  @doc "Returns the `{lower, upper}` limit tuple for `joint_index` (1-based)."
  @spec joint_limits(pos_integer()) :: {float(), float()}
  def joint_limits(joint_index), do: Enum.at(@xarm6_model_config.limits, joint_index - 1)

  @doc "Returns the max joint speed in rad/s."
  @spec max_speed() :: float()
  def max_speed, do: @xarm6_model_config.max_speed_rads

  @doc """
  Creates a pre-populated ETS table with `joint_count` joint rows and one arm row.

  Each joint row: `{index, current_position, current_torque, set_position}`
  Arm row: `{:arm, state, mode, tcp_pose}`
  """
  @spec make_ets(pos_integer()) :: :ets.table()
  def make_ets(joint_count \\ 6) do
    ets = :ets.new(:test_ets, [:public, :set])
    for i <- 1..joint_count, do: :ets.insert(ets, {i, nil, nil, nil})
    :ets.insert(ets, {:arm, 0, 0, nil})
    ets
  end
end
