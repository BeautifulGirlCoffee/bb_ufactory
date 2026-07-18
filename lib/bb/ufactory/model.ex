# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Model do
  @moduledoc """
  Per-model configuration for UFactory xArm variants.

  Provides joint counts, maximum joint speed, and per-joint radian limits for
  each supported model. Limits are taken from the official `xarm_ros2` URDF
  descriptions (`xarm_description/urdf/<model>/<model>.urdf.xacro`), which
  match the Python SDK's per-model configuration.

  The map keys are the model atoms accepted by `BB.Ufactory.Controller`:
  `:xarm5`, `:xarm6`, `:xarm7`, `:lite6`, `:xarm850`.

  Joint limits are `{lower_rad, upper_rad}` tuples, one per joint, ordered J1..Jn.

  ## Notes

  - All values are in **radians**.
  - `max_speed_rads` is the maximum joint angular speed (rad/s).
  - These are the factory hard limits enforced by the firmware. They cannot be
    widened; commands are clamped to them before being sent to the arm.
  """

  @pi :math.pi()
  @two_pi 2 * :math.pi()

  @models %{
    xarm5: %{
      joints: 5,
      max_speed_rads: @pi,
      limits: [
        {-@two_pi, @two_pi},
        {-2.059488, 2.094395},
        {-3.926990, 0.191986},
        {-1.692969, @pi},
        {-@two_pi, @two_pi}
      ]
    },
    xarm6: %{
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
    },
    xarm7: %{
      joints: 7,
      max_speed_rads: @pi,
      limits: [
        {-@two_pi, @two_pi},
        {-2.059488, 2.094395},
        {-@two_pi, @two_pi},
        {-0.191986, 3.926990},
        {-@two_pi, @two_pi},
        {-1.692969, @pi},
        {-@two_pi, @two_pi}
      ]
    },
    lite6: %{
      joints: 6,
      max_speed_rads: @pi,
      limits: [
        {-@two_pi, @two_pi},
        {-2.617990, 2.617990},
        {-0.061087, 5.235988},
        {-@two_pi, @two_pi},
        {-2.164200, 2.164200},
        {-@two_pi, @two_pi}
      ]
    },
    xarm850: %{
      joints: 6,
      max_speed_rads: @pi,
      limits: [
        {-@two_pi, @two_pi},
        {-2.303835, 2.303835},
        {-4.223697, 0.061087},
        {-@two_pi, @two_pi},
        {-2.164200, 2.164200},
        {-@two_pi, @two_pi}
      ]
    }
  }

  @doc """
  Returns the full model configuration map for `model`.

  Raises `KeyError` for unknown model atoms.

  ## Examples

      iex> %{joints: 6} = BB.Ufactory.Model.get(:xarm6)
  """
  @spec get(atom()) :: %{
          joints: pos_integer(),
          max_speed_rads: float(),
          limits: [{float(), float()}]
        }
  def get(model), do: Map.fetch!(@models, model)

  @doc """
  Returns the number of joints for `model`.

  ## Examples

      iex> BB.Ufactory.Model.joint_count(:xarm7)
      7
  """
  @spec joint_count(atom()) :: pos_integer()
  def joint_count(model), do: get(model).joints

  @doc """
  Returns the maximum joint angular speed in rad/s for `model`.

  ## Examples

      iex> BB.Ufactory.Model.max_speed_rads(:xarm6)
      3.141592653589793
  """
  @spec max_speed_rads(atom()) :: float()
  def max_speed_rads(model), do: get(model).max_speed_rads

  @doc """
  Returns the per-joint `{lower, upper}` radian limits for `model`,
  ordered J1..Jn.

  ## Examples

      iex> [{-6.283185307179586, 6.283185307179586} | _] = BB.Ufactory.Model.joint_limits(:xarm6)
  """
  @spec joint_limits(atom()) :: [{float(), float()}]
  def joint_limits(model), do: get(model).limits

  @doc """
  Returns a list of all supported model atoms.

  ## Examples

      iex> :xarm6 in BB.Ufactory.Model.supported_models()
      true
  """
  @spec supported_models() :: [atom()]
  def supported_models, do: Map.keys(@models)
end
