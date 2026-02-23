# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Message.CartesianPose do
  @moduledoc """
  Cartesian end-effector pose from the xArm TCP (tool-centre-point).

  Published by `BB.Ufactory.Controller` to `[:sensor, controller_name, :tcp_pose]`
  on every real-time report frame received from the arm.

  ## Fields

  - `x`, `y`, `z` — position in **millimetres** relative to the arm base frame
  - `roll`, `pitch`, `yaw` — orientation in **radians** (RPY convention)

  These values are taken directly from the auto-push report (bytes 35–58 of the
  real-time frame) without unit conversion; the xArm wire protocol uses mm for
  position and radians for orientation.
  """

  defstruct [:x, :y, :z, :roll, :pitch, :yaw]

  use BB.Message,
    schema: [
      x: [type: :float, required: true, doc: "X position in mm"],
      y: [type: :float, required: true, doc: "Y position in mm"],
      z: [type: :float, required: true, doc: "Z position in mm"],
      roll: [type: :float, required: true, doc: "Roll in radians"],
      pitch: [type: :float, required: true, doc: "Pitch in radians"],
      yaw: [type: :float, required: true, doc: "Yaw in radians"]
    ]

  @type t :: %__MODULE__{
          x: float(),
          y: float(),
          z: float(),
          roll: float(),
          pitch: float(),
          yaw: float()
        }
end
