# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Message.Wrench do
  @moduledoc """
  Force-torque wrench reading from the xArm F/T sensor.

  Published by `BB.Ufactory.Controller` to `[:sensor, controller_name, :wrench]`
  when the real-time report frame (port 30003) contains `ft_filtered` data —
  i.e. when the F/T sensor is enabled and the arm firmware returns 135+ byte frames.

  Re-published by `BB.Ufactory.Sensor.ForceTorque` to `[:sensor | sensor_path]`
  for consumption by the application layer.

  ## Fields

  - `fx`, `fy`, `fz` — force components in **Newtons** in the sensor frame
  - `tx`, `ty`, `tz` — torque components in **Newton-metres** in the sensor frame

  Values are taken from the `ft_filtered` field of the real-time report (bytes
  87–110); the arm applies its internal filter before pushing these values.
  """

  defstruct [:fx, :fy, :fz, :tx, :ty, :tz]

  use BB.Message,
    schema: [
      fx: [type: :float, required: true, doc: "Force X in N"],
      fy: [type: :float, required: true, doc: "Force Y in N"],
      fz: [type: :float, required: true, doc: "Force Z in N"],
      tx: [type: :float, required: true, doc: "Torque X in Nm"],
      ty: [type: :float, required: true, doc: "Torque Y in Nm"],
      tz: [type: :float, required: true, doc: "Torque Z in Nm"]
    ]

  @type t :: %__MODULE__{
          fx: float(),
          fy: float(),
          fz: float(),
          tx: float(),
          ty: float(),
          tz: float()
        }
end
