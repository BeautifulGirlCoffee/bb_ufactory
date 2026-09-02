# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Message.Command.CartesianMove do
  @moduledoc """
  Command payload for a Cartesian linear move (`MOVE_LINE`, register 0x15).

  BB's built-in `BB.Message.Actuator.Command.Position` holds a single scalar,
  which cannot express a 6-DOF end-effector pose. This payload carries the
  full pose and is declared by `BB.Ufactory.Actuator.Cartesian` via
  `c:BB.Actuator.command_payloads/1`, so it travels through the same gated
  command pipeline as any built-in command — the robot must be armed, and
  refusals reach a synchronous caller.

  ## Fields

  - `x`, `y`, `z` — target TCP position in **millimetres** (arm base frame)
  - `roll`, `pitch`, `yaw` — target orientation in **radians** (RPY)
  - `speed` — optional TCP linear speed in mm/s; the actuator's configured
    default applies when `nil`
  - `acceleration` — optional TCP linear acceleration in mm/s²; the actuator's
    configured default applies when `nil`
  - `command_id` — optional correlation reference for feedback tracking

  ## Examples

      alias BB.Message
      alias BB.Ufactory.Message.Command.CartesianMove

      {:ok, msg} = Message.new(CartesianMove, :tcp,
        x: 300.0, y: 0.0, z: 400.0,
        roll: 3.14159, pitch: 0.0, yaw: 0.0
      )

      # Deliver synchronously and learn whether it was accepted:
      BB.call(MyRobot, :tcp, {:command, msg})
  """

  defstruct [:x, :y, :z, :roll, :pitch, :yaw, :speed, :acceleration, :command_id]

  use BB.Message,
    schema: [
      x: [type: :float, required: true, doc: "Target X position in mm"],
      y: [type: :float, required: true, doc: "Target Y position in mm"],
      z: [type: :float, required: true, doc: "Target Z position in mm"],
      roll: [type: :float, required: true, doc: "Target roll in radians"],
      pitch: [type: :float, required: true, doc: "Target pitch in radians"],
      yaw: [type: :float, required: true, doc: "Target yaw in radians"],
      speed: [
        type: {:or, [nil, :float]},
        required: false,
        doc: "TCP linear speed in mm/s (actuator default when nil)"
      ],
      acceleration: [
        type: {:or, [nil, :float]},
        required: false,
        doc: "TCP linear acceleration in mm/s² (actuator default when nil)"
      ],
      command_id: [
        type: {:or, [nil, :reference]},
        required: false,
        doc: "Correlation ID for feedback"
      ]
    ]

  @type t :: %__MODULE__{
          x: float(),
          y: float(),
          z: float(),
          roll: float(),
          pitch: float(),
          yaw: float(),
          speed: float() | nil,
          acceleration: float() | nil,
          command_id: reference() | nil
        }
end
