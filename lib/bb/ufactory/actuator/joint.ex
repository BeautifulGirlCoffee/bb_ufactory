# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.Joint do
  @moduledoc """
  Joint-space position actuator for xArm joints.

  One instance runs per joint. On receiving a `%BB.Message.Actuator.Command.Position{}`
  command, the actuator clamps the angle to joint limits and writes the target
  `set_position` into the controller's ETS table. The controller's 100Hz loop reads
  all pending `set_position` values and batches them into a single `cmd_move_joints`
  frame.

  > #### Phase stub {: .warning}
  >
  > This is a Phase 1 stub. ETS writes, joint-limit clamping, and `BeginMotion`
  > publishing are not yet implemented. Full implementation is Phase 4.
  """

  use BB.Actuator,
    options_schema: [
      joint: [
        type: {:in, 1..7},
        required: true,
        doc: "1-based joint index (1 = base joint)"
      ],
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller in the robot's registry"
      ]
    ]

  @impl BB.Actuator
  def init(_opts) do
    {:ok, %{}}
  end

  @impl BB.Actuator
  def disarm(_opts), do: :ok
end
