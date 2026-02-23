# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Message.ArmStatus do
  @moduledoc """
  Arm state and mode snapshot from the xArm real-time report.

  Published by `BB.Ufactory.Controller` to `[:sensor, controller_name, :arm_status]`
  whenever the arm's `state`, `mode`, `error_code`, or `warn_code` changes between
  consecutive report frames.

  ## Fields

  - `state` — arm motion state integer:
    - `0` — idle
    - `1` — sleeping
    - `2` — suspended
    - `3` — stopping
    - `4` — transitioning
    - `5` — moving

  - `mode` — control mode integer:
    - `0` — position (normal operation)
    - `1` — servo (real-time servo mode)
    - `2` — joint teaching (manual drag)
    - `3` — Cartesian teaching (manual drag in Cartesian space)

  - `error_code` — hardware/controller error code (`0` = no error). See
    `BB.Error.Protocol.Ufactory.HardwareFault` for the full code table.

  - `warn_code` — controller warning code (`0` = no warning).

  State is taken from the lower nibble of byte 4 of the real-time report frame;
  mode from the upper nibble.
  """

  defstruct [:state, :mode, :error_code, :warn_code]

  use BB.Message,
    schema: [
      state: [type: :non_neg_integer, required: true, doc: "Arm motion state (0–5)"],
      mode: [type: :non_neg_integer, required: true, doc: "Control mode (0–3)"],
      error_code: [type: :non_neg_integer, required: true, doc: "Hardware error code (0 = none)"],
      warn_code: [
        type: :non_neg_integer,
        required: true,
        doc: "Controller warning code (0 = none)"
      ]
    ]

  @type t :: %__MODULE__{
          state: non_neg_integer(),
          mode: non_neg_integer(),
          error_code: non_neg_integer(),
          warn_code: non_neg_integer()
        }
end
