# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Controller do
  @moduledoc """
  Controller for UFactory xArm robotic arms.

  Manages two TCP connections to the arm:

  - **Port 502** — command socket: sends register frames, receives responses, sends heartbeat
  - **Port 30003** — real-time report socket: receives arm-pushed joint state at ~100Hz

  An ETS table holds the shared state between the controller and actuators. The
  100Hz control loop reads `set_position` values written by actuators and batches
  them into a single `cmd_move_joints` frame per tick.

  > #### Phase stub {: .warning}
  >
  > This is a Phase 1 stub. The TCP sockets, ETS table, and control loop are not
  > yet implemented. Full implementation is Phase 3.
  """

  use BB.Controller,
    options_schema: [
      host: [
        type: :string,
        required: true,
        doc: "IP address or hostname of the xArm controller"
      ],
      port: [
        type: :pos_integer,
        default: 502,
        doc: "Command socket port (default: 502)"
      ],
      report_port: [
        type: :pos_integer,
        default: 30_003,
        doc: "Real-time report socket port (default: 30003)"
      ],
      model: [
        type: {:in, [:xarm5, :xarm6, :xarm7, :lite6, :xarm850]},
        default: :xarm6,
        doc: "xArm model variant"
      ],
      loop_hz: [
        type: :pos_integer,
        default: 100,
        doc: "Control loop frequency in Hz (default: 100)"
      ],
      heartbeat_interval_ms: [
        type: :pos_integer,
        default: 1000,
        doc: "Heartbeat interval in milliseconds (default: 1000)"
      ],
      disarm_action: [
        type: {:in, [:stop, :hold]},
        default: :stop,
        doc: "Action taken when robot is disarmed: :stop clears motion, :hold holds position"
      ]
    ]

  @impl BB.Controller
  def init(_opts) do
    {:ok, %{}}
  end
end
