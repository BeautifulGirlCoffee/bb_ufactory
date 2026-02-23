# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Sensor.Collision do
  @moduledoc """
  Collision detection sensor for xArm arms.

  Configures the arm's firmware collision detection on `init/1` and subscribes
  to `BB.Ufactory.Message.ArmStatus` events from the controller. When an
  `ArmStatus` message carrying a collision-related error code arrives, it is
  re-published to this sensor's own pubsub path so application code can
  subscribe to a stable, sensor-scoped topic.

  ## Detected Error Codes

  | Code | Meaning |
  |------|---------|
  | 22 | Self-collision error (arm would collide with itself) |
  | 31 | Collision caused abnormal current (external contact) |
  | 35 | Safety boundary limit (TCP exited the workspace fence) |

  ## Configuration

      sensor :collision, {BB.Ufactory.Sensor.Collision,
        controller: :xarm,
        sensitivity: 3,
        rebound: false,
        self_collision_check: true
      }

  ## Sensitivity Scale

  | Level | Behaviour |
  |-------|-----------|
  | `0` | Collision detection disabled |
  | `1` | Lowest sensitivity (hardest to trigger) |
  | `3` | Balanced — recommended default |
  | `5` | Highest sensitivity (easiest to trigger) |

  ## Subscribing to Collision Events

  When a collision is detected, this sensor publishes the originating
  `BB.Ufactory.Message.ArmStatus` message (with non-zero `error_code`) to
  `[:sensor | sensor_path]`:

      BB.subscribe(MyRobot, [:sensor, :collision])

      receive do
        {:bb, [:sensor, :collision], %BB.Message{payload: %BB.Ufactory.Message.ArmStatus{
          error_code: code
        }}} ->
          IO.puts("Collision detected, error_code: \#{code}")
      end

  ## Disarm

  Collision detection settings are persistent in the arm's firmware NVRAM.
  No hardware action is taken on disarm.
  """

  use BB.Sensor,
    options_schema: [
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller in the robot's registry"
      ],
      sensitivity: [
        type: :any,
        default: nil,
        doc:
          "Collision detection sensitivity level (0–5). `nil` leaves the arm's current setting " <>
            "unchanged. 0 = disabled, 5 = most sensitive. Register 0x25, 1× u8."
      ],
      rebound: [
        type: :any,
        default: nil,
        doc:
          "Whether the arm briefly reverses direction after a collision. `nil` leaves the " <>
            "arm's current setting unchanged. Register 0x3C, 1× u8."
      ],
      self_collision_check: [
        type: :any,
        default: nil,
        doc:
          "Enable the firmware's geometric self-collision model. `nil` leaves the arm's current " <>
            "setting unchanged. Register 0x4D, 1× u8."
      ]
    ]

  require Logger

  alias BB.Ufactory.Message.ArmStatus
  alias BB.Ufactory.Protocol

  # Error codes that indicate a collision or boundary violation.
  @collision_codes [22, 31, 35]

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Sensor
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    sensitivity = Keyword.get(opts, :sensitivity)
    rebound = Keyword.get(opts, :rebound)
    self_collision_check = Keyword.get(opts, :self_collision_check)

    maybe_send_config(bb, controller, sensitivity, rebound, self_collision_check)

    BB.subscribe(bb.robot, [:sensor, controller, :arm_status])

    {:ok, %{bb: bb, controller: controller}}
  end

  # ── disarm/1 ─────────────────────────────────────────────────────────────────

  @doc """
  No-op. Collision detection settings are persisted in firmware NVRAM and do
  not need to be reset on disarm.
  """
  @impl BB.Sensor
  def disarm(_opts), do: :ok

  # ── handle_info — ArmStatus collision forwarding ─────────────────────────────

  @impl BB.Sensor
  def handle_info(
        {:bb, [:sensor, _, :arm_status],
         %BB.Message{payload: %ArmStatus{error_code: code}} = msg},
        state
      )
      when code in @collision_codes do
    BB.publish(state.bb.robot, [:sensor | state.bb.path], msg)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}

  # ── Private helpers ───────────────────────────────────────────────────────────

  defp maybe_send_config(bb, controller, sensitivity, rebound, self_collision_check) do
    frames =
      [
        sensitivity && Protocol.cmd_set_collision_sensitivity(0, sensitivity),
        rebound != nil && Protocol.cmd_set_collision_rebound(0, rebound),
        self_collision_check != nil &&
          Protocol.cmd_set_self_collision_check(0, self_collision_check)
      ]
      |> Enum.filter(& &1)

    for frame <- frames do
      case BB.Process.call(bb.robot, controller, {:send_command, frame}) do
        :ok ->
          :ok

        {:error, reason} ->
          Logger.warning(
            "[BB.Ufactory.Sensor.Collision] Config send failed for #{inspect(controller)}: #{inspect(reason)}"
          )
      end
    end
  end
end
