# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Sensor.ForceTorque do
  @moduledoc """
  Force-torque sensor for xArm arms (register 0xC8 / 0xC9).

  Enables the xArm's built-in F/T sensor on `init/1` and subscribes to
  wrench messages published by `BB.Ufactory.Controller`. The controller
  extracts `ft_filtered` data from the 135+ byte real-time report frames
  (port 30003) and publishes a `BB.Ufactory.Message.Wrench` to
  `[:sensor, controller_name, :wrench]` whenever valid data arrives.

  This sensor forwards those messages to its own pubsub path
  (`[:sensor | sensor_path]`) so application code can subscribe to a
  stable, sensor-scoped topic without knowing the controller name.

  ## Data Availability

  `ft_filtered` data is only present in real-time report frames when the
  F/T sensor has been explicitly enabled. Enabling is done automatically
  in `init/1` via `cmd_ft_sensor_enable(true)`. Until the first frame with
  F/T data arrives, no `BB.Ufactory.Message.Wrench` messages are published.

  ## Disarm

  On disarm, the sensor attempts to send `cmd_ft_sensor_enable(false)` via
  the controller process. This is best-effort: if the controller is already
  down, the attempt is silently ignored and `:ok` is returned.

  ## Configuration

      sensor :wrench, {BB.Ufactory.Sensor.ForceTorque,
        controller: :xarm
      }
  """

  use BB.Sensor,
    options_schema: [
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller in the robot's registry"
      ],
      poll_interval_ms: [
        type: :pos_integer,
        default: 20,
        doc: "Kept for API compatibility; data rate is driven by the controller's report socket"
      ]
    ]

  require Logger

  alias BB.Ufactory.Message.Wrench
  alias BB.Ufactory.Protocol

  # ── init/1 ──────────────────────────────────────────────────────────────────

  @impl BB.Sensor
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)

    enable_frame = Protocol.cmd_ft_sensor_enable(0, true)

    case BB.Process.call(bb.robot, controller, {:send_command, enable_frame}) do
      :ok ->
        :ok

      {:error, reason} ->
        Logger.warning(
          "[BB.Ufactory.Sensor.ForceTorque] ft_sensor_enable(true) failed: #{inspect(reason)}"
        )
    end

    BB.subscribe(bb.robot, [:sensor, controller, :wrench])

    {:ok, %{bb: bb, controller: controller}}
  end

  # ── disarm/1 — disable F/T sensor hardware ──────────────────────────────────

  @impl BB.Sensor
  def disarm(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    frame = Protocol.cmd_ft_sensor_enable(0, false)

    try do
      BB.Process.call(bb.robot, controller, {:send_command, frame})
    catch
      _, _ -> :ok
    end

    :ok
  end

  # ── handle_info — forward Wrench from controller pubsub ─────────────────────

  @impl BB.Sensor
  def handle_info({:bb, [:sensor, _, :wrench], %BB.Message{payload: %Wrench{}} = msg}, state) do
    BB.publish(state.bb.robot, [:sensor | state.bb.path], msg)
    {:noreply, state}
  end

  def handle_info(_msg, state), do: {:noreply, state}
end
