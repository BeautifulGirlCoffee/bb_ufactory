# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Sensor.CartesianPose do
  @moduledoc """
  Exposes the controller's TCP pose as a declared BB sensor source.

  The sensor forwards the original message to its own PubSub path. The
  controller's frame ID and timestamps survive forwarding so BB estimators
  can align it with joint and wrench reports.
  """

  use BB.Sensor,
    options_schema: [
      controller: [
        type: :atom,
        required: true,
        doc: "Name of the xArm controller that publishes the TCP pose"
      ]
    ]

  alias BB.Message
  alias BB.Ufactory.Message.CartesianPose

  @impl BB.Sensor
  def init(opts) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    source = [:sensor, controller, :tcp_pose]
    BB.subscribe(bb.robot, source)
    {:ok, %{bb: bb, source: source}}
  end

  @impl BB.Sensor
  def handle_info(
        {:bb, source, %Message{payload: %CartesianPose{}} = message},
        %{source: source} = state
      ) do
    BB.publish(state.bb.robot, [:sensor | state.bb.path], message)
    {:noreply, state}
  end

  def handle_info(_message, state), do: {:noreply, state}
end
