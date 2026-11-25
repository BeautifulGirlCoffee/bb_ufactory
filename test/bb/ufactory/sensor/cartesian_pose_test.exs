# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Sensor.CartesianPoseTest do
  use ExUnit.Case, async: false
  use Mimic

  alias BB.Ufactory.Message.CartesianPose
  alias BB.Ufactory.Sensor.CartesianPose, as: PoseSensor

  setup :verify_on_exit!

  test "forwards the original envelope from the configured controller" do
    expect(BB, :subscribe, fn TestRobot, [:sensor, :xarm, :tcp_pose] -> {:ok, self()} end)

    assert {:ok, state} =
             PoseSensor.init(bb: %{robot: TestRobot, path: [:tool_pose]}, controller: :xarm)

    message =
      BB.Message.new!(CartesianPose, :xarm,
        x: 1.0,
        y: 2.0,
        z: 3.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0
      )

    expect(BB, :publish, fn TestRobot, [:sensor, :tool_pose], ^message -> :ok end)

    assert {:noreply, ^state} =
             PoseSensor.handle_info({:bb, [:sensor, :other_arm, :tcp_pose], message}, state)

    assert {:noreply, ^state} =
             PoseSensor.handle_info({:bb, [:sensor, :xarm, :tcp_pose], message}, state)
  end
end
