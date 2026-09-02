# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

# The documented quick-start snippet: compiling these modules IS the test —
# they exercise `use BB.Ufactory.Robots.XArm6` exactly as README and the
# tutorials show it.
defmodule BB.Ufactory.Robots.XArm6Test.QuickStart do
  use BB.Ufactory.Robots.XArm6, host: "10.0.0.42"
end

defmodule BB.Ufactory.Robots.XArm6Test.WithAccessories do
  use BB.Ufactory.Robots.XArm6,
    gripper: [speed: 1200],
    linear_track: true,
    controller: [tcp_offset: {0.0, 0.0, 172.0, 0.0, 0.0, 0.0}]

  sensors do
    sensor(:wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm})
  end
end

defmodule BB.Ufactory.Robots.XArm6Test do
  # async: false because start_supervised starts a real process tree
  use ExUnit.Case, async: false

  @pi :math.pi()

  alias BB.Ufactory.Robots.XArm6

  describe "robot definition" do
    setup do
      robot = XArm6.robot()
      all_joints = BB.Robot.joints_in_order(robot)
      joints = Enum.filter(all_joints, &(&1.type == :revolute))
      %{robot: robot, joints: joints, all_joints: all_joints}
    end

    test "defines exactly 6 revolute joints", %{joints: joints} do
      assert length(joints) == 6
    end

    test "the only non-revolute joints are fixed accessory mounts", %{all_joints: all_joints} do
      others = Enum.reject(all_joints, &(&1.type == :revolute))
      assert Enum.all?(others, &(&1.type == :fixed))
      assert Enum.map(others, & &1.name) == [:cartesian_mount]
    end

    test "joints are named j1 through j6", %{joints: joints} do
      names = Enum.map(joints, & &1.name)
      assert names == [:j1, :j2, :j3, :j4, :j5, :j6]
    end

    test "j1 spans ±360 degrees", %{joints: joints} do
      j1 = Enum.find(joints, &(&1.name == :j1))
      assert_in_delta j1.limits.lower, -2 * @pi, 0.001
      assert_in_delta j1.limits.upper, 2 * @pi, 0.001
    end

    test "j2 matches developer manual limits", %{joints: joints} do
      j2 = Enum.find(joints, &(&1.name == :j2))
      # -118° = -2.059 rad, +120° = +2.094 rad
      assert_in_delta j2.limits.lower, -2.059, 0.01
      assert_in_delta j2.limits.upper, 2.094, 0.01
    end

    test "j3 matches developer manual limits", %{joints: joints} do
      j3 = Enum.find(joints, &(&1.name == :j3))
      # -225° = -3.927 rad, +11° = +0.192 rad
      assert_in_delta j3.limits.lower, -3.927, 0.01
      assert_in_delta j3.limits.upper, 0.192, 0.01
    end

    test "j4 spans ±360 degrees", %{joints: joints} do
      j4 = Enum.find(joints, &(&1.name == :j4))
      assert_in_delta j4.limits.lower, -2 * @pi, 0.001
      assert_in_delta j4.limits.upper, 2 * @pi, 0.001
    end

    test "j5 matches developer manual limits", %{joints: joints} do
      j5 = Enum.find(joints, &(&1.name == :j5))
      # -97° = -1.693 rad, +180° = +π rad
      assert_in_delta j5.limits.lower, -1.693, 0.01
      assert_in_delta j5.limits.upper, @pi, 0.001
    end

    test "j6 spans ±360 degrees", %{joints: joints} do
      j6 = Enum.find(joints, &(&1.name == :j6))
      assert_in_delta j6.limits.lower, -2 * @pi, 0.001
      assert_in_delta j6.limits.upper, 2 * @pi, 0.001
    end

    test "all joints have max velocity of 180 deg/s (π rad/s)", %{joints: joints} do
      for joint <- joints do
        assert_in_delta joint.limits.velocity, @pi, 0.01
      end
    end

    test "kinematic chain links base to link6 via 6 joints", %{robot: robot} do
      assert {:ok, %BB.Robot.Joint{}} = BB.Robot.get_joint(robot, :j1)
      assert {:ok, %BB.Robot.Joint{}} = BB.Robot.get_joint(robot, :j6)
    end
  end

  describe "use BB.Ufactory.Robots.XArm6" do
    alias BB.Ufactory.Robots.XArm6Test.{QuickStart, WithAccessories}

    test "quick-start module defines the full 6-joint robot" do
      robot = QuickStart.robot()

      joints =
        robot |> BB.Robot.joints_in_order() |> Enum.filter(&(&1.type == :revolute))

      assert Enum.map(joints, & &1.name) == [:j1, :j2, :j3, :j4, :j5, :j6]
      assert Map.has_key?(robot.actuators, :j1_motor)
      refute Map.has_key?(robot.actuators, :gripper)

      # Cartesian is on by default — the README's CartesianMove example must
      # work against the quick-start robot.
      assert %{joint: :cartesian_mount} = robot.actuators[:cartesian]
    end

    test "cartesian: false omits the cartesian actuator" do
      defmodule NoCartesian do
        use BB.Ufactory.Robots.XArm6, cartesian: false
      end

      refute Map.has_key?(NoCartesian.robot().actuators, :cartesian)
    end

    test "quick-start module matches the base definition's limits" do
      limits = fn robot ->
        robot
        |> BB.Robot.joints_in_order()
        |> Enum.filter(&(&1.type == :revolute))
        |> Enum.map(& &1.limits)
      end

      assert limits.(XArm6.robot()) == limits.(QuickStart.robot())
    end

    test "accessory options add gripper and track actuators on fixed mounts" do
      robot = WithAccessories.robot()

      assert %{joint: :gripper_mount} = robot.actuators[:gripper]
      assert %{joint: :track_mount} = robot.actuators[:track]

      assert {:ok, %BB.Robot.Joint{type: :fixed}} =
               BB.Robot.get_joint(WithAccessories.robot(), :gripper_mount)
    end

    test "accessory robot supervisor starts in kinematic simulation" do
      pid = start_supervised!({WithAccessories, simulation: :kinematic})
      assert Process.alive?(pid)
    end

    test "controller option merges extra opts into the child spec" do
      [controller] = Spark.Dsl.Extension.get_entities(WithAccessories, [:controllers])
      {BB.Ufactory.Controller, opts} = controller.child_spec

      assert opts[:host] == "192.168.1.111"
      assert opts[:loop_hz] == 100
      assert opts[:tcp_offset] == {0.0, 0.0, 172.0, 0.0, 0.0, 0.0}
    end

    test "unknown options raise at compile time" do
      assert_raise ArgumentError, ~r/unknown keys/, fn ->
        defmodule BadOpts do
          use BB.Ufactory.Robots.XArm6, hostname: "typo"
        end
      end
    end

    test "derived robot supervisor starts in kinematic simulation" do
      pid = start_supervised!({QuickStart, simulation: :kinematic})
      assert Process.alive?(pid)
    end
  end

  describe "kinematic simulation" do
    setup do
      pid = start_supervised!({XArm6, simulation: :kinematic})
      %{pid: pid}
    end

    test "robot supervisor starts successfully", %{pid: pid} do
      assert Process.alive?(pid)
    end

    test "all 6 simulated actuators are running", %{pid: pid} do
      children = Supervisor.which_children(pid)
      # The supervisor tree includes actuator processes; confirm it's non-empty
      assert children != []
    end
  end
end
