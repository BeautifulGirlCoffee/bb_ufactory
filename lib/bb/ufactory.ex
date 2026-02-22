# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory do
  @moduledoc """
  Beam Bots integration for UFactory xArm robotic arms.

  `BB.Ufactory` provides controller, actuator, and sensor modules for integrating
  UFactory xArm arms (xArm5, xArm6, xArm7, Lite6, xArm850) with the BB robotics
  framework.

  ## Communication

  The arm communicates over two TCP connections:

  - **Port 502** — command socket (send register commands, receive responses)
  - **Port 30003** — real-time report socket (arm pushes joint state at ~100Hz)

  The protocol uses UFactory's custom Modbus-TCP variant with big-endian headers and
  little-endian 32-bit float payloads. All wire angles are in radians.

  ## Usage

  ```elixir
  defmodule MyRobot do
    use BB

    controller :xarm, {BB.Ufactory.Controller,
      host: "192.168.1.111",
      model: :xarm6,
      loop_hz: 100
    }

    topology do
      link :base do
        joint :j1, type: :revolute do
          limit lower: ~u(-360 degree), upper: ~u(360 degree), velocity: ~u(180 degree_per_second)
          actuator :motor, {BB.Ufactory.Actuator.Joint, joint: 1, controller: :xarm}
        end
      end
    end

    sensors do
      sensor :wrench, {BB.Ufactory.Sensor.ForceTorque, controller: :xarm}
    end
  end
  ```
  """
end
