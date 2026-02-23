# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Ufactory.HardwareFault do
  @moduledoc """
  Structured error for xArm controller hardware faults.

  Raised when the real-time report frame contains a non-zero `error_code`. The
  controller calls `BB.Safety.report_error/3` with this exception, which triggers
  automatic disarm (unless `auto_disarm_on_error: false` is set on the robot).

  ## Error Code Table

  Error codes come from the `ControllerErrorCodeMap` in the xArm Python SDK
  (`xarm/core/config/x_code.py`). All codes are severity `:critical` — any
  non-zero error code stops the arm.

  | Code | Title |
  |------|-------|
  | 1 | Emergency Stop Button on controller pressed |
  | 2 | Emergency IO of Control Box triggered |
  | 3 | Emergency Stop Button of Three-state Switch pressed |
  | 10 | Servo motor error |
  | 11–17 | Servo motor N error |
  | 18 | Force Torque Sensor Communication Error |
  | 19 | End Effector Communication Error |
  | 21 | Kinematic Error |
  | 22 | Self-Collision Error |
  | 23 | Joints Angle Exceed Limit |
  | 24 | Speed Exceeds Limit |
  | 25 | Planning Error |
  | 26 | Linux RT Error |
  | 27 | Command Reply Error |
  | 28 | End Module Communication Error |
  | 30 | Feedback Speed Exceeds Limit |
  | 31 | Collision Caused Abnormal Current |
  | 32 | Three-point Drawing Circle Calculation Error |
  | 33 | Controller GPIO Error |
  | 34 | Recording Timeout |
  | 35 | Safety Boundary Limit |
  | 36 | Number of Delay Commands Exceeds Limit |
  | 37 | Abnormal Movement in Manual Mode |
  | 38 | Abnormal Joint Angle |
  | 39 | Abnormal Communication Between Master and Slave IC of Power Board |
  | 40 | Solution Failure of Error-free Joint Trajectory |
  | 41 | Friction File Content Invalid |
  | 42 | Calibration File Content Invalid |
  | 50 | Six-axis Force Torque Sensor Read Error |
  | 51 | Six-axis Force Torque Sensor Set Mode Error |
  | 52 | Six-axis Force Torque Sensor Set Zero Error |
  | 53 | Six-axis Force Torque Sensor Overloaded or Reading Exceeds Limit |
  | 60 | Linear Speed Exceeded Limit in servo_j Mode |
  | 110 | Robot Arm Base Board Communication Error |
  | 111 | Control Box External 485 Device Communication Error |

  Use `describe/1` to look up a human-readable title for a given error code.
  """

  use BB.Error, class: :protocol, fields: [:error_code, :description]

  defimpl BB.Error.Severity do
    def severity(_), do: :critical
  end

  @error_descriptions %{
    1 => "Emergency Stop Button on controller pressed",
    2 => "Emergency IO of Control Box triggered",
    3 => "Emergency Stop Button of Three-state Switch pressed",
    10 => "Servo motor error",
    11 => "Servo motor 1 error",
    12 => "Servo motor 2 error",
    13 => "Servo motor 3 error",
    14 => "Servo motor 4 error",
    15 => "Servo motor 5 error",
    16 => "Servo motor 6 error",
    17 => "Servo motor 7 error",
    18 => "Force Torque Sensor Communication Error",
    19 => "End Effector Communication Error",
    21 => "Kinematic Error",
    22 => "Self-Collision Error",
    23 => "Joints Angle Exceed Limit",
    24 => "Speed Exceeds Limit",
    25 => "Planning Error",
    26 => "Linux RT Error",
    27 => "Command Reply Error",
    28 => "End Module Communication Error",
    30 => "Feedback Speed Exceeds Limit",
    31 => "Collision Caused Abnormal Current",
    32 => "Three-point Drawing Circle Calculation Error",
    33 => "Controller GPIO Error",
    34 => "Recording Timeout",
    35 => "Safety Boundary Limit",
    36 => "Number of Delay Commands Exceeds Limit",
    37 => "Abnormal Movement in Manual Mode",
    38 => "Abnormal Joint Angle",
    39 => "Abnormal Communication Between Master and Slave IC of Power Board",
    40 => "Solution Failure of Error-free Joint Trajectory",
    41 => "Friction File Content Invalid",
    42 => "Calibration File Content Invalid",
    50 => "Six-axis Force Torque Sensor Read Error",
    51 => "Six-axis Force Torque Sensor Set Mode Error",
    52 => "Six-axis Force Torque Sensor Set Zero Error",
    53 => "Six-axis Force Torque Sensor Overloaded or Reading Exceeds Limit",
    60 => "Linear Speed Exceeded Limit in servo_j Mode",
    110 => "Robot Arm Base Board Communication Error",
    111 => "Control Box External 485 Device Communication Error"
  }

  @doc """
  Returns the human-readable description for a given xArm controller error code.

  Falls back to `"Unknown xArm error code <n>"` for codes not in the table.
  """
  @spec describe(non_neg_integer()) :: String.t()
  def describe(error_code) when is_integer(error_code) do
    Map.get(@error_descriptions, error_code, "Unknown xArm error code #{error_code}")
  end

  @impl true
  def message(%{error_code: error_code, description: description})
      when is_binary(description) and description != "" do
    "xArm hardware fault (code #{error_code}): #{description}"
  end

  def message(%{error_code: error_code}) do
    "xArm hardware fault (code #{error_code}): #{describe(error_code)}"
  end
end
