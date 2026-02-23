# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ErrorTest do
  use ExUnit.Case, async: true

  alias BB.Error.Protocol.Ufactory.CommandRejected
  alias BB.Error.Protocol.Ufactory.ConnectionError
  alias BB.Error.Protocol.Ufactory.HardwareFault
  alias BB.Error.Severity

  # ── HardwareFault ──────────────────────────────────────────────────────────

  describe "HardwareFault.describe/1" do
    test "returns description for known error codes" do
      assert HardwareFault.describe(1) == "Emergency Stop Button on controller pressed"
      assert HardwareFault.describe(10) == "Servo motor error"
      assert HardwareFault.describe(22) == "Self-Collision Error"
      assert HardwareFault.describe(35) == "Safety Boundary Limit"
      assert HardwareFault.describe(110) == "Robot Arm Base Board Communication Error"
    end

    test "returns fallback string for unknown error code" do
      assert HardwareFault.describe(999) == "Unknown xArm error code 999"
      assert HardwareFault.describe(0) == "Unknown xArm error code 0"
    end

    test "returns correct servo motor descriptions for codes 11–17" do
      for n <- 11..17 do
        assert HardwareFault.describe(n) == "Servo motor #{n - 10} error"
      end
    end
  end

  describe "HardwareFault.message/1" do
    test "uses explicit description when provided" do
      error = HardwareFault.exception(error_code: 42, description: "custom fault info")
      assert Exception.message(error) == "xArm hardware fault (code 42): custom fault info"
    end

    test "falls back to describe/1 when description is nil" do
      error = HardwareFault.exception(error_code: 22)
      assert Exception.message(error) == "xArm hardware fault (code 22): Self-Collision Error"
    end

    test "falls back to describe/1 when description is empty string" do
      error = HardwareFault.exception(error_code: 1, description: "")
      msg = Exception.message(error)
      assert msg == "xArm hardware fault (code 1): Emergency Stop Button on controller pressed"
    end
  end

  describe "HardwareFault severity" do
    test "is always :critical" do
      error = HardwareFault.exception(error_code: 1)
      assert Severity.severity(error) == :critical
    end
  end

  # ── ConnectionError ────────────────────────────────────────────────────────

  describe "ConnectionError.message/1" do
    test "formats host, port, and reason" do
      error = ConnectionError.exception(host: "192.168.1.111", port: 502, reason: :econnrefused)

      assert Exception.message(error) ==
               "TCP connection to 192.168.1.111:502 failed: :econnrefused"
    end

    test "handles timeout reason" do
      error = ConnectionError.exception(host: "10.0.0.1", port: 30_003, reason: :etimedout)
      assert Exception.message(error) == "TCP connection to 10.0.0.1:30003 failed: :etimedout"
    end
  end

  describe "ConnectionError severity" do
    test "is :error" do
      error = ConnectionError.exception(host: "localhost", port: 502, reason: :econnrefused)
      assert Severity.severity(error) == :error
    end
  end

  # ── CommandRejected ────────────────────────────────────────────────────────

  describe "CommandRejected.message/1" do
    test "includes description when provided" do
      error =
        CommandRejected.exception(
          register: 0x0B,
          status_byte: 0x01,
          description: "Arm not ready"
        )

      msg = Exception.message(error)
      assert msg =~ "register 0xB"
      assert msg =~ "status 0x1"
      assert msg =~ "Arm not ready"
    end

    test "shows register and status hex without description" do
      error = CommandRejected.exception(register: 0x17, status_byte: 0x02)
      msg = Exception.message(error)
      assert msg =~ "register 0x17"
      assert msg =~ "status 0x2"
      refute msg =~ "nil"
    end

    test "shows register and status hex when description is empty" do
      error = CommandRejected.exception(register: 0x15, status_byte: 0x03, description: "")
      msg = Exception.message(error)
      assert msg =~ "register 0x15"
      assert msg =~ "status 0x3"
    end
  end

  describe "CommandRejected severity" do
    test "is :error" do
      error = CommandRejected.exception(register: 0x0B, status_byte: 0x01)
      assert Severity.severity(error) == :error
    end
  end
end
