# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Ufactory.CommandRejected do
  @moduledoc """
  Indicates a rejected command reported by the xArm.

  The UFactory Modbus-TCP response frame (port 502) has the format:

  ```
  [Transaction ID: u16 BE] [Protocol: u16 BE] [Length: u16 BE]
  [Register: u8] [Status: u8] [Params: binary]
  ```

  The status byte contains flags for errors, warnings, and readiness. The
  controller checks them when confirming a disarm response. See the xArm
  Developer Manual V1.10.0, section 2.1.5 for the status byte table.
  """

  use BB.Error, class: :protocol, fields: [:register, :status_byte, :description]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  @impl true
  def message(%{register: register, status_byte: status_byte, description: description})
      when is_binary(description) and description != "" do
    "Command to register 0x#{Integer.to_string(register, 16)} rejected " <>
      "with status 0x#{Integer.to_string(status_byte, 16)}: #{description}"
  end

  def message(%{register: register, status_byte: status_byte}) do
    "Command to register 0x#{Integer.to_string(register, 16)} rejected " <>
      "with status 0x#{Integer.to_string(status_byte, 16)}"
  end
end
