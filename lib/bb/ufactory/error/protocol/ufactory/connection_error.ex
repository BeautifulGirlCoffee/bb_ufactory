# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Ufactory.ConnectionError do
  @moduledoc """
  Raised when the controller fails to establish a TCP connection to the xArm.

  The controller attempts two connections during `init/1`:

  - **Port 502** — command socket (send register commands, receive responses)
  - **Port 30003** — real-time report socket (arm pushes joint state at ~100Hz)

  If either connection fails, the controller stops with a `ConnectionError`
  containing the target host, port, and the POSIX reason from `:gen_tcp.connect/3`.

  Common `:reason` values from `:gen_tcp`:
  - `:econnrefused` — arm not listening (wrong IP or powered off)
  - `:etimedout` — network unreachable or firewall dropping packets
  - `:nxdomain` — hostname not resolvable
  """

  use BB.Error, class: :protocol, fields: [:host, :port, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  @impl true
  def message(%{host: host, port: port, reason: reason}) do
    "TCP connection to #{host}:#{port} failed: #{inspect(reason)}"
  end
end
