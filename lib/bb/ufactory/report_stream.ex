# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.ReportStream do
  @moduledoc false

  # Timestamp at socket delivery, independently of the controller's mailbox.
  # The socket stays passive until ownership has moved to the reader.
  @doc "Opens a report reader that forwards timestamped TCP chunks to its caller."
  @spec connect(charlist(), :inet.port_number(), timeout()) ::
          {:ok, :gen_tcp.socket()} | {:error, term()}
  def connect(host, port, timeout) do
    with {:ok, socket} <- :gen_tcp.connect(host, port, [:binary, active: false], timeout) do
      controller = self()
      reader = spawn_link(fn -> await_socket(controller, socket) end)

      case :gen_tcp.controlling_process(socket, reader) do
        :ok ->
          send(reader, :activate)
          {:ok, socket}

        {:error, _} = error ->
          send(reader, :cancel)
          :gen_tcp.close(socket)
          error
      end
    end
  end

  defp await_socket(controller, socket) do
    monitor = Process.monitor(controller)

    receive do
      :cancel ->
        :ok

      :activate ->
        case :inet.setopts(socket, active: true) do
          :ok -> forward(controller, monitor, socket, :inet.monitor(socket))
          {:error, reason} -> send(controller, {:tcp_error, socket, reason})
        end

      {:DOWN, ^monitor, :process, _, _} ->
        :ok
    end
  end

  defp forward(controller, monitor, socket, socket_monitor) do
    receive do
      {:tcp, ^socket, data} ->
        times = {System.monotonic_time(:nanosecond), System.system_time(:nanosecond)}
        send(controller, {:report_bytes, socket, data, times})
        forward(controller, monitor, socket, socket_monitor)

      {:tcp_closed, ^socket} = closed ->
        send(controller, closed)

      {:tcp_error, ^socket, _reason} = error ->
        send(controller, error)

      {:DOWN, ^socket_monitor, _, _, _} ->
        send(controller, {:tcp_closed, socket})

      {:DOWN, ^monitor, :process, _, _} ->
        :ok
    end
  end
end
