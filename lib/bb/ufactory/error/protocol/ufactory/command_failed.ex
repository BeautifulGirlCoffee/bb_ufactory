# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Error.Protocol.Ufactory.CommandFailed do
  @moduledoc "A UFactory operation could not be completed or confirmed."

  use BB.Error, class: :protocol, fields: [:operation, :reason]

  defimpl BB.Error.Severity do
    def severity(_), do: :error
  end

  @doc false
  @spec reply(atom(), term()) :: {:error, term()}
  def reply(_operation, %{__exception__: true} = reason), do: {:error, reason}

  def reply(operation, reason) do
    {:error, exception(operation: operation, reason: reason)}
  end

  @impl true
  def message(%{operation: operation, reason: reason}) do
    "UFactory #{operation} failed: #{inspect(reason)}"
  end
end
