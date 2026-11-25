# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Accessory do
  @moduledoc false

  import Bitwise
  alias BB.Error.Protocol.Ufactory.CommandFailed

  # These entries belong to the accessory process, which survives a controller
  # restart. A new controller restores them before it handles an arm transition.
  @doc "Records arm frames under the accessory process and notifies its controller."
  @spec register(module(), atom(), atom(), [binary()]) :: :ok | {:error, term()}
  def register(robot, controller, label, frames) do
    registry = BB.Process.registry_name(robot)
    key = {__MODULE__, controller, label}

    with {:ok, _} <- Registry.register(registry, key, frames) do
      BB.Process.call(robot, controller, {:register_arm_frames, label, frames})
    end
  catch
    :exit, reason -> {:error, reason}
  end

  @doc "Returns the live accessories’ arm frames when a controller starts."
  @spec registrations(module(), atom()) :: [{atom(), [binary()]}]
  def registrations(robot, controller) do
    Registry.select(BB.Process.registry_name(robot), [
      {{{__MODULE__, controller, :"$1"}, :_, :"$2"}, [], [{{:"$1", :"$2"}}]}
    ])
  end

  @doc "Disables an RS485 accessory and verifies the device’s write acknowledgement."
  @spec disable(keyword(), binary()) :: :ok | {:error, term()}
  def disable(opts, frame) do
    bb = Keyword.fetch!(opts, :bb)
    controller = Keyword.fetch!(opts, :controller)
    <<_header::binary-7, expected_echo::binary-7, _::binary>> = frame

    case BB.Process.call(bb.robot, controller, {:send_and_recv, frame}) do
      {:ok, {0x7C, status, ^expected_echo}, _} when (status &&& 0x48) == 0 -> :ok
      {:error, reason} -> CommandFailed.reply(:accessory_disarm, reason)
      response -> CommandFailed.reply(:accessory_disarm, {:unconfirmed, response})
    end
  catch
    kind, reason -> CommandFailed.reply(:accessory_disarm, {kind, reason})
  end
end
