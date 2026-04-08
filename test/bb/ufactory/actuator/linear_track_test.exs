# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Actuator.LinearTrackTest do
  use ExUnit.Case, async: false
  use Mimic

  setup :verify_on_exit!

  alias BB.Message
  alias BB.Message.Actuator.BeginMotion
  alias BB.Message.Actuator.Command
  alias BB.StateMachine.Transition
  alias BB.Ufactory.Actuator.LinearTrack
  alias BB.Ufactory.Protocol

  defp make_state(opts \\ []) do
    %{
      bb: %{robot: TestRobot, path: [:linear_track]},
      controller: :xarm,
      speed: Keyword.get(opts, :speed, 200)
    }
  end

  defp position_msg(pos_mm) do
    Message.new!(Command.Position, :linear_track, position: pos_mm * 1.0)
  end

  # ── Protocol encoding ────────────────────────────────────────────────────────

  describe "Protocol.cmd_linear_track_move/3 encoding" do
    test "returns a tuple of {pos_frame, spd_frame}" do
      {pos_frame, spd_frame} = Protocol.cmd_linear_track_move(1, 500.0, 200)
      assert is_binary(pos_frame)
      assert is_binary(spd_frame)
    end

    test "encodes position as round(mm * 2000) in a big-endian int32" do
      pos_mm = 100.0
      expected_units = round(pos_mm * 2000)
      {pos_frame, _spd_frame} = Protocol.cmd_linear_track_move(1, pos_mm, 200)

      # Extract the RS485 payload from the frame (skip 6-byte header + 1-byte register)
      # Then skip the RS485 write header (8 bytes: host_id, dev_id, 0x10, reg_addr(2), count(2), byte_count)
      <<_header::binary-size(7), rs485_payload::binary>> = pos_frame
      # RS485 write frame: 8 header bytes + 4 data bytes for int32
      <<_rs485_header::binary-size(8), pos_bytes::binary-size(4)>> = rs485_payload
      <<actual_units::signed-32>> = pos_bytes
      assert actual_units == expected_units
    end

    test "encodes speed as round(mm_s * 6.667) unsigned 16-bit in the speed frame" do
      speed = 350
      {_pos_frame, spd_frame} = Protocol.cmd_linear_track_move(1, 500.0, speed)

      <<_header::binary-size(7), rs485_payload::binary>> = spd_frame
      <<_rs485_header::binary-size(8), spd_bytes::binary-size(2)>> = rs485_payload
      <<actual_speed::unsigned-16>> = spd_bytes
      assert actual_speed == round(speed * 6.667)
    end

    test "handles zero position correctly" do
      {pos_frame, _} = Protocol.cmd_linear_track_move(0, 0.0, 200)
      <<_header::binary-size(7), rs485_payload::binary>> = pos_frame
      <<_rs485_header::binary-size(8), pos_bytes::binary-size(4)>> = rs485_payload
      <<actual_units::signed-32>> = pos_bytes
      assert actual_units == 0
    end
  end

  # ── init/1 ───────────────────────────────────────────────────────────────────

  describe "init/1" do
    test "stores bb, controller, and speed in state" do
      BB
      |> stub(:subscribe, fn TestRobot, [:state_machine] -> :ok end)

      opts = [
        bb: %{robot: TestRobot, path: [:linear_track]},
        controller: :xarm,
        speed: 300
      ]

      assert {:ok, state} = LinearTrack.init(opts)
      assert state.bb == %{robot: TestRobot, path: [:linear_track]}
      assert state.controller == :xarm
      assert state.speed == 300
    end

    test "defaults speed to 200 mm/s" do
      BB
      |> stub(:subscribe, fn TestRobot, [:state_machine] -> :ok end)

      opts = [bb: %{robot: TestRobot, path: [:linear_track]}, controller: :xarm]
      assert {:ok, state} = LinearTrack.init(opts)
      assert state.speed == 200
    end

    test "subscribes to state machine transitions instead of eagerly enabling" do
      BB
      |> expect(:subscribe, fn TestRobot, [:state_machine] -> :ok end)

      BB.Process
      |> reject(:call, 3)

      opts = [
        bb: %{robot: TestRobot, path: [:linear_track]},
        controller: :xarm
      ]

      assert {:ok, _state} = LinearTrack.init(opts)
    end
  end

  # ── handle_info(state machine :armed) ──────────────────────────────────────

  describe "handle_info(state machine transition to :armed)" do
    test "sends track enable command on armed transition" do
      state = make_state()
      expected_frame = Protocol.cmd_linear_track_enable(0, true)
      test_pid = self()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        send(test_pid, {:enable_sent, frame})
        :ok
      end)

      {:ok, msg} = Transition.new(:disarmed, from: :disarmed, to: :armed)
      bb_msg = {:bb, [:state_machine], msg}

      assert {:noreply, ^state} = LinearTrack.handle_info(bb_msg, state)
      assert_receive {:enable_sent, ^expected_frame}, 500
    end
  end

  # ── handle_cast position commands ────────────────────────────────────────────

  describe "handle_cast({:command, %Command.Position{}}, state)" do
    test "sends speed frame then position frame via sequential controller calls" do
      state = make_state()
      pos_mm = 500.0
      {expected_pos_frame, expected_spd_frame} = Protocol.cmd_linear_track_move(0, pos_mm, 200)

      test_pid = self()
      send_calls = :counters.new(1, [:atomics])

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, msg ->
        case msg do
          {:send_and_recv, _frame} ->
            {:ok, {0x7C, 0x00, <<0x0B, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00>>}, <<>>}

          {:send_command, frame} ->
            idx = :counters.get(send_calls, 1)
            :counters.add(send_calls, 1, 1)

            case idx do
              0 -> send(test_pid, {:frame_sent, :first, frame})
              1 -> send(test_pid, {:frame_sent, :second, frame})
            end

            :ok
        end
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(pos_mm)
      assert {:noreply, ^state} = LinearTrack.handle_cast({:command, msg}, state)

      assert_receive {:frame_sent, :first, first_frame}, 500
      assert_receive {:frame_sent, :second, second_frame}, 500
      assert first_frame == expected_spd_frame
      assert second_frame == expected_pos_frame
    end

    test "publishes BeginMotion with initial_position from track read and target_position in mm" do
      state = make_state()
      test_pid = self()

      initial_pos_raw = round(100.0 * 2000)

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, msg ->
        case msg do
          {:send_and_recv, _frame} ->
            {:ok, {0x7C, 0x00, <<0x0B, 0x01, 0x03, 0x04, initial_pos_raw::signed-32>>}, <<>>}

          {:send_command, _frame} ->
            :ok
        end
      end)

      BB
      |> expect(:publish, fn TestRobot,
                             [:actuator | _path],
                             %Message{
                               payload: %BeginMotion{} = bm
                             } ->
        send(test_pid, {:begin_motion, bm})
        :ok
      end)

      msg = position_msg(500.0)
      LinearTrack.handle_cast({:command, msg}, state)

      assert_receive {:begin_motion, bm}, 500
      assert_in_delta bm.target_position, 500.0, 0.001
      assert_in_delta bm.initial_position, 100.0, 0.001
    end

    test "does not publish BeginMotion when speed frame send fails" do
      state = make_state()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, msg ->
        case msg do
          {:send_and_recv, _frame} ->
            {:ok, {0x7C, 0x00, <<0x0B, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00>>}, <<>>}

          {:send_command, _frame} ->
            {:error, :closed}
        end
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg ->
        flunk("publish should not be called on send failure")
      end)

      msg = position_msg(500.0)
      assert {:noreply, ^state} = LinearTrack.handle_cast({:command, msg}, state)
    end

    test "ignores unknown casts" do
      state = make_state()
      assert {:noreply, ^state} = LinearTrack.handle_cast(:unexpected, state)
    end
  end

  # ── handle_info pubsub delivery ──────────────────────────────────────────────

  describe "handle_info({:bb, [:actuator | path], %Command.Position{}}, state)" do
    test "applies the same track move logic as handle_cast" do
      state = make_state()

      BB.Process
      |> stub(:call, fn TestRobot, :xarm, msg ->
        case msg do
          {:send_and_recv, _frame} ->
            {:ok, {0x7C, 0x00, <<0x0B, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00>>}, <<>>}

          {:send_command, _frame} ->
            :ok
        end
      end)

      BB
      |> stub(:publish, fn _robot, _path, _msg -> :ok end)

      msg = position_msg(200.0)

      assert {:noreply, ^state} =
               LinearTrack.handle_info({:bb, [:actuator, :linear_track], msg}, state)
    end

    test "ignores unrecognised messages" do
      state = make_state()
      assert {:noreply, ^state} = LinearTrack.handle_info(:unexpected, state)
    end
  end

  # ── disarm/1 ────────────────────────────────────────────────────────────────

  describe "disarm/1" do
    test "sends track disable command via controller" do
      expected_frame = Protocol.cmd_linear_track_enable(0, false)
      test_pid = self()

      BB.Process
      |> expect(:call, fn TestRobot, :xarm, {:send_command, frame} ->
        send(test_pid, {:disable_sent, frame})
        :ok
      end)

      assert :ok =
               LinearTrack.disarm(
                 bb: %{robot: TestRobot, path: [:linear_track]},
                 controller: :xarm
               )

      assert_receive {:disable_sent, ^expected_frame}, 500
    end

    test "returns :ok when controller call fails" do
      BB.Process
      |> stub(:call, fn _robot, _ctrl, _msg -> {:error, :closed} end)

      assert :ok =
               LinearTrack.disarm(
                 bb: %{robot: TestRobot, path: [:linear_track]},
                 controller: :xarm
               )
    end
  end
end
