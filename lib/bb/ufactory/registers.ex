# SPDX-FileCopyrightText: 2026 Holden Oullette
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Ufactory.Registers do
  @moduledoc """
  Register address constants for the UFactory xArm Modbus-TCP protocol.

  All values are sourced from `UxbusReg` in the xArm Python SDK
  (`xarm/core/config/x_config.py`) and the xArm Developer Manual V1.10.0.

  Use these constants in `BB.Ufactory.Protocol` command builders rather than
  hard-coding raw integers.
  """

  # ── Primary registers (fully documented public API) ────────────────────────

  @doc "MOTION_EN — enable/disable joint motion; axis_id `0x08` = all joints"
  @spec motion_en() :: non_neg_integer()
  def motion_en, do: 0x0B

  @doc "SET_STATE — arm state; 0 = stop/clear motion, 3 = play, 4 = pause"
  @spec set_state() :: non_neg_integer()
  def set_state, do: 0x0C

  @doc "GET_STATE — read current arm state"
  @spec get_state() :: non_neg_integer()
  def get_state, do: 0x0D

  @doc "GET_CMDNUM — queued command count"
  @spec get_cmdnum() :: non_neg_integer()
  def get_cmdnum, do: 0x0E

  @doc "GET_ERROR — read error and warning codes"
  @spec get_error() :: non_neg_integer()
  def get_error, do: 0x0F

  @doc "CLEAN_ERR — clear the current error code"
  @spec clean_err() :: non_neg_integer()
  def clean_err, do: 0x10

  @doc "CLEAN_WAR — clear the current warning code"
  @spec clean_war() :: non_neg_integer()
  def clean_war, do: 0x11

  @doc "MOVE_LINE — Cartesian linear motion (register 0x15, decimal 21). NOT arc blending."
  @spec move_cart() :: non_neg_integer()
  def move_cart, do: 0x15

  @doc "MOVE_JOINT — joint-space P2P motion"
  @spec move_joints() :: non_neg_integer()
  def move_joints, do: 0x17

  @doc "MOVE_LINE_AA / arc blending — do not confuse with `move_cart/0`"
  @spec move_cart_arc() :: non_neg_integer()
  def move_cart_arc, do: 0x5C

  @doc "GET_TCP_POSE — read current Cartesian end-effector pose"
  @spec get_tcp_pose() :: non_neg_integer()
  def get_tcp_pose, do: 0x29

  @doc "GET_JOINTS — read current joint angles in radians"
  @spec get_joints() :: non_neg_integer()
  def get_joints, do: 0x2A

  @doc "RS485_RTU proxy — gripper, linear track, and other accessories"
  @spec rs485_rtu() :: non_neg_integer()
  def rs485_rtu, do: 0x7C

  @doc "FTSENSOR_GET_DATA — read 6× fp32 LE [Fx, Fy, Fz, Tx, Ty, Tz]"
  @spec ft_get_data() :: non_neg_integer()
  def ft_get_data, do: 0xC8

  @doc "FTSENSOR_ENABLE — 1× u8 (0 or 1)"
  @spec ft_enable() :: non_neg_integer()
  def ft_enable, do: 0xC9

  @doc "RELOAD_DYNAMICS — reload dynamics model"
  @spec reload_dynamics() :: non_neg_integer()
  def reload_dynamics, do: 0x04

  @doc "MOVE_HOME — move to home position"
  @spec move_home() :: non_neg_integer()
  def move_home, do: 0x19

  @doc "SET_MODE — arm mode; 0 = position mode, 1 = servo mode"
  @spec set_mode() :: non_neg_integer()
  def set_mode, do: 0x13

  @doc "GET_VERSION — firmware version string"
  @spec get_version() :: non_neg_integer()
  def get_version, do: 0x01

  @doc "GET_ROBOT_SN — robot serial number"
  @spec get_robot_sn() :: non_neg_integer()
  def get_robot_sn, do: 0x02

  # ── Secondary registers ────────────────────────────────────────────────────
  # These match the SDK register map (UxbusReg) and are exposed for completeness.
  # Use the primary functions above for the most common operations.

  for {name, value} <- [
        check_verify: 0x03,
        system_control: 0x0A,
        set_brake: 0x12,
        move_lineb: 0x16,
        move_jointb: 0x18,
        sleep_instt: 0x1A,
        move_circle: 0x1B,
        move_line_tool: 0x1C,
        move_servoj: 0x1D,
        move_servo_cart: 0x1E,
        set_tcp_jerk: 0x1F,
        set_tcp_maxacc: 0x20,
        set_joint_jerk: 0x21,
        set_joint_maxacc: 0x22,
        set_tcp_offset: 0x23,
        set_load_param: 0x24,
        set_collis_sens: 0x25,
        set_teach_sens: 0x26,
        clean_conf: 0x27,
        save_conf: 0x28,
        get_ik: 0x2B,
        get_fk: 0x2C,
        joint_limit_check: 0x2D,
        tcp_limit_check: 0x2E,
        set_reduced_trsv: 0x2F,
        set_reduced_p2pv: 0x30,
        get_reduced_mode: 0x31,
        set_reduced_mode: 0x32,
        set_gravity_dir: 0x33,
        set_limit_xyz: 0x34,
        get_reduced_state: 0x35,
        get_joint_tau: 0x37,
        set_safe_level: 0x38,
        get_safe_level: 0x39,
        set_reduced_jrange: 0x3A,
        set_fense_on: 0x3B,
        set_collis_reb: 0x3C,
        set_traj_record: 0x3D,
        save_traj: 0x3E,
        load_traj: 0x3F,
        play_traj: 0x40,
        get_traj_rw_status: 0x41,
        set_allow_approx_motion: 0x42,
        get_dh: 0x43,
        set_dh: 0x44,
        get_movement: 0x45,
        report_tau_or_i: 0x46,
        set_timer: 0x47,
        cancel_timer: 0x48,
        set_world_offset: 0x49,
        cnter_reset: 0x4A,
        cnter_plus: 0x4B,
        cal_pose_offset: 0x4C,
        set_self_collis_check: 0x4D,
        set_collis_tool: 0x4E,
        set_simulation_robot: 0x4F,
        set_cartv_continue: 0x50,
        vc_set_jointv: 0x51,
        vc_set_cartv: 0x52,
        move_relative: 0x53,
        get_tcp_pose_aa: 0x5B,
        move_servo_cart_aa: 0x5D,
        servo_w16b: 0x65,
        servo_r16b: 0x66,
        servo_w32b: 0x67,
        servo_r32b: 0x68,
        servo_zero: 0x69,
        servo_dbmsg: 0x6A,
        servo_error: 0x6B,
        cali_tcp_pose: 0x6F,
        cali_tcp_orient: 0x70,
        cali_wrld_orient: 0x71,
        cali_wrld_pose: 0x72,
        iden_fric: 0x73,
        tgpio_mb_tiout: 0x7B,
        tgpio_err: 0x7D,
        tgpio_w16b: 0x7F,
        tgpio_r16b: 0x80,
        tgpio_w32b: 0x81,
        tgpio_r32b: 0x82,
        cgpio_get_digit: 0x83,
        cgpio_get_analog1: 0x84,
        cgpio_get_analog2: 0x85,
        cgpio_set_digit: 0x86,
        cgpio_set_analog1: 0x87,
        cgpio_set_analog2: 0x88,
        cgpio_set_in_fun: 0x89,
        cgpio_set_out_fun: 0x8A,
        cgpio_get_state: 0x8B,
        get_pwr_version: 0x8C,
        get_hd_types: 0x8D,
        delayed_cgpio_set: 0x8E,
        delayed_tgpio_set: 0x8F,
        position_cgpio_set: 0x90,
        position_tgpio_set: 0x91,
        set_io_stop_reset: 0x92,
        position_cgpio_set_analog: 0x93,
        ft_get_data_old: 0x96,
        ft_set_mode: 0xCA,
        ft_get_mode: 0xCB,
        iden_load: 0xCC,
        ft_set_load_offset: 0xCD,
        ft_set_zero: 0xCE,
        admittance_config: 0xCF,
        force_ctrl_pid: 0xD0,
        force_ctrl_config: 0xD1,
        admittance_ctrl_mkb: 0xD2,
        admittance_ctrl_config: 0xD3,
        ft_get_config: 0xD4,
        get_traj_speeding: 0xE6,
        get_max_joint_velocity: 0xE7,
        set_common_param: 0xE8,
        get_common_param: 0xE9,
        get_common_info: 0xEA,
        tgpio_com_tiout: 0xF0,
        rs485_agent: 0xF1,
        feedback_check: 0xFD,
        set_feedback_type: 0xFE
      ] do
    @doc false
    @spec unquote(name)() :: non_neg_integer()
    def unquote(name)(), do: unquote(value)
  end
end
