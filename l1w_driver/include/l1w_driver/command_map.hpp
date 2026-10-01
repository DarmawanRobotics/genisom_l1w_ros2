#pragma once

#include <string>
#include <unordered_map>

#include "zsibot_sdk/zsibot_define.h"

namespace l1w_driver
{

/// Service name -> SDK command code; each entry becomes a std_srvs/Trigger service.
inline const std::unordered_map<std::string, zsibot::CmdCode> & command_map()
{
  using C = zsibot::CmdCode;
  static const std::unordered_map<std::string, C> m = {
    {"emergency_stop", C::CMD_EMERGENCY_STOP},

    {"stand_up", C::CMD_STAND_UP},
    {"sit_down", C::CMD_SIT_DOWN},
    {"move_mode", C::CMD_MOVE_MODE},
    {"balance_stand_mode", C::CMD_BALANCE_STAND_MODE},
    {"lock_mode", C::CMD_LOCK_MODE},

    {"slow_speed", C::CMD_SLOW_SPEED},
    {"normal_speed", C::CMD_NORMAL_SPEED},
    {"fast_speed", C::CMD_FAST_SPEED},

    {"crawl_forward", C::CMD_CRAWL_FORWARD},
    {"climbing_high_platform", C::CMD_CLIMBING_HIGH_PLATFORM},
    {"hand_stand", C::CMD_HAND_STAND},
    {"unload_squat", C::CMD_UNLOAD_SQUAT},

    {"enter_lab_mode", C::CMD_ENTER_LAB_MODE},
    {"exit_lab_mode", C::CMD_EXIT_LAB_MODE},
    {"enter_entertainment_mode", C::CMD_ENTER_ENTERTAINMENT_MODE},
    {"exit_entertainment_mode", C::CMD_EXIT_ENTERTAINMENT_MODE},

    {"remote_control_right", C::CMD_REMOTE_CONTROL_RIGHT},
    {"sdk_control_right", C::CMD_SDK_CONTROL_RIGHT},
    {"roamerx_control_right", C::CMD_ROAMERX_CONTROL_RIGHT},
    {"general_sdk_control_right", C::CMD_GENERAL_SDK_CONTROL_RIGHT},
  };
  return m;
}

/// Human-readable name of a FunctionMode.
inline const char * function_mode_name(zsibot::FunctionMode m)
{
  switch (m) {
    case zsibot::FunctionMode::FM_REMOTE: return "remote";
    case zsibot::FunctionMode::FM_TRACE: return "trace";
    case zsibot::FunctionMode::FM_SDK: return "sdk";
    case zsibot::FunctionMode::FM_GENERAL_SDK: return "general_sdk";
    case zsibot::FunctionMode::FM_ROAMER: return "roamer";
    default: return "null";
  }
}

/// Human-readable name of a ControlMode.
inline const char * control_mode_name(zsibot::ControlMode m)
{
  switch (m) {
    case zsibot::ControlMode::CM_STAND_UP: return "stand_up";
    case zsibot::ControlMode::CM_SIT_DOWN: return "sit_down";
    case zsibot::ControlMode::CM_LOCK_MODE: return "lock";
    case zsibot::ControlMode::CM_EMERGENCY_STOP: return "emergency_stop";
    case zsibot::ControlMode::CM_MOVE_MODE: return "move";
    case zsibot::ControlMode::CM_BALANCE_STAND_MODE: return "balance_stand";
    case zsibot::ControlMode::CM_INIT: return "init";
    default: return "null";
  }
}

/// Human-readable name of a MotionMode.
inline const char * motion_mode_name(zsibot::MotionMode m)
{
  switch (m) {
    case zsibot::MotionMode::MM_RUNNING: return "running";
    case zsibot::MotionMode::MM_REST: return "rest";
    case zsibot::MotionMode::MM_FORBID: return "forbid";
    default: return "null";
  }
}

/// Human-readable name of a SpeedLevel.
inline const char * speed_level_name(zsibot::SpeedLevel s)
{
  switch (s) {
    case zsibot::SpeedLevel::SL_SLOW: return "slow";
    case zsibot::SpeedLevel::SL_NORMAL: return "normal";
    case zsibot::SpeedLevel::SL_FAST: return "fast";
    default: return "null";
  }
}

/// Human-readable name of a robot Model.
inline const char * model_name(zsibot::Model m)
{
  switch (m) {
    case zsibot::Model::MODEL_XG: return "xg_point_foot";
    case zsibot::Model::MODEL_XGW: return "xgw_wheel_foot";
    case zsibot::Model::MODEL_XGWHSPD: return "xgw_wheel_foot_highspeed";
    default: return "unknown";
  }
}

}  // namespace l1w_driver
