#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path)
: last_command_{false, false, 0, 0}, last_on_target_(false)
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) {
    last_on_target_ = false;
    return false;
  }

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;

  // 判断当前帧是否在瞄准范围内
  bool on_target =
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&   // command没有突变
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&     // 云台已经到位
    aimer.debug_aim_point.valid;                                   // 有有效瞄准点

  // 单发模式：只在瞄准信号的上升沿触发一次开火
  if (on_target && !last_on_target_) {
    last_command_ = command;
    last_on_target_ = true;
    return true;
  }

  last_command_ = command;
  last_on_target_ = on_target;
  return false;
}

}  // namespace auto_aim