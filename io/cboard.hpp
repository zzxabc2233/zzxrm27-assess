#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "serial/serial.h"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

// 哨兵专有
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  // 无人机专有

  CBoard(const std::string & config_path);

  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData> queue_{5000};
  serial::Serial serial_;

  IMUData data_ahead_;
  IMUData data_behind_;
  bool has_serial_;

  std::string com_port_;
  int baud_rate_;

  // 使用与 gimbal.hpp 相同的协议结构体
  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  std::thread read_thread_;
  std::atomic<bool> quit_{false};

  mutable std::mutex serial_mutex_;
  mutable std::mutex tx_mutex_;

  void read_loop();
  bool read_bytes(uint8_t * buffer, size_t size);
  void reconnect();
};

}  // namespace io

#endif  // IO__CBOARD_HPP
