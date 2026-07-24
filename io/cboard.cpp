#include "cboard.hpp"

#include <fmt/ranges.h>

#include <cstdio>
#include <cstring>
#include <thread>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  shoot_mode(ShootMode::left_shoot),
  bullet_speed(0),
  ft_angle(0),
  has_serial_(false)
{
  auto config = tools::load(config_path);

  // 读取串口配置, 与 Gimbal 使用相同的 com_port 或独立的 cboard_com_port
  if (config["cboard_com_port"]) {
    com_port_ = config["cboard_com_port"].as<std::string>();
  } else if (config["com_port"]) {
    com_port_ = config["com_port"].as<std::string>();
  } else {
    com_port_ = "";
  }
  baud_rate_ = config["cboard_baud_rate"] ? config["cboard_baud_rate"].as<int>() : 115200;

  if (com_port_.empty()) {
    tools::logger()->info("[CBoard] No serial port configured, using dummy IMU.");
    data_ahead_.q = Eigen::Quaterniond::Identity();
    data_ahead_.timestamp = std::chrono::steady_clock::now();
    data_behind_ = data_ahead_;
  } else {
    has_serial_ = true;

    // 尝试打开串口
    try {
      serial_.setPort(com_port_);
      serial_.setBaudrate(baud_rate_);

      serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
      timeout.inter_byte_timeout = 10;
      serial_.setTimeout(timeout);

      serial_.open();
      tools::logger()->info("[CBoard] Serial port {} opened at {} baud.", com_port_, baud_rate_);
    } catch (const std::exception & e) {
      tools::logger()->error("[CBoard] Failed to open serial port {}: {}", com_port_, e.what());
      has_serial_ = false;
      data_ahead_.q = Eigen::Quaterniond::Identity();
      data_ahead_.timestamp = std::chrono::steady_clock::now();
      data_behind_ = data_ahead_;
      return;
    }

    // 启动读取线程
    read_thread_ = std::thread(&CBoard::read_loop, this);

    // 等待收到第一帧有效的 IMU 数据
    tools::logger()->info("[CBoard] Waiting for first valid frame...");
    queue_.pop(data_ahead_);
    queue_.pop(data_behind_);
    tools::logger()->info("[CBoard] First valid frame received.");
  }
}

CBoard::~CBoard()
{
  quit_ = true;
  if (read_thread_.joinable()) read_thread_.join();
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_.close();
  } catch (...) {
  }
  tools::logger()->info("[CBoard] Destructed.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (!has_serial_) return Eigen::Quaterniond::Identity();

  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数球面线性插值 (slerp)
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command)
{
  if (!has_serial_) return;

  VisionToGimbal frame;
  // mode: 0=不控制, 1=控制云台但不开火, 2=控制云台且开火
  frame.mode = command.control ? (command.shoot ? 2 : 1) : 0;
  // head 和 tail 已在结构体定义中默认初始化

  {
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
    tx_data_.mode = frame.mode;
    tx_data_.yaw = static_cast<float>(command.yaw);
    tx_data_.pitch = static_cast<float>(command.pitch);
    tx_data_.yaw_vel = 0.0f;
    tx_data_.yaw_acc = 0.0f;
    tx_data_.pitch_vel = 0.0f;
    tx_data_.pitch_acc = 0.0f;

    // 日志：打印发送帧的十六进制
    {
      std::string hex;
      hex.reserve(sizeof(tx_data_) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&tx_data_);
      for (size_t i = 0; i < sizeof(tx_data_); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug("[CBoard] TX frame ({} bytes): {}", sizeof(tx_data_), hex);
    }

    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    try {
      serial_.write(reinterpret_cast<const uint8_t *>(&tx_data_), sizeof(tx_data_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Failed to write serial: {}", e.what());
    }
  }
}

bool CBoard::read_bytes(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[CBoard] Failed to read serial: {}", e.what());
    return false;
  }
}

void CBoard::read_loop()
{
  tools::logger()->info("[CBoard] read_loop started.");
  const int PACKET_SIZE = sizeof(GimbalToVision);  // 43 字节, 与 Gimbal 类一致
  uint8_t buffer[PACKET_SIZE];
  int error_count = 0;

  while (!quit_) {
    if (error_count > 50) {
      error_count = 0;
      tools::logger()->warn("[CBoard] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 1. 寻找帧头 (滑动窗口逐字节搜索 0x5A 0xA5)
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      uint8_t prev = 0;
      uint8_t cur = 0;
      while (!quit_) {
        if (!read_bytes(&cur, 1)) {
          std::this_thread::sleep_for(std::chrono::microseconds(100));
          continue;
        }

        if (prev == 0x5A && cur == 0xA5) {
          buffer[0] = 0x5A;
          buffer[1] = 0xA5;
          break;
        }

        prev = cur;
      }
    }
    if (quit_) break;

    // 等待剩余数据到达 (43字节 @ 115200bps ≈ 3.7ms)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // 2. 读取剩余 41 字节
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (!read_bytes(&buffer[2], PACKET_SIZE - 2)) {
        error_count++;
        tools::logger()->warn("[CBoard] Failed to read remaining {} bytes", PACKET_SIZE - 2);
        continue;
      }
    }

    // 3. 检查帧尾 0x7F 0xFE
    if (buffer[PACKET_SIZE - 2] != 0x7F || buffer[PACKET_SIZE - 1] != 0xFE) {
      error_count++;
      tools::logger()->warn(
        "[CBoard] Tail check failed: got [{:#04x} {:#04x}], expected [0x7f 0xfe]",
        static_cast<int>(buffer[PACKET_SIZE - 2]),
        static_cast<int>(buffer[PACKET_SIZE - 1]));
      tools::logger()->warn(
        "[CBoard] Full buffer dump: {:02x}",
        fmt::join(buffer, buffer + PACKET_SIZE, " "));
      continue;
    }

    // 4. 解析数据包 (GimbalToVision, 43字节)
    std::memcpy(&rx_data_, buffer, PACKET_SIZE);
    auto t = std::chrono::steady_clock::now();

    // 日志：打印接收帧的十六进制
    {
      std::string hex;
      hex.reserve(sizeof(rx_data_) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&rx_data_);
      for (size_t i = 0; i < sizeof(rx_data_); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug("[CBoard] RX frame ({} bytes): {}", sizeof(rx_data_), hex);
    }

    error_count = 0;

    // 构造四元数 (wxyz 顺序)
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);

    // 验证四元数合法性
    if (std::abs(q.squaredNorm() - 1.0) > 1e-2) {
      tools::logger()->warn(
        "[CBoard] Invalid q: {:.4f} {:.4f} {:.4f} {:.4f}",
        q.w(), q.x(), q.y(), q.z());
      continue;
    }

    // 更新状态 (与 Gimbal::read_thread 一致的解析逻辑)
    bullet_speed = rx_data_.bullet_speed;
    mode = static_cast<Mode>(rx_data_.mode);  // 0=idle, 1=auto_aim, 2=small_buff, 3=big_buff
    // shoot_mode 和 ft_angle 不在 GimbalToVision 协议中, 保持默认值

    // 限制日志输出频率为 1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();
    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info(
        "[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, yaw: {:.2f}°, pitch: {:.2f}°",
        bullet_speed, MODES[mode],
        rx_data_.yaw * 180.0 / M_PI, rx_data_.pitch * 180.0 / M_PI);
      last_log_time = now;
    }

    // 推入 IMU 数据队列
    queue_.push({q.normalized(), t});
  }

  tools::logger()->info("[CBoard] read_loop stopped.");
}

void CBoard::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[CBoard] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_.open();
      queue_.clear();
      tools::logger()->info("[CBoard] Reconnected serial successfully.");
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[CBoard] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  tools::logger()->error("[CBoard] Failed to reconnect after {} attempts.", max_retry_count);
}

}  // namespace io
