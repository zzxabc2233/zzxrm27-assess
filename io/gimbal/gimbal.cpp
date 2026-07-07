#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <cstdio>  //新增
#include <cstring> //新增2
 #include <fmt/ranges.h>

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");//读取配置文件中的com_port

  try {
    serial_.setPort(com_port);//设置串口
    serial_.setBaudrate(115200);//设置波特率
    
    // 设置超时：inter_byte_timeout确保字节间读取不超时
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100); // 100ms总超时
    timeout.inter_byte_timeout = 10; // 字节间超时10ms
    serial_.setTimeout(timeout);
    
    serial_.open();
    // serial_.flushInput();//不能加！否则就是问题。    
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();//我们的gimbal的队列是1000——必须先收到至少一帧合法姿态数据，系统才继续往下跑
  tools::logger()->info("[Gimbal] First q received.");//如果没有下文，说明：1、没有数据 2、我并没有收到合法的姿态数据
}

/*析构函数，用于退出*/
Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

/*获取云台模式——0: 空闲, 1: 自瞄, 2: 小符, 3: 大符*/
GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

/*获取云台状态——*/
GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";//默认
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  auto gs = state();//新增
   VisionToGimbal.yaw = tools::limit_rad(VisionToGimbal.yaw - gs.yaw);
  VisionToGimbal.pitch = tools::limit_rad(VisionToGimbal.pitch - gs.pitch);

  if (VisionToGimbal.mode == 0) { // mode=0对应IDLE，无目标
    VisionToGimbal.yaw = 0.0f;
    VisionToGimbal.pitch = 0.0f;
  }

  {
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
    tx_data_.mode = VisionToGimbal.mode;
    tx_data_.yaw = VisionToGimbal.yaw;
    tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
    tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
    tx_data_.pitch = VisionToGimbal.pitch;
    tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
    tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  // tx_data_.yaw = 0.0f;//实验，新增赋值——一会删掉
  // 帧尾已在结构体中默认为 {0x7F, 0xFE}，无需计算

    { //日志
      std::string hex;
      hex.reserve(sizeof(tx_data_) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&tx_data_);
      for (size_t i = 0; i < sizeof(tx_data_); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug(
        "[Gimbal] tx frame ({} bytes): {}",
        sizeof(tx_data_), hex);
    }

    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    try {
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  auto gs = state();
  yaw = tools::limit_rad(yaw - gs.yaw);
  pitch = tools::limit_rad(pitch - gs.pitch);

  if (!control) 
  { // control=false表示无目标，直接清零yaw/pitch
    yaw = 0.0f;
    pitch = 0.0f;
  }

  {
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
    tx_data_.mode = control ? (fire ? 2 : 1) : 0;
    tx_data_.yaw = yaw;
    tx_data_.yaw_vel = yaw_vel;
    tx_data_.yaw_acc = yaw_acc;
    tx_data_.pitch = pitch;
    tx_data_.pitch_vel = pitch_vel;
    tx_data_.pitch_acc = pitch_acc;
  // tx_data_.yaw = 0.0f;//实验，一会删掉。
  // 帧尾已在结构体中默认为 {0x7F, 0xFE}，无需计算

    { //日志
      std::string hex;
      hex.reserve(sizeof(tx_data_) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&tx_data_);
      for (size_t i = 0; i < sizeof(tx_data_); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug(
        "[Gimbal] tx frame ({} bytes): {}",
        sizeof(tx_data_), hex);
    }

    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    try {
      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
    }
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
   tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}


void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  const int PACKET_SIZE = 43;
  uint8_t buffer[PACKET_SIZE];
  int error_count = 0;
  bool has_received_data = false; // 核心：是否已收到云台的有效回调数据

  //error_count > 5000  原来
  while (!quit_) {
    if (error_count > 50) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // 1. 寻找帧头（滑动窗口，避免 2-bytes read 跨边界导致错过帧头）
    uint8_t prev = 0;
    uint8_t cur = 0;
    while (!quit_) {
      if (!read(&cur, 1)) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }

      if (prev == 0x5A && cur == 0xA5) {
        buffer[0] = 0x5A;
        buffer[1] = 0xA5;
        // tools::logger()->debug("[Gimbal] Found frame head 0x5A 0xA5");
        
        // 找到帧头后，等待剩余数据到达（43字节 @ 115200bps ≈ 3.7ms） 新增的
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        break;
      }

      prev = cur;
    }
    if (quit_) break;

    // auto t = std::chrono::steady_clock::now();
    //2、读取剩余数据（一次性读取，避免逐字节读取太慢）
    if (!read(&buffer[2], PACKET_SIZE - 2)) {
      error_count++;
      tools::logger()->warn("[Gimbal] Failed to read remaining {} bytes", PACKET_SIZE - 2);
      continue;
    }
    
    // 3. 检查帧尾
    if (buffer[PACKET_SIZE - 1] != 0xFE or buffer[PACKET_SIZE - 2] != 0x7F) {
      error_count++;
      tools::logger()->warn(
        "[Gimbal] Tail check failed: got [{:#04x} {:#04x}], expected [0x7f 0xfe]",
        static_cast<int>(buffer[PACKET_SIZE - 2]), static_cast<int>(buffer[PACKET_SIZE - 1]));
      tools::logger()->warn("[Gimbal] Full buffer dump: {:02x}", fmt::join(buffer, buffer + PACKET_SIZE, " "));
      
      // 清空串口缓冲区，丢弃积压的旧数据
      // try {
      //   serial_.flushInput();
      //   tools::logger()->info("[Gimbal] Flushed input buffer after tail check failure");
      // } catch (...) {}
      
      continue;
    }
    
    // tools::logger()->info("[Gimbal] Valid frame received! Tail check passed.");

    // 4. 解析数据包
    memcpy(&rx_data_, buffer, PACKET_SIZE);
    auto t = std::chrono::steady_clock::now();



  {
    std::string hex;
    hex.reserve(sizeof(rx_data_) * 3);
    const auto * p = reinterpret_cast<const uint8_t *>(&rx_data_);
    for (size_t i = 0; i < sizeof(rx_data_); ++i) {
      char b[4];
      std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
      hex += b;
    }
    tools::logger()->debug("[Gimbal] rx frame ({} bytes): {}", sizeof(rx_data_), hex);
  } //新增的{}——把收到的整帧数据按十六进制打印出来，方便你调试。
    // auto t = std::chrono::steady_clock::now();//尝试放这里试试看。
    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      //yaw可能需要取得反值——原来的是下面两行，改后去取了相反值
      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      // state_.yaw =  -rx_data_.yaw;
      // state_.yaw_vel = -rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;

      switch (rx_data_.mode) {
        case 0:
          mode_ = GimbalMode::IDLE;
          break;
        case 1:
          mode_ = GimbalMode::AUTO_AIM;
          break;
        case 2:
          mode_ = GimbalMode::SMALL_BUFF;
          break;
        case 3:
          mode_ = GimbalMode::BIG_BUFF;
          break;
        default:
          mode_ = GimbalMode::IDLE;
          tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
          break;
      }
    }

    queue_.push({q, t});
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
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
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
