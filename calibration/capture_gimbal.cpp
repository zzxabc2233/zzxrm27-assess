#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o | assets/img_with_q        | 输出文件夹路径   }";

void write_q(const std::string & q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz（与capture.cpp保持一致）
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder)
{
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;
  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = gimbal.latest_q();  // 直接用最新四元数，无插值延迟

    // 在图像上显示欧拉角
    auto display = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(display, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(display, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(display, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    cv::resize(display, display, {}, 0.5, 0.5);
    cv::imshow("Gimbal Capture | s=save q=quit", display);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q' || key == 'Q')
      break;
    else if (key != 's' && key != 'S')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved: {}", count, output_folder);
  }
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  std::filesystem::create_directory(output_folder);

  tools::logger()->info("标定板固定在地面，手动转动云台拍照");
  capture_loop(config_path, output_folder);
  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
