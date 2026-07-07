#include <fmt/core.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "tools/logger.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/calib_images  | 输出文件夹路径   }";

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

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int count = 0;

  tools::logger()->info("按 s 保存图片，按 q 退出！");
  
  while (true) {
    camera.read(img, timestamp);
    
    auto display_img = img.clone();
    cv::resize(display_img, display_img, {}, 0.5, 0.5);
    cv::imshow("Calibration Capture", display_img);
    
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key == 's') {
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      cv::imwrite(img_path, img);
      tools::logger()->info("[{}] Saved to: {}", count, img_path);
    }
  }

  return 0;
}
