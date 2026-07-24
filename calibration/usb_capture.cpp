#include <fmt/core.h>

#include <filesystem>
#include <opencv2/opencv.hpp>

const std::string keys =
  "{help h usage ?  |                        | 输出命令行参数说明}"
  "{camera c        | 0                      | 摄像头编号, 默认0 }"
  "{output-folder o | assets/calib_images    | 输出文件夹路径     }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  int camera_id = cli.get<int>("camera");
  std::string output_folder = cli.get<std::string>("output-folder");

  // 创建输出文件夹
  std::filesystem::create_directory(output_folder);

  // 打开摄像头
  cv::VideoCapture cap(camera_id);
  if (!cap.isOpened()) {
    fmt::print("无法打开摄像头 /dev/video{}，尝试其他编号: -c=1\n", camera_id);
    return 1;
  }

  // 设置分辨率（可选，不设就用默认）
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);

  int actual_w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  int actual_h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  fmt::print("摄像头已打开: /dev/video{}, 分辨率 {}x{}\n", camera_id, actual_w, actual_h);
  fmt::print("按 s 保存图片，按 q 退出！\n");

  cv::Mat img;
  int count = 0;

  while (true) {
    cap >> img;
    if (img.empty()) {
      fmt::print("读取帧失败，重试中...\n");
      continue;
    }

    // 缩小显示
    cv::Mat display;
    cv::resize(img, display, {}, 0.4, 0.4);
    cv::imshow("USB Capture | s=save q=quit", display);

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 'q' || key == 'Q') {
      break;
    } else if (key == 's' || key == 'S') {
      count++;
      std::string img_path = fmt::format("{}/{}.jpg", output_folder, count);
      cv::imwrite(img_path, img);
      fmt::print("[{}] 已保存: {}\n", count, img_path);
    }
  }

  cap.release();
  fmt::print("共采集 {} 张图片\n", count);
  return 0;
}
