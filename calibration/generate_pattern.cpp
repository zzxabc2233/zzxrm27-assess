#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <regex>
#include <string>

// 从 xrandr 获取当前屏幕分辨率
bool detect_screen_resolution(int & width, int & height)
{
  FILE * pipe = popen("xrandr --current 2>/dev/null | grep '*' | awk '{print $1}'", "r");
  if (!pipe) return false;

  char buffer[128];
  std::string result;
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) result += buffer;
  pclose(pipe);

  // 解析 "1920x1080"
  std::regex re(R"((\d+)x(\d+))");
  std::smatch match;
  if (std::regex_search(result, match, re)) {
    width = std::stoi(match[1]);
    height = std::stoi(match[2]);
    return true;
  }
  return false;
}

const std::string keys =
  "{help h usage ? |       | 输出命令行参数说明}"
  "{width w        | 0     | 屏幕宽度(像素), 0=自动检测 }"
  "{height ht      | 0     | 屏幕高度(像素), 0=自动检测 }"
  "{screen-w sw    | 310   | 屏幕物理宽度(mm), 用尺子量 }"
  "{cols c         | 10    | 标定板列数 }"
  "{rows r         | 7     | 标定板行数 }"
  "{radius rad     | 25    | 圆点半径(像素) }"
  "{output o       | assets/pattern.png | 输出图片路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  int screen_w = cli.get<int>("width");
  int screen_h = cli.get<int>("height");
  double screen_mm = cli.get<double>("screen-w");
  int cols = cli.get<int>("cols");
  int rows = cli.get<int>("rows");
  int radius = cli.get<int>("radius");
  std::string output = cli.get<std::string>("output");

  // 自动检测屏幕分辨率
  if (screen_w == 0 || screen_h == 0) {
    if (!detect_screen_resolution(screen_w, screen_h)) {
      fmt::print("无法自动检测屏幕分辨率, 请用 -w 和 -ht 手动指定\n");
      return 1;
    }
    fmt::print("检测到屏幕分辨率: {}x{}\n", screen_w, screen_h);
  }

  // 标定板占屏幕左半边 (约 45% 宽度), 上下居中
  int margin = 40;
  int target_width = static_cast<int>(screen_w * 0.45) - 2 * margin;
  int target_height = screen_h - 2 * margin;

  int pixel_gap_w = target_width / (cols - 1);
  int pixel_gap_h = target_height / (rows - 1);
  int pixel_gap = std::min(pixel_gap_w, pixel_gap_h);

  // 圆点间距至少是半径的 3 倍
  if (pixel_gap < radius * 3) pixel_gap = radius * 3 + 1;

  // 计算物理间距
  double physical_gap_mm = static_cast<double>(pixel_gap) / screen_w * screen_mm;

  // 生成图片
  cv::Mat img(screen_h, screen_w, CV_8UC3, cv::Scalar(255, 255, 255));

  int grid_w = (cols - 1) * pixel_gap;
  int grid_h = (rows - 1) * pixel_gap;
  // 偏左放置
  int offset_x = screen_w / 4 - grid_w / 2;
  int offset_y = (screen_h - grid_h) / 2;

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      int cx = offset_x + j * pixel_gap;
      int cy = offset_y + i * pixel_gap;
      cv::circle(img, cv::Point(cx, cy), radius, cv::Scalar(0, 0, 0), -1);
    }
  }

  // 保存
  cv::imwrite(output, img);

  // 输出信息
  fmt::print("\n========== 标定板参数 ==========\n");
  fmt::print("标定板图片: {}\n", output);
  fmt::print("圆点列数:   {}\n", cols);
  fmt::print("圆点行数:   {}\n", rows);
  fmt::print("圆点半径:   {} px\n", radius);
  fmt::print("像素间距:   {} px\n", pixel_gap);
  fmt::print("物理间距:   {:.1f} mm  (屏幕宽 {}mm / {}px × {}px)\n", physical_gap_mm, static_cast<int>(screen_mm), screen_w, pixel_gap);
  fmt::print("================================\n\n");

  fmt::print(">>> 请将以下参数写入 configs/calibration.yaml:\n");
  fmt::print("    pattern_cols: {}\n", cols);
  fmt::print("    pattern_rows: {}\n", rows);
  fmt::print("    center_distance_mm: {:.1f}\n\n", physical_gap_mm);

  fmt::print(">>> 使用方式:\n");
  fmt::print("    1. 用图片查看器打开 {}, 贴到屏幕左半边\n", output);
  fmt::print("    2. 把屏幕右半边留给校准程序的可视化窗口\n");
  fmt::print("    3. 用相机对准屏幕左半边的标定板拍照\n\n");

  // 显示预览
  cv::Mat preview;
  cv::resize(img, preview, {}, 0.5, 0.5);
  cv::imshow("标定板预览 (缩小50%) - 按任意键退出 / Press any key", preview);
  cv::waitKey(0);

  return 0;
}
