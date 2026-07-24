#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{@input-folder  | assets/img_with_q        | 输入文件夹路径   }";

// 串口Gimbal发来的四元数直接就是R_gimbal2world, 不需要R_gimbal2imubody转换
// 这个版本跳过R_gimbal2imubody, 直接用四元数

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;
  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});
  return centers_3d;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream q_file(q_path);
  double w, x, y, z;
  q_file >> w >> x >> y >> z;
  return {w, x, y, z};
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");

  // 读取参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  cv::Matx33d camera_matrix(camera_matrix_data.data());
  cv::Mat distort_coeffs(distort_coeffs_data);

  std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list;
  std::vector<cv::Mat> rvecs, tvecs;

  fmt::print("\n===== 串口Gimbal手眼标定 =====\n");
  fmt::print("四元数直接作为R_gimbal2world，不做R_gimbal2imubody转换\n\n");

  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto q_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;

    Eigen::Quaterniond q = read_q(q_path);
    Eigen::Matrix3d R_gimbal2world = q.toRotationMatrix();

    // 显示欧拉角，用于验证四元数是否正确
    Eigen::Vector3d zyx = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Gimbal Handeye | Press any key", drawing);
    cv::waitKey(0);

    fmt::print("[{}] {}  Z={:.1f} Y={:.1f} X={:.1f}\n",
               success ? "success" : "failure", img_path, zyx[0], zyx[1], zyx[2]);
    if (!success) continue;

    // PnP: 标定板 -> 相机
    cv::Mat rvec, tvec;
    auto obj_pts = centers_3d(pattern_size, center_distance_mm);
    cv::solvePnP(obj_pts, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 云台在世界系下的位姿 (假设云台在世界原点, 只旋转不平移)
    cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);

    R_gimbal2world_list.push_back(R_gimbal2world_cv);
    t_gimbal2world_list.push_back(t_gimbal2world);
    rvecs.push_back(rvec);
    tvecs.push_back(tvec);
  }

  fmt::print("\n有效数据: {} 组\n", R_gimbal2world_list.size());

  if (R_gimbal2world_list.size() < 3) {
    fmt::print("错误: 至少需要3组有效数据!\n");
    return 1;
  }

  // ====== 方法1: calibrateHandEye ======
  cv::Mat R_camera2gimbal, t_camera2gimbal;
  cv::calibrateHandEye(
    R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs,
    R_camera2gimbal, t_camera2gimbal, cv::CALIB_HAND_EYE_TSAI);
  t_camera2gimbal /= 1e3;  // mm -> m

  // ====== 方法2: calibrateRobotWorldHandEye ======
  cv::Mat R_gimbal2camera_rw, t_gimbal2camera_rw;
  cv::Mat R_world2board_rw, t_world2board_rw;
  cv::calibrateRobotWorldHandEye(
    rvecs, tvecs, R_gimbal2world_list, t_gimbal2world_list,
    R_world2board_rw, t_world2board_rw, R_gimbal2camera_rw, t_gimbal2camera_rw);
  t_gimbal2camera_rw /= 1e3;
  t_world2board_rw /= 1e3;

  cv::Mat R_camera2gimbal_rw, t_camera2gimbal_rw;
  cv::transpose(R_gimbal2camera_rw, R_camera2gimbal_rw);
  t_camera2gimbal_rw = -R_camera2gimbal_rw * t_gimbal2camera_rw;

  // 计算偏角
  auto calc_ypr = [](const cv::Mat & R) -> Eigen::Vector3d {
    Eigen::Matrix3d R_eigen;
    cv::cv2eigen(R, R_eigen);
    Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
    Eigen::Matrix3d R_camera2ideal = R_gimbal2ideal * R_eigen;
    return tools::eulers(R_camera2ideal, 1, 0, 2) * 57.3;
  };

  auto ypr1 = calc_ypr(R_camera2gimbal);
  auto ypr2 = calc_ypr(R_camera2gimbal_rw);

  fmt::print("\n===== calibrateHandEye (Tsai) =====\n");
  fmt::print("相机偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree\n", ypr1[0], ypr1[1], ypr1[2]);

  fmt::print("\n===== calibrateRobotWorldHandEye =====\n");
  fmt::print("相机偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} degree\n", ypr2[0], ypr2[1], ypr2[2]);

  auto x = t_world2board_rw.at<double>(0);
  auto y = t_world2board_rw.at<double>(1);
  auto dist = std::sqrt(x * x + y * y);
  fmt::print("标定板到世界原点距离: {:.2f} m\n", dist);

  // 输出 YAML
  YAML::Emitter out;
  std::vector<double> R1(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t1(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());
  std::vector<double> R2(R_camera2gimbal_rw.begin<double>(), R_camera2gimbal_rw.end<double>());
  std::vector<double> t2(t_camera2gimbal_rw.begin<double>(), t_camera2gimbal_rw.end<double>());

  out << YAML::BeginMap;
  out << YAML::Comment("========== calibrateHandEye (Tsai) ==========");
  out << YAML::Newline;
  out << YAML::Comment(fmt::format("相机偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} deg", ypr1[0], ypr1[1], ypr1[2]));
  out << YAML::Key << "R_camera2gimbal_tsai";
  out << YAML::Value << YAML::Flow << R1;
  out << YAML::Key << "t_camera2gimbal_tsai";
  out << YAML::Value << YAML::Flow << t1;
  out << YAML::Newline;
  out << YAML::Comment("========== calibrateRobotWorldHandEye ==========");
  out << YAML::Newline;
  out << YAML::Comment(fmt::format("相机偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} deg", ypr2[0], ypr2[1], ypr2[2]));
  out << YAML::Key << "R_camera2gimbal";
  out << YAML::Value << YAML::Flow << R2;
  out << YAML::Key << "t_camera2gimbal";
  out << YAML::Value << YAML::Flow << t2;
  out << YAML::Newline;
  out << YAML::EndMap;
  fmt::print("\n{}\n", out.c_str());

  return 0;
}
