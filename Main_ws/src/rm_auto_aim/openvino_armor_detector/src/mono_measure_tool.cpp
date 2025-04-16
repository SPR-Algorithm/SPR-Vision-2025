// Copyright 2025 ZhangSuHang

#include <openvino_armor_detector/mono_measure_tool.hpp>
#include <openvino_armor_detector/openvino_detector.hpp>

namespace rm_auto_aim
{

std::vector<cv::Point3f> MonoMeasureTool::small_armor_3d_points = {{-0.066, 0.027, 0},//顺序为左上，左下，右下，右上，单位为m
  {-0.066, -0.027, 0},
  {0.066, -0.027, 0},
  {0.066, 0.027, 0}};

std::vector<cv::Point3f> MonoMeasureTool::big_armor_3d_points = {{-0.1125, 0.027, 0},//顺序为左上，左下，右下，右上，单位为m
  {-0.1125, -0.027, 0},
  {0.1125, -0.027, 0},
  {0.1125, 0.027, 0}};

bool is_big_armor(const ArmorObject & obj)
{
  switch (obj.label) {
    case 1://大装甲板
      return true;
    case 8://基地大装甲板
      return true;
    default://其余默认为小装甲板
      return false;
  }
}

MonoMeasureTool::MonoMeasureTool(
  std::vector<double> camera_intrinsic,
  std::vector<double> camera_distortion)
{
  set_camera_info(camera_intrinsic, camera_distortion);
}

bool MonoMeasureTool::set_camera_info(
  std::vector<double> camera_intrinsic,
  std::vector<double> camera_distortion)
{
  if (camera_intrinsic.size() != 9) {
    // the size of camera intrinsic must be 9 (equal 3*3)
    return false;
  }
  // init camera_intrinsic and camera_distortion
  cv::Mat camera_intrinsic_mat(camera_intrinsic, true);
  camera_intrinsic_mat = camera_intrinsic_mat.reshape(0, 3);
  camera_intrinsic_ = camera_intrinsic_mat.clone();

  cv::Mat camera_distortion_mat(camera_distortion, true);
  camera_distortion_mat = camera_distortion_mat.reshape(0, 1);
  camera_distortion_ = camera_distortion_mat.clone();
  return true;
}

bool MonoMeasureTool::solve_pnp(
  const std::vector<cv::Point2f> & points2d, const std::vector<cv::Point3f> & points3d,
  cv::Point3f & position, cv::Mat & rvec, cv::SolvePnPMethod pnp_method)
{
  if (points2d.size() != points3d.size()) {
    return false;  // 投影点数量不匹配
  }
  // cv::Mat rot = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat trans = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat r;  // 旋转向量
  bool res = cv::solvePnP(
    points3d, points2d, camera_intrinsic_, camera_distortion_, r, trans, false,
    pnp_method);
  rvec = r.clone();
  position = cv::Point3f(trans);
  return res;
}

// refer to :http://www.cnblogs.com/singlex/p/pose_estimation_1_1.html
// 根据输入的参数将图像坐标转换到相机坐标中
// 输入为图像上的点坐标
// double distance 物距
// 输出3d点坐标的单位与distance（物距）的单位保持一致
cv::Point3f MonoMeasureTool::unproject(cv::Point2f p, double distance)
{
  auto fx = camera_intrinsic_.ptr<double>(0)[0];
  auto u0 = camera_intrinsic_.ptr<double>(0)[2];
  auto fy = camera_intrinsic_.ptr<double>(1)[1];
  auto v0 = camera_intrinsic_.ptr<double>(1)[2];

  double zc = distance;
  double xc = (p.x - u0) * distance / fx;
  double yc = (p.y - v0) * distance / fy;
  return cv::Point3f(xc, yc, zc);
}

// 获取image任意点的视角，pitch，yaw（相对相机坐标系）。
// 与相机坐标系保持一致。
void MonoMeasureTool::calc_view_angle(cv::Point2f p, float & pitch, float & yaw)
{
  auto fx = camera_intrinsic_.ptr<double>(0)[0];
  auto u0 = camera_intrinsic_.ptr<double>(0)[2];
  auto fy = camera_intrinsic_.ptr<double>(1)[1];
  auto v0 = camera_intrinsic_.ptr<double>(1)[2];

  pitch = atan2((p.y - v0), fy);
  yaw = atan2((p.x - u0), fx);
}

bool MonoMeasureTool::calc_armor_target(
  const ArmorObject & obj, cv::Point3f & position,
  cv::Mat & rvec)
{
  std::vector<cv::Point2f> pts;

  //使用for循环将obj中landmarks的8个元素转换为cv::Point2f类型的pts，obj是指针类型
  for (int i = 0; i < 8; i += 2) {
    pts.push_back(cv::Point2f(obj.landmarks[i], obj.landmarks[i + 1]));
  }

  if (is_big_armor(obj)) {
    return solve_pnp(pts, big_armor_3d_points, position, rvec, cv::SOLVEPNP_IPPE);
  } else {
    return solve_pnp(pts, small_armor_3d_points, position, rvec, cv::SOLVEPNP_IPPE);
  }
}

float MonoMeasureTool::calc_distance_to_center(const ArmorObject & obj)
{
  cv::Point2f img_center(this->camera_intrinsic_.at<double>(0, 2),
    this->camera_intrinsic_.at<double>(1, 2));
  cv::Point2f armor_center;
  armor_center.x = (obj.landmarks[0] + obj.landmarks[2] + obj.landmarks[4] + obj.landmarks[6]) / 4.;
  armor_center.y = (obj.landmarks[1] + obj.landmarks[3] + obj.landmarks[5] + obj.landmarks[7]) / 4.;
  auto dis_vec = img_center - armor_center;
  return sqrt(dis_vec.dot(dis_vec));
}

}  // namespace rm_auto_aim
