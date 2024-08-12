#pragma once
#include "postprocess.h"
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "debugstream.hpp"

const double DISTANCE_THRESHOLD = 6;

// 检测框的坐标 (归一化平面坐标)
struct Box {
  // 中心点的坐标
  double x;
  double y;
  // 框的宽高
  double w;
  double h;
};

class DistanceEstimator {
public:
  DistanceEstimator(const Eigen::Matrix3d &camera_matrix, int img_width,
                    int img_height)
      : img_width(img_width), img_height(img_height),
        camera_matrix(camera_matrix) {
    // 不同类别的平均身高 (单位: 米)
    height_lookup = {{"person", 1.70}, {"Ren", 1.75}, {"Dianti", 2.2}};
  }

  double EstimateDistance(double x, double y, double w, double h,
                          const std::string &class_name) {
    // 计算检测框的中心点坐标 (像素)

    double center_x = x * img_width;
    double center_y = y * img_height;

    // 计算图像平面上的检测框高度(像素)
    double bbox_height_px = h * img_height;

    // 根据目标类别查找平均身高
    if (height_lookup.find(class_name) == height_lookup.end()) {
      std::cerr << "error: " << class_name << " not found" << std::endl;
      std::terminate();
      return -1;
    }
    double target_height = height_lookup[class_name];

    // 根据相似三角形原理计算距离
    double distance = (target_height * camera_matrix(0, 0)) / bbox_height_px;
    return distance;
  }

  Eigen::Vector3d Get3DPosition(double x, double y, double w, double h,
                                const std::string &class_name) {
    // 计算检测框的中心点坐标 (像素)
    double center_x = x * img_width;

    double center_y = y * img_height;

    // 计算距离
    double distance = EstimateDistance(x, y, w, h, class_name);

    if (distance < 0) {
      return Eigen::Vector3d::Zero();
    }

    // 根据相机内参和距离计算3D位置
    double Z = distance;
    double X = (center_x - camera_matrix(0, 2)) * Z / camera_matrix(0, 0);
    double Y = (center_y - camera_matrix(1, 2)) * Z / camera_matrix(1, 1);
    return Eigen::Vector3d(X, Y, Z);
  }

  // 批量处理一系列的3D点，返回值是对应到相机坐标系的3D点
  // 注意这个批量只能是同一个类别
  // 如果有多个类别，需要多次执行处理
  std::vector<Eigen::Vector3d> Get3DPosition(const std::vector<double> &x,
                                             const std::vector<double> &y,
                                             const std::vector<double> &w,
                                             const std::vector<double> &h,
                                             const std::string &class_name) {
    assert(x.size() == y.size() && x.size() == w.size() &&
           x.size() == h.size());
    std::vector<Eigen::Vector3d> res(x.size());
    for (int i = 0; i < x.size(); i++) {
      res[i] = Get3DPosition(x[i], y[i], w[i], h[i], class_name);
    }
    return res;
  }
  // 批量处理一系列的3D点，返回值是对应到相机坐标系的3D点
  // 注意这个批量只能是同一个类别
  // 如果有多个类别，需要多次执行处理
  std::vector<Eigen::Vector3d> Get3DPosition(const std::vector<Box> &boxes,
                                             const std::string &class_name) {
    std::vector<Eigen::Vector3d> res(boxes.size());
    for (int i = 0; i < boxes.size(); i++) {
      res[i] = Get3DPosition(boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h,
                             class_name);
    }
    return res;
  }

private:
  int img_width;
  int img_height;
  Eigen::Matrix3d camera_matrix;
  std::unordered_map<std::string, double> height_lookup;
};

inline double CalculateDistance(const Eigen::Vector3d &point1,
                                const Eigen::Vector3d &point2) {
  return std::sqrt((point1 - point2).squaredNorm());
}

inline void DealImage(DistanceEstimator &es, cv::Mat &image,
                      const std::vector<Box> &rens,
                      const std::vector<Box> &diantis) {
  gDebug(rens);
  gDebug(diantis);
  static std::vector<cv::Scalar> colors{{0, 255, 0},   {0, 0, 255},
                                        {255, 0, 0},   {255, 255, 0},
                                        {255, 0, 255}, {0, 255, 255}};
  auto positions_ren = es.Get3DPosition(rens, "Ren");
  gDebug(positions_ren);
  auto positions_dianti = es.Get3DPosition(diantis, "Dianti");
  gDebug(positions_dianti);

  std::vector<int> relate_ren(diantis.size(), 0);
  for (int i = 0; i < diantis.size(); i++) {
    cv::Point p_dianti(diantis[i].x * image.cols, diantis[i].y * image.rows);
    for (int j = 0; j < rens.size(); j++) {
      auto distance = CalculateDistance(positions_dianti[i], positions_ren[j]);
      gDebug(distance);
      if (distance > DISTANCE_THRESHOLD) {
        continue;
      }
      relate_ren[i]++;
      cv::Point p_ren(rens[j].x * image.cols, rens[j].y * image.rows);
      cv::arrowedLine(image, p_dianti, p_ren, colors.at(i % colors.size()), 2);
    }
  }
}

inline int Test() {
  int img_width = 1280; // 图像宽度 (像素)
  int img_height = 640; // 图像高度 (像素)

  // 相机内参
  Eigen::Matrix3d camera_matrix;
  camera_matrix << 787.22316869, 0.0, 628.91534144, 0.0, 793.45182,
      313.46301416, 0.0, 0.0, 1.0;

  DistanceEstimator estimator(camera_matrix, img_width, img_height);

  // 估算人的距离

  // Eigen::Vector4d bbox1(0.4, 0.3, 0.2, 0.5);  // 检测框的归一化坐标 (x, y, w,
  // h)
  double distance_person =
      estimator.EstimateDistance(0.4, 0.3, 0.2, 0.5, "Ren");
  std::cout << "估算的人距离: " << distance_person << " 米" << std::endl;
  Eigen::Vector3d position_person =
      estimator.Get3DPosition(0.4, 0.3, 0.2, 0.5, "Ren");

  std::cout << "估算的人位置: " << position_person.transpose() << " 米"
            << std::endl;

  // 估算车的距离
  // Eigen::Vector4d bbox2(0.8, 0.3, 0.2, 0.1);  // 检测框的归一化坐标 (x, y, w,
  // h)
  double distance_vehicle =
      estimator.EstimateDistance(0.8, 0.3, 0.2, 0.1, "Dianti");
  std::cout << "估算的车距离: " << distance_vehicle << " 米" << std::endl;
  Eigen::Vector3d position_vehicle =
      estimator.Get3DPosition(0.8, 0.3, 0.2, 0.1, "Dianti");
  std::cout << "估算的车位置: " << position_vehicle.transpose() << " 米"
            << std::endl;

  double distance = CalculateDistance(position_person, position_vehicle);
  std::cout << "两点之间的距离: " << distance << " 米" << std::endl;

  std::vector<Box> rens;
  rens.push_back({0.4, 0.3, 0.2, 0.1});
  rens.push_back({0.5, 0.4, 0.2, 0.5});
  std::vector<Box> diantis;
  diantis.push_back({0.5, 0.3, 0.2, 0.5});
  diantis.push_back({0.6, 0.4, 0.2, 0.5});

  cv::Mat image = cv::Mat::zeros(640, 1280, CV_8UC3);
  DealImage(estimator, image, rens, diantis);
  cv::imwrite("output.jpg", image);
  // cv::imshow("Image", image);
  // cv::waitKey(0);
  // cv::destroyAllWindows();
  // gDebug(relate_ren);

  return 0;
}
