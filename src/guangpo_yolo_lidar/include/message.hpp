#pragma once

#include <perception_msgs/PerceptionObstacle.h>
#include <perception_msgs/PerceptionObstacles.h>

#include "yolo_lidar_help.hpp"

inline perception_msgs::PerceptionObstacles
PackageMessage(const std::vector<mybox> &box3ds, bool verbose = false) {
  static auto SetValue = [](geometry_msgs::Point &des, const cv::Point2f &src) {
    des.x = src.x;
    des.y = src.y;
    des.z = 0;
  };

  perception_msgs::PerceptionObstacles obstacles;

  for (int i = 0; i < box3ds.size(); i++) {
    auto &box = box3ds[i];
    perception_msgs::PerceptionObstacle obstacle;
    obstacle.label = box.label;
    // 如果聚类失败了(点太少或者不满足要求)
    if (box.lidar_cluster->points.empty()) {
      continue;
    }
    SetValue(obstacle.center_point, box.position.center);
    obstacle.corner_points.resize(4);
    SetValue(obstacle.corner_points.at(0), box.position.vertices.at(0));
    SetValue(obstacle.corner_points.at(1), box.position.vertices.at(1));
    SetValue(obstacle.corner_points.at(2), box.position.vertices.at(2));
    SetValue(obstacle.corner_points.at(3), box.position.vertices.at(3));
    obstacles.obstacles.emplace_back(std::move(obstacle));
  }
  if (verbose) {
    for (int i = 0; i < obstacles.obstacles.size(); i++) {
      auto &obst = obstacles.obstacles.at(i);
      gDebug() << VAR("obstacle", i, obst.label);
      gDebug() << VAR(obst.center_point.x, obst.center_point.y);
      gDebug() << VAR(obst.corner_points.at(0).x, obst.corner_points.at(0).y);
      gDebug() << VAR(obst.corner_points.at(1).x, obst.corner_points.at(1).y);
      gDebug() << VAR(obst.corner_points.at(2).x, obst.corner_points.at(2).y);
      gDebug() << VAR(obst.corner_points.at(3).x, obst.corner_points.at(3).y);
    }
  }
  return obstacles;
}

inline std::vector<std::vector<cv::Point2f>>
Package2dBoxMarker(const std::vector<mybox> &box3ds) {
  std::vector<std::vector<cv::Point2f>> res;
  for (int i = 0; i < box3ds.size(); i++) {
    auto &box = box3ds[i];
    // 如果聚类失败了(点太少或者不满足要求)
    if (box.lidar_cluster->points.empty()) {
      continue;
    }
    res.push_back(box.position.vertices);
  }
  return res;
}