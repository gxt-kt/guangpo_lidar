#pragma once

#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "debugstream.hpp"
#include "yaml_config.h"
#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Dense>

// 计算旋转矩阵
inline Eigen::Matrix4d TransforMatrix(const Eigen::Vector3d &translation,
                                      double roll, double pitch, double yaw) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  Eigen::Matrix4d transformation_matrix =
      Eigen::Matrix4d::Identity(); // 初始化为单位矩阵
  transformation_matrix.block<3, 3>(0, 0) =
      q.toRotationMatrix(); // 设置旋转矩阵部分
  transformation_matrix.block<3, 1>(0, 3) = translation; // 设置平移向量部分
  //   std::cout << "旋转矩阵：" << std::endl << transformation_matrix <<
  //   std::endl;
  return transformation_matrix;
}

struct MinAreaRectInfo {
  cv::Point2f center;
  std::vector<cv::Point2f> vertices;
};

struct mybox {
  mybox()
      : lidar_points(new pcl::PointCloud<pcl::PointXYZ>),
        lidar_cluster(new pcl::PointCloud<pcl::PointXYZ>) {}

  std::string label;
  cv::Rect rect;
  cv::Rect small_rect;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cluster;
  Eigen::Vector3f obstacle_centroids;
  Eigen::Vector3f obstacle_sizes;
  Eigen::Quaternionf obstacle_orientations;
  // 对应到机体坐标系上的点
  MinAreaRectInfo position;

  // 拷贝构造函数
  mybox(const mybox &other)
      : label(other.label), rect(other.rect), small_rect(other.small_rect),
        position(other.position),
        lidar_points(new pcl::PointCloud<pcl::PointXYZ>(*other.lidar_points)),
        lidar_cluster(new pcl::PointCloud<pcl::PointXYZ>(*other.lidar_cluster)),
        obstacle_centroids(other.obstacle_centroids),
        obstacle_sizes(other.obstacle_sizes),
        obstacle_orientations(other.obstacle_orientations) {}

  // 拷贝赋值运算符
  mybox &operator=(const mybox &other) {
    if (this != &other) {
      label = other.label;
      rect = other.rect;
      small_rect = other.small_rect;

      position = other.position,
      lidar_points.reset(
          new pcl::PointCloud<pcl::PointXYZ>(*other.lidar_points));
      lidar_cluster.reset(
          new pcl::PointCloud<pcl::PointXYZ>(*other.lidar_cluster));
      obstacle_centroids = other.obstacle_centroids;
      obstacle_sizes = other.obstacle_sizes;
      obstacle_orientations = other.obstacle_orientations;
    }
    return *this;
  }

  // 移动构造函数
  mybox(mybox &&other) noexcept
      : label(std::move(other.label)), rect(std::move(other.rect)),

        position(std::move(other.position)),
        small_rect(std::move(other.small_rect)),
        lidar_points(std::move(other.lidar_points)),
        lidar_cluster(std::move(other.lidar_cluster)),
        obstacle_centroids(std::move(other.obstacle_centroids)),
        obstacle_sizes(std::move(other.obstacle_sizes)),
        obstacle_orientations(std::move(other.obstacle_orientations)) {}

  // 移动赋值运算符
  mybox &operator=(mybox &&other) noexcept {
    if (this != &other) {
      label = std::move(other.label);
      rect = std::move(other.rect);
      small_rect = std::move(other.small_rect);
      position = std::move(other.position),
      lidar_points = std::move(other.lidar_points);
      lidar_cluster = std::move(other.lidar_cluster);
      obstacle_centroids = std::move(other.obstacle_centroids);
      obstacle_sizes = std::move(other.obstacle_sizes);
      obstacle_orientations = std::move(other.obstacle_orientations);
    }
    return *this;
  }
};

// 获取一个3维点云的最小的矩形
inline MinAreaRectInfo GetMinAreaRectInfoFrom3DCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  std::vector<cv::Point2f> points;
  for (const auto &point : cloud->points) {
    points.emplace_back(point.x, point.y);
  }
  cv::RotatedRect rect = cv::minAreaRect(points);

  MinAreaRectInfo info;
  info.center = rect.center;

  cv::Point2f vertices[4];
  rect.points(vertices);
  for (int i = 0; i < 4; i++) {
    info.vertices.push_back(vertices[i]);
  }

  return info;
}

// 把点云去除地面相关点
// 返回值第一个是非地面点,第二个是地面点
template <typename PointT>
inline std::pair<typename pcl::PointCloud<PointT>::Ptr,
                 typename pcl::PointCloud<PointT>::Ptr>
RansacSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                   int maxIterations, float distanceTol) {
  // 使用pcl的分割，分离出地面
  // Count time
  auto startTime = std::chrono::steady_clock::now();

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;

  // Optional
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceTol);

  // Segment the largest planar component
  // PtCdtr<PointT> cloud_plane(new pcl::PointCloud<PointT>());
  // PtCdtr<PointT> cloud_out(new pcl::PointCloud<PointT>());
  pcl::ModelCoefficients::Ptr cloud_out(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr cloud_plane(new pcl::PointIndices);

  seg.setInputCloud(cloud);
  seg.segment(*cloud_plane, *cloud_out);

  typename pcl::PointCloud<PointT>::Ptr in_plane(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr out_plane(
      new pcl::PointCloud<PointT>());

  // Extract the inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(cloud_plane);
  extract.setNegative(false);
  extract.filter(*in_plane);

  // Extract the outliers
  extract.setNegative(true);
  extract.filter(*out_plane);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane ransac-segment took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return std::make_pair(out_plane, in_plane);
}

// reference
// https://pointclouds.org/documentation/tutorials/moment_of_inertia.html
/**
 * 从聚类结果中提取障碍物信息
 * @param obstacle_clouds 输出的障碍物点云集合
 * @param obstacle_centroids 输出的障碍物质心集合
 * @param obstacle_sizes 输出的障碍物尺寸集合
 * @param obstacle_orientations 输出的障碍物旋转集合
 */
inline void ExtractObstacles(
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &obstacle_clouds,
    std::vector<Eigen::Vector3f> &obstacle_centroids,
    std::vector<Eigen::Vector3f> &obstacle_sizes,
    std::vector<Eigen::Quaternionf> &obstacle_orientations,
    bool enable_verbose = false) {
  auto startTime = std::chrono::steady_clock::now();
  obstacle_centroids.clear();
  obstacle_sizes.clear();
  obstacle_orientations.clear();

  static int obb_omp_threads = yaml_config["obb_omp_threads"].as<int>();
  omp_set_num_threads(obb_omp_threads);
#pragma omp parallel for
  for (const auto &obstacle_cloud : obstacle_clouds) {
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(obstacle_cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                             rotational_matrix_OBB);

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f size;
    size << (max_point_OBB.x - min_point_OBB.x),
        (max_point_OBB.y - min_point_OBB.y),
        (max_point_OBB.z - min_point_OBB.z);
#pragma omp critical
    {
      obstacle_centroids.push_back(position);
      obstacle_orientations.push_back(quat);
      obstacle_sizes.push_back(size);
    }
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "extract obstacles took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  // debug
  if (enable_verbose) {
    std::cout << obstacle_centroids.size();
    for (int i = 0; i < obstacle_centroids.size(); i++) {
      std::cout << "======obstacle[" << i << "]======" << std::endl;
      std::cout << obstacle_centroids[i] << std::endl;
      std::cout << obstacle_sizes[i] << std::endl;
      std::cout << obstacle_orientations[i].matrix() << std::endl;
      std::cout << "========================" << std::endl;
    }
  }
}

/**
 * 从聚类结果中提取障碍物信息
 * @param obstacle_clouds 输出的障碍物点云集合
 * @param obstacle_centroids 输出的障碍物质心集合
 * @param obstacle_sizes 输出的障碍物尺寸集合
 * @param obstacle_orientations 输出的障碍物旋转集合
 */
inline void ExtractObstacles(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &obstacle_cloud,
    Eigen::Vector3f &obstacle_centroids, Eigen::Vector3f &obstacle_sizes,
    Eigen::Quaternionf &obstacle_orientations, bool enable_verbose = false) {
  auto startTime = std::chrono::steady_clock::now();

  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(obstacle_cloud);
  feature_extractor.compute();

  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;

  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                           rotational_matrix_OBB);

  Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat(rotational_matrix_OBB);
  Eigen::Vector3f size;
  size << (max_point_OBB.x - min_point_OBB.x),
      (max_point_OBB.y - min_point_OBB.y), (max_point_OBB.z - min_point_OBB.z);
  obstacle_centroids = std::move(position);
  obstacle_orientations = std::move(quat);
  obstacle_sizes = std::move(size);
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "extract obstacles took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  // debug
  if (enable_verbose) {
    std::cout << "========obstacle========" << std::endl;
    std::cout << obstacle_centroids << std::endl;
    std::cout << obstacle_sizes << std::endl;
    std::cout << obstacle_orientations.matrix() << std::endl;
    std::cout << "========================" << std::endl;
  }
}

class CoordinateHelp {
public:
  CoordinateHelp() {
    // 初始化相机内参矩阵
    camera_inner_matrix_ =
        (cv::Mat_<double>(3, 3)
             << yaml_config["camera_in_param"]["dx"].as<double>(),
         0.0, yaml_config["camera_in_param"]["u0"].as<double>(), 0.0,
         yaml_config["camera_in_param"]["dy"].as<double>(),
         yaml_config["camera_in_param"]["v0"].as<double>(), 0.0, 0.0, 1.0);
    gDebugCol5(camera_inner_matrix_);

    // 初始化相机畸变矩阵
    camera_distort_matrix_ =
        (cv::Mat_<double>(1, 5)
             << yaml_config["camera_distort_params"]["p0"].as<double>(),
         yaml_config["camera_distort_params"]["p1"].as<double>(),
         yaml_config["camera_distort_params"]["p2"].as<double>(),
         yaml_config["camera_distort_params"]["p3"].as<double>(),
         yaml_config["camera_distort_params"]["p4"].as<double>());
    gDebugCol5(camera_distort_matrix_);


    // 初始化雷达到车体位姿
    auto get = Lidar2Car();
    convert_lidar2car_matrix_ = get.first;
    convert_lidar2car_affine_ = get.second;
    gDebugCol5(convert_lidar2car_matrix_);
    gDebugCol5(convert_lidar2car_affine_);

    Eigen::Matrix3d camera_inner_eigen;
    cv::cv2eigen(camera_inner_matrix_, camera_inner_eigen);
    if(yaml_config["if_use_matrix"].as<bool>()) {
      Eigen::Matrix4d tmp;
      std::vector<float> ext= yaml_config["convert_carmera2lidar_matrix"].as<std::vector<float>>();
      gDebug(ext);
      assert(ext.size()==16);
      for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
          tmp(row, col) = ext.at(col + row * 4);
        }
      }
      gDebug(tmp);
      // std::terminate();
      // Eigen::Affine3d tmp2 = Eigen::Affine3d(tmp.inverse());
      Eigen::Affine3d tmp2 = Eigen::Affine3d(tmp);
      all_in_one_matrix_ = camera_inner_eigen * tmp2;
    } else {
      // 初始化相机到雷达位姿
      Eigen::Matrix4d convert_camera2lidar_ = Camera2Lidar();
      gDebugCol5(convert_camera2lidar_);

      Eigen::Affine3d convert_point_cl = Eigen::Affine3d(
          convert_camera2lidar_.inverse()); // 转换为Affine3d类型
      Eigen::Matrix3d xyz2zyx;
      xyz2zyx << 0, -1, 0, 0, 0, 1, 1, 0, 0;
      all_in_one_matrix_ = camera_inner_eigen * xyz2zyx * convert_point_cl;
    }
  }
  cv::Mat camera_inner_matrix_;              // 相机内参矩阵
  cv::Mat camera_distort_matrix_;            // 相机畸变参数矩阵
  Eigen::Matrix4d convert_lidar2car_matrix_; // 雷达到车的位姿转换
  Eigen::Affine3f convert_lidar2car_affine_; // 雷达到车的位姿转换

  Eigen::Affine3d all_in_one_matrix_; // 激光点到相机像素坐标系点
public:
  void UndistortImage(cv::Mat &image) {
    UndistortImageHelp(image, camera_inner_matrix_, camera_distort_matrix_);
  }
  // 点云点到相机像素像素平面
  Eigen::Vector3d
  ProjectLidarPointToCameraPixel(const Eigen::Vector3d &lidarPoint) {
    return all_in_one_matrix_ * lidarPoint;
  }

private:
  inline void UndistortImageHelp(cv::Mat &inputImage,
                                 const cv::Mat &cameraMatrix,
                                 const cv::Mat &distCoeffs) {
    static std::pair<cv::Mat, cv::Mat> undistort_map = [&]() {
      // 创建用于存储映射矩阵的 Mat
      cv::Mat map1, map2;
      // 计算并缓存去畸变映射
      cv::Size img_size(inputImage.cols, inputImage.rows); // 图像大小
      cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                  cameraMatrix, img_size, CV_16SC2, map1, map2);
      return std::pair<cv::Mat, cv::Mat>{map1, map2};
    }();
    cv::remap(inputImage, inputImage, undistort_map.first, undistort_map.second,
              cv::INTER_LINEAR);
  }

  Eigen::Matrix4d Camera2Lidar() {
    // 初始化坐标系变换的旋转矩阵
    Eigen::Vector3d trans;
    trans[0] =
        yaml_config["convert_carmera2lidar"]["transform"]["x"].as<double>();
    trans[1] =
        yaml_config["convert_carmera2lidar"]["transform"]["y"].as<double>();
    trans[2] =
        yaml_config["convert_carmera2lidar"]["transform"]["z"].as<double>();
    std::cout << "trans x=" << trans[0] << std::endl;
    std::cout << "trans y=" << trans[1] << std::endl;
    std::cout << "trans z=" << trans[2] << std::endl;
    // trans<<0.3643,0.2078,1.21;
    double roll =
        yaml_config["convert_carmera2lidar"]["rotation"]["roll"].as<double>();
    double pitch =
        yaml_config["convert_carmera2lidar"]["rotation"]["pitch"].as<double>();
    double yaw =
        yaml_config["convert_carmera2lidar"]["rotation"]["yaw"].as<double>();
    std::cout << "roll=" << roll << std::endl;
    std::cout << "pitch=" << pitch << std::endl;
    std::cout << "yaw=" << yaw << std::endl;
    Eigen::Matrix4d tran_matrix = TransforMatrix(
        trans, roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
    // trans, roll , pitch , yaw );
    gDebug() << VAR(tran_matrix);
    //   Eigen::Affine3d affine3d_transform(tran_matrix); // 转换为Affine3d类型
    //   affine3f_transform = affine3d_transform.cast<float>(); //
    //   转换为Affine3f类型 affine3f_transform = affine3f_transform.inverse();
    return tran_matrix;
  }
  inline std::pair<Eigen::Matrix4d, Eigen::Affine3f> Lidar2Car() {
    // 初始化坐标系变换的旋转矩阵
    Eigen::Vector3d trans;
    trans[0] = yaml_config["convert_lidar2car"]["transform"]["x"].as<double>();
    trans[1] = yaml_config["convert_lidar2car"]["transform"]["y"].as<double>();
    trans[2] = yaml_config["convert_lidar2car"]["transform"]["z"].as<double>();
    std::cout << "trans x=" << trans[0] << std::endl;
    std::cout << "trans y=" << trans[1] << std::endl;
    std::cout << "trans z=" << trans[2] << std::endl;
    // trans<<0.3643,0.2078,1.21;
    double roll =
        yaml_config["convert_lidar2car"]["rotation"]["roll"].as<double>();
    double pitch =
        yaml_config["convert_lidar2car"]["rotation"]["pitch"].as<double>();
    double yaw =
        yaml_config["convert_lidar2car"]["rotation"]["yaw"].as<double>();
    std::cout << "roll=" << roll << std::endl;
    std::cout << "pitch=" << pitch << std::endl;
    std::cout << "yaw=" << yaw << std::endl;
    Eigen::Matrix4d tran_matrix = TransforMatrix(
        trans, roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
    gDebug() << VAR(tran_matrix);
    Eigen::Affine3d affine3d_transform(tran_matrix); // 转换为Affine3d类型
    Eigen::Affine3f affine3f_transform = affine3d_transform.cast<float>(); //
    // 转换为Affine3f类型
    affine3f_transform = affine3f_transform.inverse();
    return {tran_matrix, affine3f_transform};
  }
};
