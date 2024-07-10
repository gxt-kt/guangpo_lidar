#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "yaml_config.h"

/**
 * 使用欧式聚类对点云进行分割
 * @param input_cloud 输入点云
 * @param cluster_tolerance 聚类距离阈值(单位:米)
 * @param min_cluster_size 最小聚类尺寸
 * @param max_cluster_size 最大聚类尺寸
 * @param output_clouds 输出聚类结果点云向量
 */
template <typename PointT = pcl::PointXYZI>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
GxtEuclideanClustering(typename pcl::PointCloud<PointT>::Ptr input_cloud,
                       float cluster_tolerance, int min_cluster_size,
                       int max_cluster_size) {
  auto startTime = std::chrono::steady_clock::now();
  // 创建欧式聚类对象
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(pcl::search::KdTree<pcl::PointXYZI>::Ptr(
      new pcl::search::KdTree<PointT>));

  // 提取聚类
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setInputCloud(input_cloud);
  ec.extract(cluster_indices);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> output_clouds;
  // 生成聚类结果点云
  output_clouds.clear();
  for (const auto &indices : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
    for (int index : indices.indices) {
      cluster->push_back(input_cloud->at(index));
    }
    output_clouds.push_back(cluster);
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "euclidean clustering took " << elapsedTime.count() << " milliseconds"
            << std::endl;
  if (false) {
    // 可视化聚类结果
    pcl::visualization::PCLVisualizer viewer("Euclidean Clustering");
    viewer.setWindowName("My Plane Segmentation Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    int j = 0;
    for (const auto &indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(
          new pcl::PointCloud<pcl::PointXYZI>);
      for (int index : indices.indices) {
        cluster->push_back(input_cloud->at(index));
      }
      viewer.addPointCloud<pcl::PointXYZI>(cluster,
                                           "cluster_" + std::to_string(j++));
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
          "cluster_" + std::to_string(j - 1));
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          (float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX,
          (float)rand() / (float)RAND_MAX, "cluster_" + std::to_string(j - 1));
    }
    viewer.spin();
    // // 显示可视化窗口
    // while (!viewer.wasStopped()) {
    //   viewer.spinOnce();
    // }
  }
  return output_clouds;
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
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &obstacle_clouds,
    std::vector<Eigen::Vector3f> &obstacle_centroids,
    std::vector<Eigen::Vector3f> &obstacle_sizes,
    std::vector<Eigen::Quaternionf> &obstacle_orientations,
    bool enable_verbose = false) {
  auto startTime = std::chrono::steady_clock::now();
  obstacle_centroids.clear();
  obstacle_sizes.clear();
  obstacle_orientations.clear();
  
  int obb_omp_threads = yaml_config["obb"]["threads"].as<int>();
  omp_set_num_threads(obb_omp_threads);
  #pragma omp parallel for
  for (const auto &obstacle_cloud : obstacle_clouds) {
    pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(obstacle_cloud);
    feature_extractor.compute();

    pcl::PointXYZI min_point_OBB;
    pcl::PointXYZI max_point_OBB;
    pcl::PointXYZI position_OBB;
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
    // 可视化障碍物框
    if (false) {
      pcl::visualization::PCLVisualizer::Ptr viewer(
          new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor(0, 0, 0);
      viewer->addCoordinateSystem(1.0);
      viewer->initCameraParameters();
      viewer->addPointCloud<pcl::PointXYZI>(obstacle_cloud, "sample cloud");

      // 添加可视化
      viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x,
                      max_point_OBB.y - min_point_OBB.y,
                      max_point_OBB.z - min_point_OBB.z, "OBB");
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
    }
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "extract obstacles took " << elapsedTime.count() << " milliseconds"
            << std::endl;

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
 * 可视化障碍物
 * @param obstacle_clouds 障碍物点云集合
 * @param obstacle_centroids 障碍物质心集合
 * @param obstacle_sizes 障碍物尺寸集合
 * @param obstacle_orientations 障碍物旋转集合
 */
inline void visualizeObstacles(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloudClusters,
    const std::vector<Eigen::Vector3f> &obstacle_centroids,
    const std::vector<Eigen::Vector3f> &obstacle_sizes,
    const std::vector<Eigen::Quaternionf> &obstacle_orientations) {
  // 创建可视化窗口
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  // 绘制包围盒
  for (size_t i = 0; i < obstacle_centroids.size(); i++) {

    std::stringstream cloud_name;
    cloud_name << "cloud_" << i;
    viewer->addPointCloud<pcl::PointXYZI>(cloudClusters[i], cloud_name.str());

    std::stringstream box_name;
    box_name << "box_" << i;

    viewer->addCube(obstacle_centroids[i], obstacle_orientations[i],
                    obstacle_sizes[i][0], obstacle_sizes[i][1],
                    obstacle_sizes[i][2], box_name.str());
    // 设置颜色
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, box_name.str());
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        box_name.str());
  }

  // 启动可视化
  while (!viewer->wasStopped()) {
    viewer->spinOnce();
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// 计算旋转矩阵
inline Eigen::Matrix4d TransforMatrix(Eigen::Vector3d translation, double roll,
                                      double pitch, double yaw) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  Eigen::Matrix4d transformation_matrix =
      Eigen::Matrix4d::Identity(); // 初始化为单位矩阵
  transformation_matrix.block<3, 3>(0, 0) =
      q.toRotationMatrix(); // 设置旋转矩阵部分
  transformation_matrix.block<3, 1>(0, 3) = translation; // 设置平移向量部分
  std::cout << "旋转矩阵：" << std::endl << transformation_matrix << std::endl;
  return transformation_matrix;
}
