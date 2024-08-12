#pragma once

#include "debugstream.hpp"
// #include "include/dbscan/dbScan.h"
#include "yaml_config.h"

// inline pcl::PointCloud<pcl::PointXYZ>::Ptr
// DbScanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
//   dbScanSpace::dbscan dbscan;
//   std::vector<htr::Point3D> groupA;
//   // 创建一个新的 pcl::PointCloud<pcl::PointXYZRGB>::Ptr 类型的点云
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
//       new pcl::PointCloud<pcl::PointXYZRGB>);

//   // 遍历输入点云,并将每个点转换为 pcl::PointXYZRGB 类型
//   for (const auto &point : input_cloud->points) {
//     pcl::PointXYZRGB new_point;
//     new_point.x = point.x;
//     new_point.y = point.y;
//     new_point.z = point.z;
//     new_point.r = 255; // 设置 RGB 颜色
//     new_point.g = 255;
//     new_point.b = 255;
//     cloud->points.push_back(new_point);
//   }

//   // 设置输出点云的其他属性
//   cloud->width = cloud->points.size();
//   cloud->height = 1;
//   cloud->is_dense = true;
//   static int octreeResolution =
//       yaml_config["cluster"]["octreeResolution"].as<int>();
//   static float epsilon = yaml_config["cluster"]["epsilon"].as<float>();
//   static int minPtsAux = yaml_config["cluster"]["minPtsAux"].as<int>();
//   static int minPts = yaml_config["cluster"]["minPts"].as<int>();
//   dbscan.init(groupA, cloud, 120, 40, 5, 5);
//   dbscan.generateClusters();
//   gDebug(dbscan.getClusters().size());

//   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> store;
//   std::vector<dbScanSpace::cluster> cloud_cluster = dbscan.getClusters();

//   std::sort(cloud_cluster.begin(), cloud_cluster.end(),
//             [](dbScanSpace::cluster &l, dbScanSpace::cluster &r) {
//               return l.clusterPoints.size() > r.clusterPoints.size();
//             });

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
//       new pcl::PointCloud<pcl::PointXYZ>);
//   if (!cloud_cluster.empty()) {
//     auto &clusterPoints = cloud_cluster[0].clusterPoints;
//     cluster->points.resize(clusterPoints.size());
//     for (size_t i = 0; i < clusterPoints.size(); ++i) {
//       cluster->points[i].x = clusterPoints[i].x;
//       cluster->points[i].y = clusterPoints[i].y;
//       cluster->points[i].z = clusterPoints[i].z;
//     }
//     gDebugCol4() << VAR(input_cloud->points.size(), cluster->points.size());
//     return cluster;
//   } else {
//     gDebugCol4() << VAR(input_cloud->points.size(), "nullptr");
//     return cluster;
//   }

//   //   for (int i = 0; i < cloud_cluster.size(); i++) {
//   //     auto &clusterPoints = cloud_cluster[i].clusterPoints;
//   //     // 将 std::vector<pcl::PointXYZ> 拷贝到
//   pcl::PointCloud<pcl::PointXYZ>
//   //     cluster->points.resize(clusterPoints.size());
//   //     for (size_t i = 0; i < clusterPoints.size(); ++i) {
//   //       cluster->points[i].x = clusterPoints[i].x;
//   //       cluster->points[i].y = clusterPoints[i].y;
//   //       cluster->points[i].z = clusterPoints[i].z;
//   //     }
//   //     store.emplace_back(cluster);
//   //     cluster->points.clear();
//   //   }
//   //   return res;
// }

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

inline pcl::PointCloud<pcl::PointXYZ>::Ptr
EuclideanCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud) {
  // Create a kdtree for the input cloud
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  // Extract the Euclidean clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(
      static_cast<double>(yaml_config["cluster"]["epsilon"].as<float>()));
  ec.setMinClusterSize(
      static_cast<int>(yaml_config["cluster"]["minPts"].as<int>()));
  ec.setMaxClusterSize(input_cloud->size());
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  ec.extract(cluster_indices);

  // Find the largest cluster
  size_t largest_cluster_size = 0;
  size_t largest_cluster_index = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > largest_cluster_size) {
      largest_cluster_size = cluster_indices[i].indices.size();
      largest_cluster_index = i;
    }
  }

  // Create a new point cloud for the largest cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (!cluster_indices.empty()) {
    for (int index : cluster_indices[largest_cluster_index].indices) {
      largest_cluster->push_back(input_cloud->at(index));
    }
  }
  largest_cluster->width = largest_cluster->points.size();
  largest_cluster->height = 1;
  largest_cluster->is_dense = true;

  gDebugCol4() << VAR(input_cloud->points.size(),
                      largest_cluster->points.size());
  return largest_cluster;
}

/**
 * 使用欧式聚类对点云进行分割
 * @param input_cloud 输入点云
//  * @param cluster_tolerance 聚类距离阈值(单位:米)
//  * @param min_cluster_size 最小聚类尺寸
//  * @param max_cluster_size 最大聚类尺寸
 * @param output_clouds 输出聚类结果点云向量
 */
template <typename PointT = pcl::PointXYZ>
typename pcl::PointCloud<PointT>::Ptr
GxtEuclideanClustering(typename pcl::PointCloud<PointT>::Ptr input_cloud) {
  auto startTime = std::chrono::steady_clock::now();
  // 创建欧式聚类对象
  pcl::EuclideanClusterExtraction<PointT> ec;

  static double cluster_tolerance = static_cast<double>(
      yaml_config["cluster"]["cluster_tolerance"].as<float>());
  static double min_cluster_size = static_cast<double>(
      yaml_config["cluster"]["min_cluster_size"].as<float>());
  static double max_cluster_size = static_cast<double>(
      yaml_config["cluster"]["max_cluster_size"].as<float>());
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(
      pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<PointT>));

  // 提取聚类
  std::vector<pcl::PointIndices> cluster_indices;
  ec.setInputCloud(input_cloud);
  ec.extract(cluster_indices);

  size_t largest_cluster_size = 0;
  size_t largest_cluster_index = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > largest_cluster_size) {
      largest_cluster_size = cluster_indices[i].indices.size();
      largest_cluster_index = i;
    }
  }

  // 生成聚类结果点云
  typename pcl::PointCloud<PointT>::Ptr output_clouds;

  for (int idx : cluster_indices[largest_cluster_index].indices) {
    output_clouds->points.push_back(input_cloud->points[idx]);
  }
  output_clouds->width = output_clouds->points.size();
  output_clouds->height = 1;
  output_clouds->is_dense = true;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "euclidean clustering took " << elapsedTime.count()
            << " milliseconds" << std::endl;
  return output_clouds;
}