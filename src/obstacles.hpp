#pragma once

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// reference https://pointclouds.org/documentation/tutorials/moment_of_inertia.html
/**
 * 从聚类结果中提取障碍物信息
 * @param obstacle_clouds 输出的障碍物点云集合
 * @param obstacle_centroids 输出的障碍物质心集合
 * @param obstacle_sizes 输出的障碍物尺寸集合
 * @param obstacle_orientations 输出的障碍物旋转集合
 */
void ExtractObstacles(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& obstacle_clouds,
    std::vector<Eigen::Vector3f>& obstacle_centroids,
    std::vector<Eigen::Vector3f>& obstacle_sizes,
    std::vector<Eigen::Quaternionf>& obstacle_orientations)
{
    obstacle_centroids.clear();
    obstacle_sizes.clear();
    obstacle_orientations.clear();
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

      pcl::visualization::PCLVisualizer::Ptr viewer(
          new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor(0, 0, 0);
      viewer->addCoordinateSystem(1.0);
      viewer->initCameraParameters();
      viewer->addPointCloud<pcl::PointXYZI>(obstacle_cloud, "sample cloud");

      Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
      Eigen::Quaternionf quat(rotational_matrix_OBB);
      // 添加可视化
      // viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x,
      //                 max_point_OBB.y - min_point_OBB.y,
      //                 max_point_OBB.z - min_point_OBB.z, "OBB");
      // viewer->setShapeRenderingProperties(
      //     pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      //     pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
      obstacle_centroids.push_back(position);
      obstacle_orientations.push_back(quat);
      Eigen::Vector3f size;
      size << (max_point_OBB.x - min_point_OBB.x) ,(max_point_OBB.y - min_point_OBB.y),(max_point_OBB.z - min_point_OBB.z);
      obstacle_sizes.push_back(size);
    }

    // debug
    if(0) {
        std::cout << obstacle_centroids.size();
        for(int i=0;i < obstacle_centroids.size();i++) {
            std::cout << "====" << std::endl;
            std::cout << obstacle_centroids[i] << std::endl;
            std::cout << obstacle_sizes[i] << std::endl;
            // std::cout << obstacle_orientations[i] << std::endl;
        }
    }
}

/**
 * 可视化障碍物
 * @param viewer PCL可视化器
 * @param obstacle_clouds 障碍物点云集合
 * @param obstacle_centroids 障碍物质心集合
 * @param obstacle_sizes 障碍物尺寸集合
 * @param obstacle_orientations 障碍物旋转集合
 */
void visualizeObstacles(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloudClusters,
    const std::vector<Eigen::Vector3f>& obstacle_centroids,
    const std::vector<Eigen::Vector3f>& obstacle_sizes,
    const std::vector<Eigen::Quaternionf>& obstacle_orientations)
{
    // 创建可视化窗口
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 绘制包围盒
    for (size_t i = 0; i < obstacle_centroids.size(); i++)
    {
        
        std::stringstream cloud_name;
        cloud_name << "cloud_" << i;
        viewer->addPointCloud<pcl::PointXYZI>(cloudClusters[i], cloud_name.str());
        
        std::stringstream box_name;
        box_name << "box_" << i;
        
      viewer->addCube(obstacle_centroids[i], obstacle_orientations[i], obstacle_sizes[i][0],
                      obstacle_sizes[i][1],
                      obstacle_sizes[i][2], box_name.str());
         // 设置颜色
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                    0.5,0,0, box_name.str());  
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_name.str());
    }

    // 启动可视化
    while (!viewer->wasStopped()) {
      viewer->spinOnce();
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


// 计算旋转矩阵  
Eigen::Matrix4d TransforMatrix(Eigen::Vector3d translation,double roll,double pitch,double yaw) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())  
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())  
                         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());  
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵  
    transformation_matrix.block<3, 3>(0, 0) = q.toRotationMatrix(); // 设置旋转矩阵部分  
    transformation_matrix.block<3, 1>(0, 3) = translation; // 设置平移向量部分  
    std::cout << "旋转矩阵：" << std::endl << transformation_matrix << std::endl;  
    return transformation_matrix;
}
