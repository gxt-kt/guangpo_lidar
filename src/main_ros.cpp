#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "processPointClouds.h"
#include "render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "yaml_config.h"

#include "obstacles.hpp"

using namespace lidar_obstacle_detection;

template <typename PointT = pcl::PointXYZI>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol) {
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

  PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());
  PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());

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

  bool if_view_segment = yaml_config["view"]["segment"].as<bool>();
  if (if_view_segment) {
    // 创建可视化窗口
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("RANSAC Plane Segmentation"));
    viewer->setWindowName("My Plane Segmentation Viewer");
    viewer->setBackgroundColor(0, 0, 0);

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    // 添加栅格
    for (float x = -10.0; x <= 10.0; x += 1.0) {
      viewer->addLine(pcl::PointXYZ(x, -10, 0), pcl::PointXYZ(x, 10, 0), 0.5,
                      0.5, 0.5, std::string("grid_") + std::to_string(x) + "y");
    }
    for (float y = -10.0; y <= 10.0; y += 1.0) {
      viewer->addLine(pcl::PointXYZ(-10, y, 0), pcl::PointXYZ(10, y, 0), 0.5,
                      0.5, 0.5, std::string("grid_") + std::to_string(y) + "x");
    }

    // 可视化地面点云
    pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_color(
        in_plane, 255, 0, 0);
    viewer->addPointCloud<PointT>(in_plane, plane_color, "plane");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane");

    // 可视化非地面点云
    pcl::visualization::PointCloudColorHandlerCustom<PointT> non_plane_color(
        out_plane, 0, 255, 0);
    viewer->addPointCloud<PointT>(out_plane, non_plane_color, "non_plane");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "non_plane");

    // 显示可视化窗口
    while (!viewer->wasStopped()) {
      viewer->spinOnce();
    }
  }
  return std::make_pair(out_plane, in_plane);
}

template <typename PointT = pcl::PointXYZI>
pcl::PointCloud<pcl::PointXYZI>::Ptr
FilterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float filterRes,
            const Eigen::Vector4f &minPoint, const Eigen::Vector4f &maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering Create the filtering object: downsample the dataset using a
  // leaf size of .2m 体素降采样
  pcl::VoxelGrid<PointT> vg;
  PtCdtr<PointT> cloudFiltered(new pcl::PointCloud<PointT>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  // 设置roi区域
  PtCdtr<PointT> cloudRegion(new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  // 去除屋顶点云
  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  // roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  // roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  double max_x = yaml_config["roof"]["max_x"].as<double>();
  double max_y = yaml_config["roof"]["max_y"].as<double>();
  double max_z = yaml_config["roof"]["max_z"].as<double>();
  double min_x = yaml_config["roof"]["min_x"].as<double>();
  double min_y = yaml_config["roof"]["min_y"].as<double>();
  double min_z = yaml_config["roof"]["min_z"].as<double>();
  roof.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1));
  roof.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1));
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices) {
    inliers->indices.push_back(point);
  }
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  //    return cloud;
  return cloudRegion;
}

class PCLSubscriber {
public:
  PCLSubscriber(ros::NodeHandle &nh) {
    std::string sub_topic = yaml_config["sub_cloud_topic"].as<std::string>();
    std::cout << "订阅点云话题" << sub_topic << std::endl;
    std::string pub_cloud_ground_topic =
        yaml_config["pub_cloud_ground_topic"].as<std::string>();
    std::cout << "发布地面点云话题" << pub_cloud_ground_topic << std::endl;
    std::string pub_cloud_noground_topic =
        yaml_config["pub_cloud_noground_topic"].as<std::string>();
    std::cout << "发布非地面点云话题" << pub_cloud_ground_topic << std::endl;
    std::string markder_pub_topic =
        yaml_config["obstacles_topic"].as<std::string>();
    std::cout << "发布障碍物可视化话题" << markder_pub_topic << std::endl;
    sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
        sub_topic, 10, &PCLSubscriber::pointCloudCallback, this);
    pub_cloud_ground_ =
        nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_ground_topic, 10);
    pub_cloud_noground_ =
        nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_noground_topic, 10);
    marker_pub_ =
        nh.advertise<visualization_msgs::MarkerArray>(markder_pub_topic, 10);

    // 初始化坐标系变换的旋转矩阵
    Eigen::Vector3d trans;
    trans[0] = yaml_config["convert"]["transform"]["x"].as<double>();
    trans[1] = yaml_config["convert"]["transform"]["y"].as<double>();
    trans[2] = yaml_config["convert"]["transform"]["z"].as<double>();
    std::cout << "trans x=" << trans[0] << std::endl;
    std::cout << "trans y=" << trans[1] << std::endl;
    std::cout << "trans z=" << trans[2] << std::endl;
    // trans<<0.3643,0.2078,1.21;
    double roll = yaml_config["convert"]["rotation"]["roll"].as<double>();
    double pitch = yaml_config["convert"]["rotation"]["pitch"].as<double>();
    double yaw = yaml_config["convert"]["rotation"]["yaw"].as<double>();
    std::cout << "roll=" << roll << std::endl;
    std::cout << "pitch=" << pitch << std::endl;
    std::cout << "yaw=" << yaw << std::endl;
    Eigen::Matrix4d tran_matrix = TransforMatrix(
        trans, roll * M_PI / 180.0, pitch * M_PI / 180.0, yaw * M_PI / 180.0);
    Eigen::Affine3d affine3d_transform(tran_matrix); // 转换为Affine3d类型
    affine3f_transform = affine3d_transform.cast<float>(); // 转换为Affine3f类型
    affine3f_transform = affine3f_transform.inverse();
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    processPointCloud(cloud);
  }

private:
  void processPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    // 在这里添加对点云数据的处理逻辑
    ROS_INFO("收到包含 %lu 个点的点云数据", cloud->size());

    // 将点云进行转换
    pcl::transformPointCloud(*cloud, *cloud, affine3f_transform);

    auto startTime = std::chrono::steady_clock::now();
    // 对点云进行处理
    DealWithCloud(cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "All operators took " << elapsedTime.count()
              << " milliseconds" << std::endl;
  }

  // Test read Lidar data
  void DealWithCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Setting hyper parameters

    // FilterCloud
    // float filterRes = 0.4;
    float filterRes = yaml_config["filterRes"].as<float>();
    // Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    // Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    double max_x = yaml_config["roi"]["max_x"].as<double>();
    double max_y = yaml_config["roi"]["max_y"].as<double>();
    double max_z = yaml_config["roi"]["max_z"].as<double>();
    double min_x = yaml_config["roi"]["min_x"].as<double>();
    double min_y = yaml_config["roi"]["min_y"].as<double>();
    double min_z = yaml_config["roi"]["min_z"].as<double>();
    Eigen::Vector4f maxpoint(max_x, max_y, max_z, 1);
    Eigen::Vector4f minpoint(min_x, min_y, min_z, 1);
    // SegmentPlane
    // int maxIterations = 40;
    int maxIterations = yaml_config["ransac"]["maxIterations"].as<int>();
    // float distanceThreshold = 0.3;
    float distanceThreshold =
        yaml_config["ransac"]["distanceThreshold"].as<float>();
    // 聚类参数设置
    // Clustering
    // float clusterTolerance = 0.5;
    // int minsize = 10;
    // int maxsize = 140;
    float clusterTolerance =
        yaml_config["cluster"]["clusterTolerance"].as<float>();
    int minsize = yaml_config["cluster"]["minsize"].as<int>();
    int maxsize = yaml_config["cluster"]["maxsize"].as<int>();

    // 1. 降采样+设置roi区域+去除车辆相关点云
    // First:Filter cloud to reduce amount of points
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
        FilterCloud(inputCloud, filterRes, minpoint, maxpoint);
    // 2. 把点云分离出路面
    // Second: Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              pcl::PointCloud<pcl::PointXYZI>::Ptr>
        segmentCloud =
            RansacSegmentPlane(filteredCloud, maxIterations, distanceThreshold);

    // 发布点云
    ROS_INFO("发布点云cloud");
    PublishGroundCloud(segmentCloud.second);
    PublishNoGroundCloud(segmentCloud.first);
    //    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0,
    //    0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1,
    // 0));
    //    renderPointCloud(viewer,inputCloud,"inputCloud");
    // Third: Cluster different obstacle cloud
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
    // pointProcessorI->EuclideanClustering(segmentCloud.first,
    // clusterTolerance, minsize, maxsize); 提取出障碍物的点云
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
        GxtEuclideanClustering(segmentCloud.first, clusterTolerance, minsize,
                               maxsize);

    std::cout << "begin ExtractObstacles" << std::endl;
    std::vector<Eigen::Vector3f> centroids;
    std::vector<Eigen::Vector3f> sizes;
    std::vector<Eigen::Quaternionf> orientations;
    bool enable_verbose = yaml_config["log"]["enable_obstacles"].as<bool>();
    // 根据障碍物点云提取出框
    ExtractObstacles(cloudClusters, centroids, sizes, orientations,
                     enable_verbose);
    // visualizeObstacles(cloudClusters, centroids, sizes, orientations);

    // int clusterId = 0;
    // std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0,
    // 1)};

    // for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    //
    //   std::cout << "cluster size";
    //   pointProcessorI->numPoints(cluster);
    //   renderPointCloud(viewer, cluster, "obstCLoud" +
    //   std::to_string(clusterId),
    //                    colors[clusterId % colors.size()]);
    //   // Fourth: Find bounding boxes for each obstacle cluster
    //   Box box = pointProcessorI->BoundingBox(cluster);
    //   renderBox(viewer, box, clusterId);
    //   ++clusterId;
    // }
    PublishObstacles(centroids, sizes, orientations);
  }

  void PublishGroundCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    // 将 pcl::PointCloud 转换为 sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    pub_cloud_ground_.publish(msg);
  }
  void PublishNoGroundCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    // 将 pcl::PointCloud 转换为 sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    pub_cloud_noground_.publish(msg);
  }

  void PublishObstacles(
      const std::vector<Eigen::Vector3f> &obstacle_centroids,
      const std::vector<Eigen::Vector3f> &obstacle_sizes,
      const std::vector<Eigen::Quaternionf> &obstacle_orientations) {

    visualization_msgs::MarkerArray marker_array;
    // 首先发布一个DELETEALL命令,删除上次发布的所有障碍物标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "world";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    marker_pub_.publish(marker_array);
    marker_array.markers.clear(); // 清空marker_array

    for (size_t i = 0; i < obstacle_centroids.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = i;

      marker.pose.position.x = obstacle_centroids[i].x();
      marker.pose.position.y = obstacle_centroids[i].y();
      marker.pose.position.z = obstacle_centroids[i].z();

      marker.pose.orientation.x = obstacle_orientations[i].x();
      marker.pose.orientation.y = obstacle_orientations[i].y();
      marker.pose.orientation.z = obstacle_orientations[i].z();
      marker.pose.orientation.w = obstacle_orientations[i].w();

      marker.scale.x = obstacle_sizes[i].x();
      marker.scale.y = obstacle_sizes[i].y();
      marker.scale.z = obstacle_sizes[i].z();

      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.3;

      marker_array.markers.push_back(marker);
    }

    marker_pub_.publish(marker_array);
  }
  ros::Publisher marker_pub_;

  ros::Subscriber sub_;
  ros::Publisher pub_cloud_noground_;
  ros::Publisher pub_cloud_ground_;

  // 激光雷达到车体坐标系的变换
  Eigen::Affine3f affine3f_transform;
};

int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "pcl_subscriber");
  ros::NodeHandle nh;

  PCLSubscriber subscriber(nh);

  ros::spin();
  return 0;
}
