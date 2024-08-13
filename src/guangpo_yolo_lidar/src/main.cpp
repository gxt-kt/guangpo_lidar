#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <geometry_msgs/PolygonStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <perception_msgs/PerceptionObstacle.h>
#include <perception_msgs/PerceptionObstacles.h>

#include "dbscan.hpp"
#include "debugstream.hpp"
#include "message.hpp"
#include "yaml_config.h"
#include "yolo_lidar_help.hpp"

std::string model_dianti_name = []() -> std::string {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path model_path =
      current_dir / "../model/RK3588/guangpo.rknn";
  return model_path;
}();
std::string model_other_name = []() -> std::string {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path model_path =
      current_dir / "../model/RK3588/yolov5s-640-640.rknn";
  return model_path;
}();
std::string input_path = []() -> std::string {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path img_path = current_dir / "../model/RK3588/889.jpg";
  return img_path;
}();

std::vector<std::string> labels_dianti = {"Dianti", "Ren"};
std::vector<std::string> labels_other = {
    "person",        "bicycle",      "car",
    "motorcycle",    "airplane",     "bus",
    "train",         "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench",        "bird",
    "cat",           "dog",          "horse",
    "sheep",         "cow",          "elephant",
    "bear",          "zebra",        "giraffe",
    "backpack",      "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",
    "skis",          "snowboard",    "sports ball",
    "kite",          "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket",
    "bottle",        "wine glass",   "cup",
    "fork",          "knife",        "spoon",
    "bowl",          "banana",       "apple",
    "sandwich",      "orange",       "broccoli",
    "carrot",        "hot dog",      "pizza",
    "donut",         "cake",         "chair",
    "couch",         "potted plant", "bed",
    "dining table",  "toilet",       "tv",
    "laptop",        "mouse",        "remote",
    "keyboard",      "cell phone",   "microwave",
    "oven",          "toaster",      "sink",
    "refrigerator",  "book",         "clock",
    "vase",          "scissors",     "teddy bear",
    "hair drier",    "toothbrush"};

#include "debugstream.hpp"
#include "dianti.h"
#include "yolov5.hpp"

class ImageReceiver {
public:
  Yolov5 yolo_dianti;
  Yolov5 yolo_other;
  std::shared_ptr<DistanceEstimator> estimator;
  ImageReceiver(ros::NodeHandle &nh) {
    img_width = yaml_config["image"]["width"].as<int>();
    img_height = yaml_config["image"]["height"].as<int>();
    img_distort_enable = yaml_config["img_distort_enable"].as<bool>();
    leaf_size = yaml_config["leaf_size"].as<double>();

    // 初始化yolo
    Init();
    yolo_dianti.SetLabels(labels_dianti);
    yolo_dianti.LoadModel(model_dianti_name);
    yolo_other.SetLabels(labels_other);
    yolo_other.LoadModel(model_other_name);

    pub_obstacles_ = nh.advertise<perception_msgs::PerceptionObstacles>(
        yaml_config["obstacles_topic_"].as<std::string>(), 10);

    person_density_pub_ = nh.advertise<std_msgs::Int8>(
        yaml_config["person_density_topic"].as<std::string>(), 10);

    sub_detect_congest_enable_ = nh.subscribe(
        yaml_config["detect_congest_enable_topic"].as<std::string>(), 10,
        &ImageReceiver::detectCongestCallback, this);

    // 初始化订阅话题和回调等
    std::string sub_image_topic =
        yaml_config["sub_image_topic"].as<std::string>();
    std::string sub_pointcloud_topic =
        yaml_config["sub_pointcloud_topic"].as<std::string>();
    std::string pub_cloud_noground_topic =
        yaml_config["pub_cloud_noground_topic"].as<std::string>();
    pub_cloud_noground_ =
        nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_noground_topic, 10);
    std::string pub_box3d_markers_topic =
        yaml_config["pub_box3d_markers_topic"].as<std::string>();
    pub_3dbox_ = nh.advertise<visualization_msgs::MarkerArray>(
        pub_box3d_markers_topic, 10);
    std::string send_image_topic =
        yaml_config["pub_image_topic"].as<std::string>();
    image_pub_ = nh.advertise<sensor_msgs::Image>(send_image_topic, 1);
    std::string send_lidar_topic =
        yaml_config["pub_lidar_topic"].as<std::string>();
    lidar_pub1_ = nh.advertise<sensor_msgs::PointCloud2>(send_lidar_topic, 1);
    lidar_pub2_ =
        nh.advertise<sensor_msgs::PointCloud2>(send_lidar_topic + "2", 1);
    gDebugCol3() << VAR(sub_image_topic, sub_pointcloud_topic);

    bool use_bag = yaml_config["use_bag"].as<bool>();
    gDebugCol3() << VAR(use_bag);

    if (use_bag) {
      image_sub_.subscribe(nh, sub_image_topic, 10);
      point_cloud_sub_.subscribe(nh, sub_pointcloud_topic, 10);
      sync_ = std::make_shared<Sync>(MySyncPolicy(500), image_sub_,
                                     point_cloud_sub_);
      double max_time_interval = yaml_config["max_time_interval"].as<double>();
      ros::Duration max_interval(max_time_interval);
      sync_->setMaxIntervalDuration(max_interval);
      sync_->registerCallback(
          boost::bind(&ImageReceiver::callback, this, _1, _2));
    } else {
      // my_image_sub_ =
      //     nh.subscribe(sub_image_topic, 10, &ImageReceiver::imageCallback,
      //     this);

      // 准备读取图像数据
      std::string camera_image_dev =
          yaml_config["camera_image_dev"].as<std::string>();
      // 打开视频设备
      cap = std::make_shared<cv::VideoCapture>(camera_image_dev);

      // 检查设备是否打开成功
      if (!cap->isOpened()) {
        std::cerr << "Failed to open video device!" << std::endl;
        std::terminate();
      }
      // 设置视频帧大小为 1280x720
      cap->set(cv::CAP_PROP_FRAME_WIDTH, img_width);
      cap->set(cv::CAP_PROP_FRAME_HEIGHT, img_height);

      my_pointcloud_sub_ = nh.subscribe(sub_pointcloud_topic, 10,
                                        &ImageReceiver::lidarCallback, this);
    }
  }
  ~ImageReceiver() {
    if (cap) {
      cap->release();
    }
  }
  void detectCongestCallback(const std_msgs::Bool::ConstPtr &msg) {
    ROS_INFO("Received /detect_congest_enable: %s",
             msg->data ? "true" : "false");
    enable_detect_congest = msg->data;
  }

  void SendImage(const cv::Mat &image) {
    // 将cv::Mat图像转换为sensor_msgs::Image消息
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = ros::Time::now();

    // 发布图像消息
    image_pub_.publish(msg);
    ROS_INFO("Image published!");
  }

  // 输入的点云是激光坐标系下的,发布的是车辆坐标系
  void SendLidar(ros::Publisher &pub,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr publish(
        new pcl::PointCloud<pcl::PointXYZ>);
    // 将点云进行转换
    pcl::transformPointCloud(*cloud, *publish, help.convert_lidar2car_affine_);

    // 发送点云数组
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*publish, msg);
    msg.header.frame_id = "map"; // 设置坐标系
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
  }

  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg) {
    ROS_INFO("Received lidar!");
    // 接收雷达后,需要从驱动读取图像 1280x720

    // 创建用于存储图像的 Mat 对象
    cv::Mat image;
    // 读取一帧图像
    if (!cap->read(image)) {
      std::cerr << "Failed to read frame from video device!" << std::endl;
      std::terminate();
    }

    // 将点云转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *cloud);

    // 调用统计接口
    CallBackImageAndLidar(image, cloud);
  }
  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    ROS_INFO("Received image!");
    // 在此处添加你的图像处理代码
    try {
      // 使用cv_bridge将sensor_msgs::Image转换为cv::Mat
      // cv_bridge::CvImagePtr cv_ptr =
      //     cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // cv::Mat image_ = cv_ptr->image;
      // DetectImage(image_);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  void PublishObstacles(
      const std::vector<Eigen::Vector3f> &obstacle_centroids,
      const std::vector<Eigen::Vector3f> &obstacle_sizes,
      const std::vector<Eigen::Quaternionf> &obstacle_orientations) {

    visualization_msgs::MarkerArray marker_array;
    // 首先发布一个DELETEALL命令,删除上次发布的所有障碍物标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    pub_3dbox_.publish(marker_array);
    marker_array.markers.clear(); // 清空marker_array

    for (size_t i = 0; i < obstacle_centroids.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
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

    pub_3dbox_.publish(marker_array);
  }
  void Publish2dObstacles(
      const std::vector<std::vector<cv::Point2f>> &obstacle_polygons) {
    visualization_msgs::MarkerArray marker_array;

    // 首先发布一个DELETEALL命令,删除上次发布的所有障碍物标记
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.action = visualization_msgs::Marker::LINE_STRIP;
    marker_array.markers.push_back(delete_marker);
    pub_3dbox_.publish(marker_array);
    marker_array.markers.clear(); // 清空marker_array

    for (size_t i = 0; i < obstacle_polygons.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.id = i;
      marker.scale.x = 0.1;
      marker.pose.orientation.w = 1.0; // 朝向为单位四元数
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.3;

      geometry_msgs::Point p;
      for (const auto &point : obstacle_polygons[i]) {
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0; // 2D 矩形,高度设为0
        marker.points.push_back(p);
      }
      p.x = obstacle_polygons[i][0].x;
      p.y = obstacle_polygons[i][0].y;
      p.z = 0.0; // 2D 矩形,高度设为0
      marker.points.push_back(p);

      marker_array.markers.push_back(marker);
    }

    pub_3dbox_.publish(marker_array);
  }

  void callback(const sensor_msgs::ImageConstPtr &image_msg,
                const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg) {
    // 在这里处理接收到的图像和点云数据
    // 打印时间戳（单位：ms）
    ROS_INFO("Image timestamp: %f ms", image_msg->header.stamp.toSec() * 1000);
    ROS_INFO("Point cloud timestamp: %f ms",
             point_cloud_msg->header.stamp.toSec() * 1000);

    // 将图像转换为cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;
    try {
      cv_ptr =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = cv_ptr->image;
      // 在这里可以对图像进行处理或者显示
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 将点云转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *cloud);

    // 调用统计接口
    CallBackImageAndLidar(image, cloud);
  }

  // 返回计算出来的相关联的人数
  int GetPersonNumber(const detect_result_group_t &detect_result_group) {
    int res{0};
    static double person_distance = yaml_config["person_distance"].as<double>();
    double x, y, w, h;
    std::string class_name;
    for (int i = 0; i < detect_result_group.count; i++) {
      const detect_result_t *det_result = &(detect_result_group.results[i]);
      x = (double)(det_result->box.left + det_result->box.right) / 2.0 /
          img_width;
      y = (double)(det_result->box.top + det_result->box.bottom) / 2.0 /
          img_height;
      w = (double)std::abs(det_result->box.right - det_result->box.left) /
          img_width;
      h = (double)std::abs(det_result->box.bottom - det_result->box.top) /
          img_height;
      class_name = det_result->name;
      if (class_name == "person") {
        const auto position_3d =
            estimator->Get3DPosition(x, y, w, h, class_name);
        double distance = position_3d.norm();
        if (distance <= person_distance) {
          res++;
        }
      }
    }
    return res;
  }

  // 统一接口
  void CallBackImageAndLidar(cv::Mat &image,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    TIME_BEGIN(CallBackImageAndLidar);

    TIME_BEGIN(preprecess);
    // 先对图像去畸变
    if (img_distort_enable) {
      help.UndistortImage(image);
    }
    // 对点云进行降采样
    if (leaf_size > 0) {
      // 创建 VoxelGrid 滤波器
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(leaf_size, leaf_size, leaf_size);
      vg.filter(*cloud);
    }
    TIME_END(preprecess);

    // yolo_dianti.Infer(image, "");
    detect_result_group_t detect_result_group;
    TIME_BEGIN(infer);
    detect_result_group = yolo_other.Infer(image, "");
    TIME_END(infer);

    TIME_BEGIN(send_image);
    SendImage(image);
    TIME_END(send_image);

    // 开启电梯拥挤度检测
    if (enable_detect_congest) {
      int person_density = GetPersonNumber(detect_result_group);
      gDebugCol4(person_density);
      // 发布person_density话题
      std_msgs::Int8 density_msg;
      density_msg.data = person_density;
      person_density_pub_.publish(density_msg);
    }

    // 开始计算3d障碍物
    TIME_BEGIN_MS(deal_lidar);

    // 如果需要去除地面的点云
    static bool remove_ground_points_enable =
        yaml_config["remove_ground_points_enable"].as<bool>();
    if (remove_ground_points_enable) {
      static int max_iterations = yaml_config["max_iterations"].as<double>();
      static double remove_ground_height =
          yaml_config["remove_ground_height"].as<double>();
      // gDebug() << VAR(max_iterations, remove_ground_height);
      auto [noground, ground] = RansacSegmentPlane<pcl::PointXYZ>(
          cloud, max_iterations, remove_ground_height);
      SendLidar(pub_cloud_noground_, noground);
      // 把cloud赋值成noground,后面全部基于非地面点云做
      *cloud = *noground;
    }

    gDebugWarn() << G_FILE_LINE;
    std::vector<mybox> box3ds;
    gDebugWarn() << G_FILE_LINE;
    for (int i = 0; i < detect_result_group.count; i++) {
      const detect_result_t *det_result = &(detect_result_group.results[i]);
      // box.x = (double)(det_result->box.left + det_result->box.right) / 2.0 /
      //         img_width;
      // box.y = (double)(det_result->box.top + det_result->box.bottom) / 2.0 /
      //         img_height;
      // box.w = (double)std::abs(det_result->box.right - det_result->box.left)
      // /
      //         img_width;
      // box.h = (double)std::abs(det_result->box.bottom - det_result->box.top)
      // /
      //         img_height;
      mybox box;
      box.rect = cv::Rect(det_result->box.left, det_result->box.top,
                          det_result->box.right - det_result->box.left,
                          det_result->box.bottom - det_result->box.top);
      static const double shrink_factor = 0.1;
      box.small_rect.x = box.rect.x + shrink_factor * box.rect.width / 2.0;
      box.small_rect.y = box.rect.y + shrink_factor * box.rect.height / 2.0;
      box.small_rect.width = box.rect.width * (1 - shrink_factor);
      box.small_rect.height = box.rect.height * (1 - shrink_factor);
      std::string class_name = det_result->name;
      if (class_name == "person" || class_name == "bicycle" ||
          class_name == "car" || class_name == "motorcycle" ||
          class_name == "cat" || class_name == "dog") {
        box.label = class_name;
        box3ds.push_back(box);
      }
    }
    gDebugWarn() << G_FILE_LINE;
    Eigen::Vector3d X;
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it =
             cloud->points.begin();
         it != cloud->points.end(); it++) {
      // if (it->x > 20 || it->x < 0.0 || std::abs(it->y) > 20) {
      //   continue;
      // }
      // 筛选掉一些不符合逻辑的点
      static double min_x = yaml_config["lidar2camera"]["min_x"].as<double>();
      static double max_x = yaml_config["lidar2camera"]["max_x"].as<double>();
      static double min_y = yaml_config["lidar2camera"]["min_y"].as<double>();
      static double max_y = yaml_config["lidar2camera"]["max_y"].as<double>();
      static double min_z = yaml_config["lidar2camera"]["min_z"].as<double>();
      static double max_z = yaml_config["lidar2camera"]["max_z"].as<double>();
      if (it->x < min_x || it->x > max_x || it->x < min_y || it->x > max_y ||
          it->x < min_z || it->x > max_z) {
        continue;
      }

      X[0] = it->x;
      X[1] = it->y;
      X[2] = it->z;

      auto Y = help.ProjectLidarPointToCameraPixel(X);

      // 计算出的点云在像素平面上的点
      cv::Point pt;
      pt.x = Y[0] / Y[2];
      pt.y = Y[1] / Y[2];

      int only_one_box_contain = 0;
      int box_id = 0;
      for (int i = 0; i < box3ds.size(); i++) {
        if (box3ds[i].small_rect.contains(pt)) {
          box_id = i;
          ++only_one_box_contain;
          if (only_one_box_contain > 1) {
            break;
          }
        }
      }
      if (only_one_box_contain == 1) {
        box3ds.at(box_id).lidar_points->push_back(*it);
      }
      // float val = it->x;
      // float maxVal = 3;
      // if (val < maxVal) {
      //   cv::circle(image, pt, 1, cv::Scalar(0, 255, 0), -1);
      // } else {
      //   cv::circle(image, pt, 1, cv::Scalar(0, 0, 255), -1);
      // }
      // int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
      // int green =
      //     std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
      // cv::circle(image, pt, 1, cv::Scalar(0, green, red), -1);
    }

    gDebugWarn() << G_FILE_LINE;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_lidar_points(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < box3ds.size(); i++) {
      const auto &box = box3ds[i];
      *combined_lidar_points += *box.lidar_points;
    }
    SendLidar(lidar_pub1_, combined_lidar_points);

    gDebugWarn() << G_FILE_LINE;
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_lidar_points2(
        new pcl::PointCloud<pcl::PointXYZ>);
    // 点云聚类
    for (int i = 0; i < box3ds.size(); i++) {
      auto &box = box3ds[i];
      box.lidar_cluster = EuclideanCluster(box.lidar_points);
      *combined_lidar_points2 += *box.lidar_cluster;
    }
    SendLidar(lidar_pub2_, combined_lidar_points2);

    // 暂时不需要3dbox
    // gDebugWarn() << G_FILE_LINE;
    // // 把点云提取出3dbox
    // static int obb_omp_threads = yaml_config["obb_omp_threads"].as<int>();
    // // omp_set_num_threads(obb_omp_threads);
    // // #pragma omp parallel for
    // for (int i = 0; i < box3ds.size(); i++) {
    //   auto &box = box3ds[i];
    //   ExtractObstacles(box.lidar_cluster, box.obstacle_centroids,
    //                    box.obstacle_sizes, box.obstacle_orientations, true);
    // }
    // gDebugWarn() << G_FILE_LINE;
    // std::vector<Eigen::Vector3f> obstacle_centroids;
    // std::vector<Eigen::Vector3f> obstacle_sizes;
    // std::vector<Eigen::Quaternionf> obstacle_orientations;
    // for (int i = 0; i < box3ds.size(); i++) {
    //   auto &box = box3ds[i];
    //   // 如果聚类失败了(点太少或者不满足要求)
    //   if (box.lidar_cluster->points.empty()) {
    //     continue;
    //   }
    //   obstacle_centroids.push_back(box.obstacle_centroids);
    //   obstacle_sizes.push_back(box.obstacle_sizes);
    //   obstacle_orientations.push_back(box.obstacle_orientations);
    // }
    // gDebugWarn() << G_FILE_LINE;
    // PublishObstacles(obstacle_centroids, obstacle_sizes,
    // obstacle_orientations);

    // 先把点云转到车体坐标系
    for (int i = 0; i < box3ds.size(); i++) {
      auto &box = box3ds[i];
      pcl::transformPointCloud(*box.lidar_cluster, *box.lidar_cluster,
                               help.convert_lidar2car_affine_);
      // 如果聚类失败了(点太少或者不满足要求)
      if (box.lidar_cluster->points.empty()) {
        continue;
      }
      box.position = GetMinAreaRectInfoFrom3DCloud(box.lidar_cluster);
      // // 输出最小包围矩形的信息
      // std::cout << "Center: (" << rect.center.x << ", " << rect.center.y <<
      // ")"
      //           << std::endl;
      // std::cout << "Size: (" << rect.size.width << ", " << rect.size.height
      //           << ")" << std::endl;
      // std::cout << "Angle: " << rect.angle << " degrees" << std::endl;
    }
    auto obstacles_msg = PackageMessage(box3ds, true);
    pub_obstacles_.publish(obstacles_msg);

    // 发布可视化内容
    auto packages_marker = Package2dBoxMarker(box3ds);
    Publish2dObstacles(packages_marker);
  }

  void Init() {
    Eigen::Matrix3d camera_matrix;
    camera_matrix << yaml_config["camera_in_param"]["dx"].as<double>(), 0.0,
        yaml_config["camera_in_param"]["u0"].as<double>(), 0.0,
        yaml_config["camera_in_param"]["dy"].as<double>(),
        yaml_config["camera_in_param"]["v0"].as<double>(), 0.0, 0.0, 1.0;
    gDebug(camera_matrix);
    estimator = std::make_shared<DistanceEstimator>(camera_matrix, img_width,
                                                    img_height);
  }

  void DetectImage(cv::Mat &img) {
    TIME_BEGIN_MS(DetectImage);
    detect_result_group_t detect_result_group;
    detect_result_group = yolo_dianti.Infer(img, "");

    TIME_END(DetectImage);
    int img_width = img.cols;
    int img_height = img.rows;

    // gxt: add my process here ==============begin
    // 相机内参
    std::vector<Box> rens;
    std::vector<Box> diantis;
    gDebug(detect_result_group.count);
    for (int i = 0; i < detect_result_group.count; i++) {
      detect_result_t *det_result = &(detect_result_group.results[i]);
      Box box;
      box.x = (double)(det_result->box.left + det_result->box.right) / 2.0 /
              img_width;
      box.y = (double)(det_result->box.top + det_result->box.bottom) / 2.0 /
              img_height;
      box.w = (double)std::abs(det_result->box.right - det_result->box.left) /
              img_width;
      box.h = (double)std::abs(det_result->box.bottom - det_result->box.top) /
              img_height;
      std::string class_name = det_result->name;
      if (class_name == "Dianti") {
        diantis.push_back(box);
      } else if (class_name == "Ren") {
        rens.push_back(box);
      }
    }
    TIME_END(DetectImage);
    DealImage(*estimator, img, rens, diantis);
    TIME_END(DetectImage);
    // gxt: add my process here ==============end
    SendImage(img);

#if DEBUG
    printf("save detect result to %s\n", out_path.c_str());
    imwrite(out_path, img);
#endif
  }

public:
  CoordinateHelp help;

private:
  int img_width;
  int img_height;
  double leaf_size;
  bool img_distort_enable = true;
  ros::Subscriber my_image_sub_;
  ros::Subscriber my_pointcloud_sub_;
  // 订阅话题/detect_congest_enable
  ros::Subscriber sub_detect_congest_enable_;
  bool enable_detect_congest = true;
  ros::Publisher person_density_pub_;
  ros::Publisher pub_obstacles_;

  ros::Publisher image_pub_;
  ros::Publisher lidar_pub1_;
  ros::Publisher lidar_pub2_;
  ros::Publisher pub_cloud_noground_;
  ros::Publisher pub_3dbox_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
  // 订阅图像和点云话题
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::PointCloud2>
      MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  std::shared_ptr<cv::VideoCapture> cap;
};

/*-------------------------------------------
                  Main Functions
-------------------------------------------*/
int main(int argc, char **argv) {

  ros::init(argc, argv, "image_receiver");
  ros::NodeHandle nh;
  ImageReceiver image_receiver(nh);

  // 把它当作warmup了
  if (yaml_config["warmup"].as<bool>()) {
    cv::Mat image = cv::imread(input_path);
    for (int n = 10; n >= 0; n--) {
      image_receiver.DetectImage(image);
    }
  }

  ros::spin();

  return 0;
}