#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "yaml_config.h"

#include "obstacles.hpp"

using namespace lidar_obstacle_detection;


class PCLSubscriber {
public:
    PCLSubscriber(ros::NodeHandle& nh) {
        std::string sub_topic=yaml_config["sub_cloud_topic"].as<std::string>();
        std::cout << "订阅点云话题" << sub_topic << std::endl;
        std::string pub_topic=yaml_config["pub_cloud_topic"].as<std::string>();
        std::cout << "发布点云话题" << pub_topic << std::endl;
        sub_ = nh.subscribe<sensor_msgs::PointCloud2>(sub_topic, 10, &PCLSubscriber::pointCloudCallback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        processPointCloud(cloud);
        publishCloud(cloud);
    }

private:
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        // 在这里添加对点云数据的处理逻辑
        ROS_INFO("收到包含 %lu 个点的点云数据", cloud->size());
    }

    void publishCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        // 将 pcl::PointCloud 转换为 sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now();

        pub_.publish(msg);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_subscriber");
    ros::NodeHandle nh;

    PCLSubscriber subscriber(nh);

    ros::spin();
    return 0;
}
