#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "yaml_config.h"

class ImageSaver {
 public:
  ImageSaver(ros::NodeHandle& nh) {
    std::string subscribe_topic =
        yaml_config["sub_image_topic"].as<std::string>();
    std::cout << "subscribe_topic=" << subscribe_topic << std::endl;
    // 订阅图像话题
    image_sub_ =
        nh.subscribe(subscribe_topic, 1, &ImageSaver::imageCallback, this);

    // 设置保存图像的路径
    save_path_ = yaml_config["save_path"].as<std::string>();
    if (save_path_.empty()) {
      std::filesystem::path current_dir =
          std::filesystem::path(__FILE__).parent_path().parent_path();
      std::filesystem::path yaml_path = current_dir / "images";
      save_path_ = yaml_path;
      if (!std::filesystem::exists(yaml_path)) {
        if (std::filesystem::create_directory(yaml_path)) {
          std::cout << "Created directory: " << yaml_path << std::endl;
        } else {
          std::cerr << "Failed to create directory: " << yaml_path << std::endl;
          std::terminate();
        }
      }
    }
    std::cout << "save_path=" << save_path_ << std::endl;
    frame_count_ = 0;

    save_interval_ = yaml_config["save_interval"].as<int>();
    std::cout << "save_interval=" << save_interval_ << std::endl;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 每3帧保存一张图像
    if (frame_count_ % save_interval_ == 0) {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      // 保存图像
      std::string file_name =
          save_path_ + "/" + "image_" + std::to_string(save_count_) + ".jpg";
      cv::imwrite(file_name, cv_ptr->image);
      save_count_++;
      ROS_INFO("Saved image: %s", file_name.c_str());
    }

    frame_count_++;
  }

 private:
  ros::Subscriber image_sub_;
  std::string save_path_;
  int frame_count_;
  int save_count_ = 0;
  int save_interval_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_saver");
  ros::NodeHandle nh;

  ImageSaver image_saver(nh);

  ros::spin();

  return 0;
}
