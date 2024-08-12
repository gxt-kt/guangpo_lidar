#include "yolov5.hpp"
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

std::string model_name = []() -> std::string {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path model_path =
      current_dir / "../model/RK3588/guangpo.rknn";
  return model_path;
}();
std::string input_path = []() -> std::string {
  std::filesystem::path current_dir =
      std::filesystem::path(__FILE__).parent_path();
  std::filesystem::path img_path = current_dir / "../model/RK3588/889.jpg";
  return img_path;
}();

int main(int argc, char **argv) {
  ros::init(argc, argv, "main_test");
  ros::NodeHandle nh;

  Yolov5 yolo;
  std::cout << "yolo begin" << std::endl;
  yolo.SetLabels({"Dianti", "Ren"});
  yolo.LoadModel(model_name);

  cv::Mat image = cv::imread(input_path, 1);
  yolo.Infer(image, "output.jpg");

  ros::spin();

  return 0;
}
