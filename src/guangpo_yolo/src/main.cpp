#include "debugstream.hpp"
#include "yaml_config.h"
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

std::vector<std::string> labels={"Dianti","Ren"};

#include "debugstream.hpp"
#include "dianti.h"
#include "yolov5.hpp"

class ImageReceiver {
public:
  Yolov5 yolo;
  std::shared_ptr<DistanceEstimator> estimator;
  ImageReceiver(ros::NodeHandle &nh) {
    std::string sub_topic = yaml_config["sub_image_topic"].as<std::string>();
    std::string send_topic = yaml_config["pub_image_topic"].as<std::string>();
    image_sub_ =
        nh.subscribe(sub_topic, 1, &ImageReceiver::imageCallback, this);
    image_pub_ = nh.advertise<sensor_msgs::Image>(send_topic, 1);

    Init();
    yolo.SetLabels(labels);
  yolo.LoadModel(model_name);
  }
  ~ImageReceiver() {}
  void SendImage(const cv::Mat &image) {
    // 将cv::Mat图像转换为sensor_msgs::Image消息
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = ros::Time::now();

    // 发布图像消息
    image_pub_.publish(msg);
    ROS_INFO("Image published!");
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    ROS_INFO("Received image!");
    // 在此处添加你的图像处理代码
    try {
      // 使用cv_bridge将sensor_msgs::Image转换为cv::Mat
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image_ = cv_ptr->image;
      DetectImage(image_);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void Init() {
    gDebug(model_name);

    Eigen::Matrix3d camera_matrix;
    camera_matrix << yaml_config["camera_in_param"]["dx"].as<double>(), 0.0,
        yaml_config["camera_in_param"]["u0"].as<double>(), 0.0,
        yaml_config["camera_in_param"]["dy"].as<double>(),
        yaml_config["camera_in_param"]["v0"].as<double>(), 0.0, 0.0, 1.0;
    gDebug(camera_matrix);
    estimator = std::make_shared<DistanceEstimator>(
        camera_matrix, yaml_config["image"]["width"].as<double>(),
        yaml_config["image"]["height"].as<double>());
  }

  void DetectImage(cv::Mat &img) {
    TIME_BEGIN_MS(DetectImage);
    detect_result_group_t detect_result_group;
    detect_result_group = yolo.Infer(img, "");

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

private:
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  cv::Mat image_;
};

/*-------------------------------------------
                  Main Functions
-------------------------------------------*/
int main(int argc, char **argv) {

  ros::init(argc, argv, "image_receiver");
  ros::NodeHandle nh;
  ImageReceiver image_receiver(nh);

  cv::Mat image = cv::imread(input_path);

  // 把它当作warmup了
  if (yaml_config["warmup"].as<bool>()) {
    for (int n = 10; n >= 0; n--) {
      image_receiver.DetectImage(image);
    }
  }

  ros::spin();

  return 0;
}