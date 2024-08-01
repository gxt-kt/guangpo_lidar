#include "debugstream.hpp"
#include "yaml_config.h"
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#define DEBUG 0

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

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define _BASETSD_H

#include "RgaUtils.h"

#include "postprocess.h"

#include "preprocess.h"
#include "rknn_api.h"

#include "debugstream.hpp"
#include "dianti.h"

#define PERF_WITH_POST 1
/*-------------------------------------------
                  Functions
-------------------------------------------*/

int ret;
rknn_context ctx;

static void dump_tensor_attr(rknn_tensor_attr *attr) {
  std::string shape_str = attr->n_dims < 1 ? "" : std::to_string(attr->dims[0]);
  for (int i = 1; i < attr->n_dims; ++i) {
    shape_str += ", " + std::to_string(attr->dims[i]);
  }

  printf("  index=%d, name=%s, n_dims=%d, dims=[%s], n_elems=%d, size=%d, "
         "w_stride = %d, size_with_stride=%d, fmt=%s, "
         "type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, shape_str.c_str(),
         attr->n_elems, attr->size, attr->w_stride, attr->size_with_stride,
         get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz) {
  unsigned char *data;
  int ret;

  data = NULL;

  if (NULL == fp) {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0) {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char *)malloc(sz);
  if (data == NULL) {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

static unsigned char *load_model(const std::string &filename, int *model_size) {
  FILE *fp;
  unsigned char *data;

  fp = fopen(filename.c_str(), "rb");
  if (NULL == fp) {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}

static int saveFloat(const char *file_name, float *output, int element_size) {
  FILE *fp;
  fp = fopen(file_name, "w");
  for (int i = 0; i < element_size; i++) {
    fprintf(fp, "%.6f\n", output[i]);
  }
  fclose(fp);
  return 0;
}

class ImageReceiver {
public:
  unsigned char *model_data;
  std::string option = "letterbox";
  std::string out_path = "./out.jpg";
  int img_channel = 0;
  int channel = 3;
  int width = 0;
  int height = 0;
  rknn_input_output_num io_num;
  rknn_input inputs[1];
  rknn_tensor_attr *output_attrs;
  // init rga context
  rga_buffer_t src;
  rga_buffer_t dst;
  size_t actual_size = 0;
  const float nms_threshold = NMS_THRESH;      // 默认的NMS阈值
  const float box_conf_threshold = BOX_THRESH; // 默认的置信度阈值
  std::string sub_topic;
  std::string send_topic;
  DistanceEstimator *estimator;
  ImageReceiver(ros::NodeHandle &nh) {
    sub_topic = yaml_config["sub_image_topic"].as<std::string>();
    send_topic = yaml_config["pub_image_topic"].as<std::string>();
    image_sub_ =
        nh.subscribe(sub_topic, 1, &ImageReceiver::imageCallback, this);
    image_pub_ = nh.advertise<sensor_msgs::Image>(send_topic, 1);

    Init();
  }
  ~ImageReceiver() {
    deinitPostProcess();
    // release
    ret = rknn_destroy(ctx);

    if (model_data) {
      free(model_data);
    }
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

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    ROS_INFO("Received image!");
    // 在此处添加你的图像处理代码
    try {
      // 使用cv_bridge将sensor_msgs::Image转换为cv::Mat
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      image_ = cv_ptr->image;
      TIME_BEGIN_MS(detect_image);
      DetectImage(image_);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void Init() {
    gDebug(model_name);
    gDebug(input_path);

    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    printf("post process config: box_conf_threshold = %.2f, nms_threshold = "
           "%.2f\n",
           box_conf_threshold, nms_threshold);

    /* Create the neural network */
    printf("Loading mode...\n");
    int model_data_size = 0;
    model_data = load_model(model_name, &model_data_size);
    ret = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0) {
      printf("rknn_init error ret=%d\n", ret);
      return;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version,
                     sizeof(rknn_sdk_version));
    if (ret < 0) {
      printf("rknn_init error ret=%d\n", ret);
      return;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version,
           version.drv_version);

    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
      printf("rknn_init error ret=%d\n", ret);
      return;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input,
           io_num.n_output);

    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (int i = 0; i < io_num.n_input; i++) {
      input_attrs[i].index = i;
      ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]),
                       sizeof(rknn_tensor_attr));
      if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return;
      }
      dump_tensor_attr(&(input_attrs[i]));
    }

    // rknn_tensor_attr output_attrs[io_num.n_output];
    output_attrs = new rknn_tensor_attr[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (int i = 0; i < io_num.n_output; i++) {
      output_attrs[i].index = i;
      ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]),
                       sizeof(rknn_tensor_attr));
      dump_tensor_attr(&(output_attrs[i]));
    }

    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
      printf("model is NCHW input fmt\n");
      channel = input_attrs[0].dims[1];
      height = input_attrs[0].dims[2];
      width = input_attrs[0].dims[3];
    } else {
      printf("model is NHWC input fmt\n");
      height = input_attrs[0].dims[1];
      width = input_attrs[0].dims[2];
      channel = input_attrs[0].dims[3];
    }

    printf("model input height=%d, width=%d, channel=%d\n", height, width,
           channel);

    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8;
    inputs[0].size = width * height * channel;
    inputs[0].fmt = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    Eigen::Matrix3d camera_matrix;
    camera_matrix << yaml_config["camera_in_param"]["dx"].as<double>(), 0.0,
        yaml_config["camera_in_param"]["u0"].as<double>(), 0.0,
        yaml_config["camera_in_param"]["dy"].as<double>(),
        yaml_config["camera_in_param"]["v0"].as<double>(), 0.0, 0.0, 1.0;
    gDebug(camera_matrix);
    estimator = new DistanceEstimator(
        camera_matrix, yaml_config["image"]["width"].as<double>(),
        yaml_config["image"]["height"].as<double>());
  }

  void DetectImage(cv::Mat &img) {
    TIME_BEGIN_MS(DetectImage);
    // 读取图片
    // printf("Read %s ...\n", input_path);
    cv::Mat &orig_img = img;
    if (!orig_img.data) {
      printf("cv::imread %s fail!\n", input_path);
      return;
    }
    cv::cvtColor(orig_img, img, cv::COLOR_BGR2RGB);
    int img_width = img.cols;
    int img_height = img.rows;
    printf("img width = %d, img height = %d\n", img_width, img_height);

    // 指定目标大小和预处理方式,默认使用LetterBox的预处理
    BOX_RECT pads;
    memset(&pads, 0, sizeof(BOX_RECT));
    cv::Size target_size(width, height);
    cv::Mat resized_img(target_size.height, target_size.width, CV_8UC3);
    // 计算缩放比例
    float scale_w = (float)target_size.width / img.cols;
    float scale_h = (float)target_size.height / img.rows;

    if (img_width != width || img_height != height) {
      // 直接缩放采用RGA加速
      if (option == "resize") {
        printf("resize image by rga\n");
        ret = resize_rga(src, dst, img, resized_img, target_size);
        if (ret != 0) {
          fprintf(stderr, "resize with rga error\n");
          return;
        }
// 保存预处理图片
#if DEBUG
        cv::imwrite("resize_input.jpg", resized_img);
#endif
      } else if (option == "letterbox") {
        printf("resize image with letterbox\n");
        float min_scale = std::min(scale_w, scale_h);
        scale_w = min_scale;
        scale_h = min_scale;
        letterbox(img, resized_img, pads, min_scale, target_size);
// 保存预处理图片
#if DEBUG
        cv::imwrite("letterbox_input.jpg", resized_img);
#endif
      } else {
        fprintf(stderr,
                "Invalid resize option. Use 'resize' or 'letterbox'.\n");
        return;
      }
      inputs[0].buf = resized_img.data;
    } else {
      inputs[0].buf = img.data;
    }

    rknn_inputs_set(ctx, io_num.n_input, inputs);

    rknn_output outputs[io_num.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (int i = 0; i < io_num.n_output; i++) {
      outputs[i].index = i;
      outputs[i].want_float = 0;
    }

    // 执行推理
    ret = rknn_run(ctx, NULL);
    ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);

    // 后处理
    detect_result_group_t detect_result_group;
    std::vector<float> out_scales;
    std::vector<int32_t> out_zps;
    for (int i = 0; i < io_num.n_output; ++i) {
      out_scales.push_back(output_attrs[i].scale);
      out_zps.push_back(output_attrs[i].zp);
    }
    post_process((int8_t *)outputs[0].buf, (int8_t *)outputs[1].buf,
                 (int8_t *)outputs[2].buf, height, width, box_conf_threshold,
                 nms_threshold, pads, scale_w, scale_h, out_zps, out_scales,
                 &detect_result_group);

    // 画框和概率
    char text[256];
    for (int i = 0; i < detect_result_group.count; i++) {
      detect_result_t *det_result = &(detect_result_group.results[i]);
      sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);
      printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left,
             det_result->box.top, det_result->box.right, det_result->box.bottom,
             det_result->prop);
      int x1 = det_result->box.left;
      int y1 = det_result->box.top;
      int x2 = det_result->box.right;
      int y2 = det_result->box.bottom;
      rectangle(orig_img, cv::Point(x1, y1), cv::Point(x2, y2),
                cv::Scalar(256, 0, 0, 256), 3);
      putText(orig_img, text, cv::Point(x1, y1 + 12), cv::FONT_HERSHEY_SIMPLEX,
              0.4, cv::Scalar(255, 255, 255));
    }

    // gxt: add my process here ==============begin
    // 相机内参
    // Eigen::Matrix3d camera_matrix;
    // camera_matrix << 787.22316869, 0.0, 628.91534144, 0.0, 793.45182,
    //     313.46301416, 0.0, 0.0, 1.0;
    // //   int my_width=orig_img.cols;
    // //   int my_height=orig_img.rows;
    // DistanceEstimator estimator(camera_matrix, img_width, img_height);
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
    DealImage(*estimator, orig_img, rens, diantis);
    // gxt: add my process here ==============end
    SendImage(orig_img);

#if DEBUG
    printf("save detect result to %s\n", out_path.c_str());
    imwrite(out_path, orig_img);
#endif
    ret = rknn_outputs_release(ctx, io_num.n_output, outputs);
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
