#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"

class YoloOnnxNode : public rclcpp::Node
{
public:
  YoloOnnxNode() : Node("yolo_onnx_node")
  {
    model_path_ = this->declare_parameter<std::string>(
      "model_path",
      "yolo11n.onnx"
    );

    confidence_threshold_ = this->declare_parameter<double>(
      "confidence_threshold",
      0.45
    );

    nms_threshold_ = this->declare_parameter<double>(
      "nms_threshold",
      0.45
    );

    person_class_id_ = this->declare_parameter<int>(
      "person_class_id",
      0
    );

    net_ = cv::dnn::readNetFromONNX(model_path_);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw",
      10,
      std::bind(&YoloOnnxNode::imageCallback, this, std::placeholders::_1)
    );

    person_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/perception/person_detected",
      10
    );

    detections_pub_ =
      this->create_publisher<vision_msgs::msg::Detection2DArray>(
        "/perception/detections",
        10
      );

    RCLCPP_INFO(this->get_logger(), "YOLO ONNX node started");
    RCLCPP_INFO(this->get_logger(), "Model: %s", model_path_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;

    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(
      frame,
      blob,
      1.0 / 255.0,
      cv::Size(640, 640),
      cv::Scalar(),
      true,
      false
    );

    net_.setInput(blob);

    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    bool person_detected = false;

    vision_msgs::msg::Detection2DArray detection_array;
    detection_array.header = msg->header;

    if (outputs.empty()) {
      publishPerson(person_detected);
      detections_pub_->publish(detection_array);
      return;
    }

    cv::Mat output = outputs[0];

    if (output.dims == 3 && output.size[1] < output.size[2]) {
      output = output.reshape(1, output.size[1]);
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    float x_factor = static_cast<float>(frame.cols) / 640.0f;
    float y_factor = static_cast<float>(frame.rows) / 640.0f;

    const int rows = output.rows;
    const int dimensions = output.cols;

    for (int i = 0; i < rows; ++i) {
      float * data = output.ptr<float>(i);

      float center_x = data[0];
      float center_y = data[1];
      float width = data[2];
      float height = data[3];

      float best_score = 0.0f;
      int best_class_id = -1;

      for (int c = 4; c < dimensions; ++c) {
        if (data[c] > best_score) {
          best_score = data[c];
          best_class_id = c - 4;
        }
      }

      if (best_score >= confidence_threshold_) {
        int left = static_cast<int>((center_x - width / 2.0f) * x_factor);
        int top = static_cast<int>((center_y - height / 2.0f) * y_factor);
        int box_width = static_cast<int>(width * x_factor);
        int box_height = static_cast<int>(height * y_factor);

        class_ids.push_back(best_class_id);
        confidences.push_back(best_score);
        boxes.emplace_back(left, top, box_width, box_height);
      }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(
      boxes,
      confidences,
      confidence_threshold_,
      nms_threshold_,
      indices
    );

    for (int index : indices) {
      vision_msgs::msg::Detection2D detection;

      detection.bbox.center.position.x =
        boxes[index].x + boxes[index].width / 2.0;
      detection.bbox.center.position.y =
        boxes[index].y + boxes[index].height / 2.0;
      detection.bbox.size_x = boxes[index].width;
      detection.bbox.size_y = boxes[index].height;

      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = std::to_string(class_ids[index]);
      hypothesis.hypothesis.score = confidences[index];

      detection.results.push_back(hypothesis);
      detection_array.detections.push_back(detection);

      if (class_ids[index] == person_class_id_) {
        person_detected = true;
      }
    }

    publishPerson(person_detected);
    detections_pub_->publish(detection_array);
  }

  void publishPerson(bool detected)
  {
    std_msgs::msg::Bool msg;
    msg.data = detected;
    person_pub_->publish(msg);
  }

  std::string model_path_;
  double confidence_threshold_;
  double nms_threshold_;
  int person_class_id_;

  cv::dnn::Net net_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr person_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloOnnxNode>());
  rclcpp::shutdown();
  return 0;
}
