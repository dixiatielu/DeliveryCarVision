#ifndef CIRCLE_DETECTOR_HPP_
#define CIRCLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <yolo_msgs/msg/detection_array.hpp>
#include <deque>
#include <utility>

class CircleDetector : public rclcpp::Node {
public:
    CircleDetector();

private:
    // 成员变量
    int detecting_task_no_;
    std::string qr_info_;
    std::deque<std::pair<float, float>> detected_poses_;
    const int image_width_ = 640;
    const int image_height_ = 480;
    bool yolo_enabled_; // YOLO回调使能
    
    // 订阅者
    rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr yolo_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_sub_;
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_pub_;

    // 回调函数
    void yoloCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
    void qrCallback(const std_msgs::msg::String::SharedPtr msg);
    void serialCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    
    // 辅助函数
    std::string getCurrentRegion();
    std::string getCurrentColorCode();
    std::string getCurrentColor();
    std::string getCurrentTarget();
    bool checkPositionStable(float x, float y);
    void processNextTask();
    std::pair<float, float> getBestDetection(const yolo_msgs::msg::DetectionArray::SharedPtr msg);
    std::string uint8ArrayToString(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    std_msgs::msg::UInt8MultiArray stringToUInt8Array(const std::string &str);
    std_msgs::msg::UInt8MultiArray intToUInt8Array(int dx, int dy);
};

#endif