#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class ColorCircleDetector : public rclcpp::Node {
public:
    ColorCircleDetector() : Node("color_circle_detector") {
        // 创建三个话题用于发布红色、蓝色、绿色圆环的偏差坐标
        red_circle_publisher_ = this->create_publisher<std_msgs::msg::String>("/red_circle_offset", 10);
        blue_circle_publisher_ = this->create_publisher<std_msgs::msg::String>("/blue_circle_offset", 10);
        green_circle_publisher_ = this->create_publisher<std_msgs::msg::String>("/green_circle_offset", 10);

        // 启动图像捕获
        image_capture_thread_ = std::thread(&ColorCircleDetector::processImage, this);
    }

    ~ColorCircleDetector() override {
        running_ = false;
        if (image_capture_thread_.joinable()) {
            image_capture_thread_.join();
        }
    }

private:
    void processImage() {
        cv::VideoCapture cap(4); // 0表示默认摄像头
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 设置宽度
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 设置高度


        cv::Point2f image_center(cap.get(cv::CAP_PROP_FRAME_WIDTH) / 2.0, cap.get(cv::CAP_PROP_FRAME_HEIGHT) / 2.0);

        while (running_) {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) continue;

            detectCircleAndPublish(frame, image_center, 1); // 红色圆环
            detectCircleAndPublish(frame, image_center, 2); // 蓝色圆环
            detectCircleAndPublish(frame, image_center, 3); // 绿色圆环

            // 显示结果（可选）
            cv::imshow("Frame", frame);
            if (cv::waitKey(30) >= 0) break;
        }
    }

    void detectCircleAndPublish(cv::Mat& frame, const cv::Point2f& image_center, int color_code) {
        // cv::Scalar lower_color, upper_color;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        cv::Scalar circle_color; // 用于在图像上绘制圆环的颜色

        // 红色范围1
        cv::Scalar lower_red_1(0, 100, 100);
        cv::Scalar upper_red_1(10, 255, 255);
        // 红色范围2
        cv::Scalar lower_red_2(160, 100, 100);
        cv::Scalar upper_red_2(180, 255, 255);
        // 绿色范围
        cv::Scalar lower_green(35, 100, 100);
        cv::Scalar upper_green(85, 255, 255);
        // 蓝色范围
        cv::Scalar lower_blue(100, 100, 100);
        cv::Scalar upper_blue(140, 255, 255);

        // 筛选出不同颜色的部分
        cv::Mat red_mask_1, red_mask_2, green_mask, blue_mask;
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_red_1, upper_red_1, red_mask_1);
        cv::inRange(hsv, lower_red_2, upper_red_2, red_mask_2);
        cv::inRange(hsv, lower_green, upper_green, green_mask);
        cv::inRange(hsv, lower_blue, upper_blue, blue_mask);

        // 合并红色的两个范围
        cv::Mat red_mask = red_mask_1 | red_mask_2;

        // 根据颜色代码设置阈值和对应发布者
        if (color_code == 1) { // 红色
            mask = red_mask;
            publisher = red_circle_publisher_;
            circle_color = cv::Scalar(0, 0, 255); // BGR格式的红色
        } else if (color_code == 2) { // 蓝色
            mask = blue_mask;
            publisher = blue_circle_publisher_;
            circle_color = cv::Scalar(255, 0, 0); // BGR格式的蓝色
        } else if (color_code == 3) { // 绿色
            mask = green_mask;
            publisher = green_circle_publisher_;
            circle_color = cv::Scalar(0, 255, 0); // BGR格式的绿色
        } else {
            return; // 无效的颜色代码
        }

        // 处理图像
        
        // cv::inRange(hsv, lower_color, upper_color, mask);

        // 检测圆形
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, 150, 300, 25, 0, 0);

        if (!circles.empty()) {
            cv::Vec3f c = circles[0];
            cv::Point2f circle_center(c[0], c[1]);
            cv::Point2f offset = circle_center - image_center;
            float radius = c[2];

            // 绘制检测到的圆环
            cv::circle(frame, circle_center, radius, circle_color, 2); // 绘制圆边
            cv::circle(frame, circle_center, 2, circle_color, 3); // 绘制圆心

            // 发布偏差数据
            std_msgs::msg::String msg;
            std::stringstream ss;
            ss << offset.x << "," << offset.y;
            msg.data = ss.str();
            publisher->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Published offset for color code %d: %s", color_code, ss.str().c_str());
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Not found circle of color %d", color_code);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr red_circle_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blue_circle_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr green_circle_publisher_;

    std::thread image_capture_thread_;
    bool running_ = true;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorCircleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
