//
// Created by dxtl on 24-9-27.
//
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace boost::asio;
using namespace std::chrono_literals;

class ColorCircleDetector : public rclcpp::Node {
public:
    ColorCircleDetector() : Node("color_circle_detector") {
        qr_code_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/qr_code_info", 10, std::bind(&ColorCircleDetector::qrCodeCallback, this, std::placeholders::_1));

        // 串口设置
        setupSerial();

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
    void qrCodeCallback(const std_msgs::msg::String::SharedPtr msg) {
        parseColorSequence(msg->data);
    }

    void setupSerial() {
        sp_.open("/dev/ttyACM0"); // 根据实际情况修改串口名
        sp_.set_option(serial_port::baud_rate(9600));
        sp_.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp_.set_option(serial_port::parity(serial_port::parity::none));
        sp_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp_.set_option(serial_port::character_size(8));
    }

    void sendData(const std::string& data) {
        write(sp_, buffer(data));
    }

    void parseColorSequence(const std::string& data) {
        color_sequence_.clear();
        for (char c : data) {
            if (c >= '1' && c <= '3') {
                color_sequence_.push_back(c - '0');
            }
        }
    }

    void processImage() {
        cv::VideoCapture cap(0); // 0表示默认摄像头
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }

        cv::Point2f image_center(cap.get(cv::CAP_PROP_FRAME_WIDTH) / 2.0, cap.get(cv::CAP_PROP_FRAME_HEIGHT) / 2.0);
        const float threshold = 1000.0; // 偏差阈值

        while (running_) {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) continue;

            for (int color_code : color_sequence_) {
                cv::Scalar lower_color, upper_color;

                // 根据颜色代码设置阈值
                if (color_code == 1) { // 红色
                    lower_color = cv::Scalar(0, 100, 100);
                    upper_color = cv::Scalar(10, 255, 255);
                } else if (color_code == 2) { // 蓝色
                    lower_color = cv::Scalar(100, 100, 100);
                    upper_color = cv::Scalar(140, 255, 255);
                } else if (color_code == 3) { // 绿色
                    lower_color = cv::Scalar(35, 100, 100);
                    upper_color = cv::Scalar(85, 255, 255);
                } else {
                    continue; // 无效的颜色代码
                }

                // 处理图像
                cv::Mat hsv, mask;
                cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
                cv::inRange(hsv, lower_color, upper_color, mask);

                // 检测圆形
                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, 100, 100, 20, 0, 0);

                if (!circles.empty()) {
                    cv::Vec3f c = circles[0];
                    cv::Point2f circle_center(c[0], c[1]);
                    cv::Point2f offset = circle_center - image_center;

                    // 发送偏差数据
                    std::stringstream ss;
                    ss << offset.x << "," << offset.y << "\n";
                    sendData(ss.str());
                    RCLCPP_INFO(this->get_logger(), "OFFSET:%s\n", ss.str().c_str());

                    // 检查距离
                    float distance = cv::norm(offset);
                    RCLCPP_INFO(this->get_logger(), "NOW_DISTANCE:%f\n", distance);
                    if (distance < threshold) {
                        sendData("READY\n");
                        break; // 继续下一个颜色
                    }
                }
            }

            // 显示结果（可选）
            cv::imshow("Frame", frame);
            if (cv::waitKey(30) >= 0) break;
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_code_subscriber_;
    std::vector<int> color_sequence_;
    boost::asio::io_service io_service_;
    boost::asio::serial_port sp_{io_service_};
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
