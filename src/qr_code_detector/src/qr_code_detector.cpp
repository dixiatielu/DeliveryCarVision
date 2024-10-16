//
// Created by dxtl on 24-9-25.
//
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <opencv2/imgproc.hpp>

class QRCodeDetector : public rclcpp::Node
{
public:
    QRCodeDetector()
        : Node("qr_code_detector")
    {
        // 使用QoS将消息设置为持久化
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code_info", qos);

        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
            rclcpp::shutdown();
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&QRCodeDetector::detectQRCode, this));
    }

private:
    void detectQRCode()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            return;
        }
        // 转换为灰度图像
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 创建ZBAR图像
        zbar::Image zbarImage(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

        // 扫描图像
        int n = scanner.scan(zbarImage);
        if (n > 0) {
            for (zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin(); symbol != zbarImage.symbol_end(); ++symbol) {
                std::string data = symbol->get_data();
                RCLCPP_INFO(this->get_logger(), "QR Code detected: %s", data.c_str());
                std_msgs::msg::String msg;
                msg.data = data;
                publisher_->publish(msg);
                cap_.release(); // 识别到二维码后释放摄像头
                break; // 识别到一个二维码后退出
            }
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeDetector>());
    rclcpp::shutdown();
    return 0;
}
