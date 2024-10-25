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
        // 声明参数并设置默认值
        this->declare_parameter<int>("camera_source", 0);
        this->declare_parameter<int>("frame_width", 640);
        this->declare_parameter<int>("frame_height", 480);

        // 使用QoS将消息设置为持久化
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        publisher_ = this->create_publisher<std_msgs::msg::String>("qr_code_info", qos);
        int camera_source;
        this->get_parameter<int>("camera_source", camera_source);
        
        cap_.open(camera_source, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
            rclcpp::shutdown();
        }
        // 设置摄像头宽度和高度
        int frame_width, frame_height;
        this->get_parameter("frame_width", frame_width);
        this->get_parameter("frame_height", frame_height);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);  // 设置宽度，Outside Parameter
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height); // 设置高度，Outside Parameter

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
        cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0); // 高斯模糊
        cv::equalizeHist(gray, gray); // 直方图均衡化
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
