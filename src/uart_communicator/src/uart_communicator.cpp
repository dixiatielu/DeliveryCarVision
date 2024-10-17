#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

class UartCommunicator : public rclcpp::Node
{
public:
    UartCommunicator()
    : Node("uart_communicator"), io_(), serial_port_(io_)
    {
        // 初始化串口
        try {
            serial_port_.open("/dev/ttyS0");  // 根据实际串口设备名称修改
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        } catch (boost::system::system_error &e) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", e.what());
        }

        // 订阅话题
        qr_code_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/qr_code_info", 10,
            std::bind(&UartCommunicator::qrCodeCallback, this, std::placeholders::_1));

        red_circle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/red_circle_offset", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->circleOffsetCallback(msg, "red"); });

        blue_circle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/blue_circle_offset", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->circleOffsetCallback(msg, "blue"); });

        green_circle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/green_circle_offset", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->circleOffsetCallback(msg, "green"); });
    }

private:
    void qrCodeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到二维码信息: %s", msg->data.c_str());

        // 通过UART发送信息
        if (serial_port_.is_open()) {
            try {
                std::string data = msg->data + "\n";  // 添加换行符作为结束符
                boost::asio::write(serial_port_, boost::asio::buffer(data));
            } catch (boost::system::system_error &e) {
                RCLCPP_ERROR(this->get_logger(), "串口发送错误: %s", e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "串口未打开，无法发送数据");
        }
    }

    void circleOffsetCallback(const std_msgs::msg::Float32::SharedPtr msg, const std::string &color)
    {
        RCLCPP_INFO(this->get_logger(), "接收到%s圆偏移量: %f", color.c_str(), msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_code_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr red_circle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr blue_circle_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr green_circle_subscriber_;

    boost::asio::io_service io_;
    boost::asio::serial_port serial_port_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartCommunicator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
