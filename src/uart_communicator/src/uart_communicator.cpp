#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <poll.h>
#include <string>
#include <unistd.h>

class UARTCommunicator : public rclcpp::Node {
public:
    UARTCommunicator() : Node("uart_communicator"), uart_fd(-1), qr_code_data_(""), data_sent_(false) {
        // 打开串口设备
        uart_fd = serialOpen("/dev/ttyS5", 9600);
        if (uart_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/ttyS5");
            rclcpp::shutdown();
            return;
        }

        // 初始化 wiringPi
        wiringPiSetup();

        // 订阅 /qr_code_info 话题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/qr_code_info", 10,
            [this](std_msgs::msg::String::UniquePtr msg) {
                if (!data_sent_) {
                    qr_code_data_ = msg->data;
                    sendData();
                    data_sent_ = true;
                }
            }
        );

        // 使用线程来处理串口事件
        uart_thread_ = std::thread([this]() { checkSerialInput(); });
    }

    ~UARTCommunicator() {
        if (uart_fd >= 0) {
            serialClose(uart_fd);
        }
        if (uart_thread_.joinable()) {
            uart_thread_.join();
        }
    }

private:
    void sendData() {
        if (!qr_code_data_.empty() && uart_fd >= 0) {
            serialPuts(uart_fd, qr_code_data_.c_str());
            RCLCPP_INFO(this->get_logger(), "Sent data: %s", qr_code_data_.c_str());
        }
    }

    void checkSerialInput() {
        struct pollfd fds;
        fds.fd = uart_fd;
        fds.events = POLLIN;

        while (rclcpp::ok()) {
            int ret = poll(&fds, 1, -1); // 无限等待直到串口有数据
            if (ret > 0) {
                if (fds.revents & POLLIN) {
                    std::string input;
                    while (serialDataAvail(uart_fd) > 0) {
                        char c = serialGetchar(uart_fd);
                        input += c;
                        usleep(100);  // 防止读取速度过快
                    }
                    RCLCPP_INFO(this->get_logger(), "Received data: %s", input.c_str());

                    if (input.find("GET_QR") != std::string::npos) {
                        sendData();
                    }
                }
            }
        }
    }

    int uart_fd;
    std::string qr_code_data_;
    bool data_sent_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::thread uart_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UARTCommunicator>());
    rclcpp::shutdown();
    return 0;
}
