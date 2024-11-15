#include "circle_detector/circle_detector.hpp"

CircleDetector::CircleDetector() : Node("circle_detector"), detecting_task_no_(1) {
    // 创建订阅者
    yolo_sub_ = create_subscription<yolo_msgs::msg::DetectionArray>(
        "/yolo/detections", 10, 
        std::bind(&CircleDetector::yoloCallback, this, std::placeholders::_1));
        
    qr_sub_ = create_subscription<std_msgs::msg::String>(
        "/qr_code_info", 10,
        std::bind(&CircleDetector::qrCallback, this, std::placeholders::_1));
        
    serial_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/serial_read", 10,
        std::bind(&CircleDetector::serialCallback, this, std::placeholders::_1));

    // 创建发布者
    serial_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/serial_write", 10);
}

void CircleDetector::qrCallback(const std_msgs::msg::String::SharedPtr msg) {
    qr_info_ = msg->data;
}

std::string CircleDetector::getCurrentColor() {
    int task_group = (detecting_task_no_ - 1) / 12;
    int color_index = (detecting_task_no_ - 1) % 3;
    std::string color_code;
    
    if (task_group == 0) {
        color_code = qr_info_[color_index];
    } else {
        color_code = qr_info_[color_index + 4];
    }
    
    if (color_code == "1") return "RED";
    if (color_code == "2") return "GREEN";
    return "BLUE";
}

std::string CircleDetector::getCurrentTarget() {
    std::string color = getCurrentColor();
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);
    bool is_solid = (((detecting_task_no_ - 1) / 3) % 2 == 0);
    return color + "_" + (is_solid ? "solid" : "hollow") + "_circle";
}

bool CircleDetector::checkPositionStable(float x, float y) {
    detected_poses_.push_back({x, y});
    if (detected_poses_.size() >= 2) {
        auto front = detected_poses_.front();
        float dx = std::abs(x - front.first);
        float dy = std::abs(y - front.second);
        detected_poses_.pop_front();
        return (dx < 0.1 * image_width_ && dy < 0.1 * image_height_);
    }
    return false;
}

std::pair<float, float> CircleDetector::getBestDetection(
    const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
    std::string target = getCurrentTarget();
    float best_score = -1.0;
    float best_x = 0.0, best_y = 0.0;
    
    for (const auto& detection : msg->detections) {
        if (detection.class_name == target && detection.score > best_score) {
            best_score = detection.score;
            best_x = detection.bbox.center.position.x;
            best_y = detection.bbox.center.position.y;
        }
    }
    return {best_x, best_y};
}

void CircleDetector::processNextTask() {
    detecting_task_no_++;
    if (detecting_task_no_ % 3 == 0) {
        auto msg = stringToUInt8Array("GO");
        serial_pub_->publish(msg);
    }
}

void CircleDetector::serialCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    std::string data = uint8ArrayToString(msg);
    if (data == "ARRIVED") {
        detected_poses_.clear();
    } else if (data == "FINISHED") {
        processNextTask();
    }
}

void CircleDetector::yoloCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg) {
    auto [x, y] = getBestDetection(msg);
    if (x == 0 && y == 0) return;

    float dx = x - image_width_/2;
    float dy = y - image_height_/2;

    if (!checkPositionStable(x, y)) {
        auto pos_msg = intToUInt8Array(static_cast<int>(dx), static_cast<int>(dy));
        serial_pub_->publish(pos_msg);
        return;
    }

    if (std::abs(dx) < 0.1 * image_width_ && std::abs(dy) < 0.1 * image_height_) {
        std::string task_type = (((detecting_task_no_ - 1) / 3) % 2 == 0 ? "GRAB" : "PUT");
        auto msg = stringToUInt8Array("READY_" + 
            task_type + 
            "_" + getCurrentColor());
        serial_pub_->publish(msg);
    }
}

std::string CircleDetector::uint8ArrayToString(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    std::string result;
    for (auto c : msg->data) {
        result += static_cast<char>(c);
    }
    return result;
}

std_msgs::msg::UInt8MultiArray CircleDetector::stringToUInt8Array(const std::string &str) {
    std_msgs::msg::UInt8MultiArray msg;
    for (auto c : str) {
        msg.data.push_back(static_cast<uint8_t>(c));
    }
    return msg;
}

std_msgs::msg::UInt8MultiArray CircleDetector::intToUInt8Array(int dx, int dy) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data.push_back(static_cast<uint8_t>(dx & 0xFF));
    msg.data.push_back(static_cast<uint8_t>((dx >> 8) & 0xFF));
    msg.data.push_back(static_cast<uint8_t>(dy & 0xFF));
    msg.data.push_back(static_cast<uint8_t>((dy >> 8) & 0xFF));
    return msg;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CircleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}