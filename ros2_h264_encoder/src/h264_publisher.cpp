// System Includes
#include <functional>
#include <cmath>

// Project Includes
#include "ros2_h264_encoder/h264_publisher.hpp"

void PacketPublisher::publish(const sensor_msgs::msg::Image& message, const PublishFn& publish_fn) const {
    std::unique_lock<std::mutex> lock(*encoder_mutex);
    if (encoder) {
        h264_msgs::msg::Packet packet = h264_msgs::msg::Packet();
        packet.header = message.header;
        packet.seq = encoder->get_seq();
        if (encoder->encode_image(message, packet)) {
            publish_fn(packet);
        }
    }
}

void PacketPublisher::advertiseImpl(rclcpp::Node* node, const std::string& base_topic, rmw_qos_profile_t custom_qos, rclcpp::PublisherOptions options) {
    SimplePublisherPlugin::advertiseImpl(node, base_topic, custom_qos, options);
    logger = node->get_logger();
    RCLCPP_INFO_STREAM(logger, "Started Encoder!");
    std::unique_lock<std::mutex> lock(*encoder_mutex);
    encoder = std::make_shared<ROS2Encoder>(node->get_logger());
    enable_encoder_service = node->create_service<std_srvs::srv::SetBool>(
        base_topic + "/enable_encoder",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            std::unique_lock<std::mutex> lock(*encoder_mutex);
            if (request->data) {
                encoder = std::make_shared<ROS2Encoder>(logger);
                response->message = "Encoder enabled!";
            } else {
                encoder.reset();
                response->message = "Encoder disabled!";
            }
            response->success = true;
        });
}