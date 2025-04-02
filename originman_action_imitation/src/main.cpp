#include "action_imitation.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionImitationNode>("ActionImitationNode", rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_WARN(node->get_logger(), "Action imitation node shutdown.");
    return 0;
}