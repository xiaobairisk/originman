#ifndef ACTION_IMITATION_HPP
#define ACTION_IMITATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <ai_msgs/msg/perception_targets.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include "order_interpreter.hpp"

namespace originman {
constexpr int MIN_PULSE = 0;
constexpr int MAX_PULSE_RIGHT = 900;
constexpr int MAX_PULSE_LEFT = 1000;
constexpr int DEFAULT_PULSE = 500;
constexpr int PWM_CENTER = 1500;
constexpr int FILTER_WINDOW_SIZE = 2;
constexpr double PI = M_PI;
constexpr char SCRIPT_PATH[] = "Joystick.py";
}

struct Point {
    int x, y;
    Point(int x_, int y_) : x(x_), y(y_) {}
    static double distance(const Point& p1, const Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }
};

class ActionImitationNode : public rclcpp::Node {
public:
    ActionImitationNode(const std::string& node_name, const rclcpp::NodeOptions& options);
    ~ActionImitationNode();

private:
    void subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg);
    void message_process();

    bool collision_detection(double degree1, double degree2);
    bool collision_detection_pluse_r(int p6, int p7);
    bool collision_detection_pluse_l(int p14, int p15);
    double angle_calculator(const Point& point_1, const Point& point_2, const Point& point_3);
    void angle_mean_filter(double angle, int& num, std::vector<int>& angles, int& filter_result);

    void run_python_script();
    void stop_python_script();

    std::string sub_topic_ = "/hobot_hand_gesture_detection";
    int offset_ = 70;
    int pluse_ = 2000;
    float ratio_ = 0.15;
    bool process_stop_ = false;
    bool start_control_ = false;
    bool gesture_control_ = false;
    bool imitating_control_ = false;
    bool update_data_ = false;
    bool py_thread_start_ = false;
    int start_num_ = 0, end_num_ = 0;
    int left_control_num_ = 0, right_control_num_ = 0;

    std::mutex target_mutex_;
    std::mutex start_mutex_;
    ai_msgs::msg::PerceptionTargets targets_msg_;
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr target_subscriber_;
    std::shared_ptr<OrderInterpreter> order_interpreter_;
    std::shared_ptr<std::thread> msg_process_;
    std::thread python_thread_;

    int num1_ = 0, num2_ = 0, num3_ = 0, num4_ = 0;
    int filter_result1_ = 0, filter_result2_ = 0, filter_result3_ = 0, filter_result4_ = 0;
    std::vector<int> angles1_{0, 0}, angles2_{0, 0}, angles3_{0, 0}, angles4_{0, 0};
};

#endif // ACTION_IMITATION_HPP