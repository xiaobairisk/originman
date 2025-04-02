#include "action_imitation.hpp"
#include <cstdlib>

ActionImitationNode::ActionImitationNode(const std::string& node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    declare_parameter<std::string>("sub_topic", sub_topic_);
    declare_parameter<int>("offset", offset_);
    declare_parameter<int>("pluse", pluse_);
    declare_parameter<float>("ratio", ratio_);

    get_parameter("sub_topic", sub_topic_);
    get_parameter("offset", offset_);
    get_parameter("pluse", pluse_);
    get_parameter("ratio", ratio_);

    RCLCPP_INFO(get_logger(), "Node initialized with sub_topic: %s, pluse: %d, offset: %d, ratio: %.2f",
                sub_topic_.c_str(), pluse_, offset_, ratio_);

    order_interpreter_ = std::make_shared<OrderInterpreter>();
    target_subscriber_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
        sub_topic_, 1, std::bind(&ActionImitationNode::subscription_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic: %s", sub_topic_.c_str());

    order_interpreter_->control_serial_servo("stand");
    order_interpreter_->control_pwm_servo(1, pluse_, 400);
    order_interpreter_->control_pwm_servo(2, originman::PWM_CENTER, 400);
    RCLCPP_INFO(get_logger(), "Initial actions: stand, PWM1=%d, PWM2=%d", pluse_, originman::PWM_CENTER);

    msg_process_ = std::make_shared<std::thread>([this]() { message_process(); });
}

ActionImitationNode::~ActionImitationNode() {
    process_stop_ = true;
    if (msg_process_ && msg_process_->joinable()) {
        msg_process_->join();
        msg_process_ = nullptr;
    }
    stop_python_script();
    if (python_thread_.joinable()) {
        python_thread_.detach();
    }
    RCLCPP_INFO(get_logger(), "Node shutdown complete.");
}

void ActionImitationNode::subscription_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg) {
    std::unique_lock<std::mutex> lock(target_mutex_);
    targets_msg_ = *targets_msg;
    update_data_ = true;
    lock.unlock();
    RCLCPP_DEBUG(get_logger(), "Received message with %zu targets", targets_msg->targets.size());
}

void ActionImitationNode::message_process() {
    while (!process_stop_) {
        if (!py_thread_start_ && !start_control_) {
            python_thread_ = std::thread([this]() { run_python_script(); });
            py_thread_start_ = true;
            RCLCPP_INFO(get_logger(), "Started Joystick.py script");
        }

        ai_msgs::msg::PerceptionTargets targets_msg;
        {
            std::unique_lock<std::mutex> lock(target_mutex_);
            if (update_data_) {
                targets_msg = targets_msg_;
                update_data_ = false;
            } else {
                lock.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
        }
        RCLCPP_DEBUG(get_logger(), "Processing message with %zu targets", targets_msg.targets.size());

        // Head detection and tracking (commented out in original, retained as is)
        ai_msgs::msg::Target max_head_target;
        int max_head_size = 0;
        if (start_control_) {
            for (const auto& target : targets_msg.targets) {
                if (!target.rois.empty() && target.rois[0].type == "head") {
                    int size = target.rois[0].rect.height * target.rois[0].rect.width;
                    if (size > max_head_size) {
                        max_head_target = target;
                        max_head_size = size;
                    }
                }
            }
            if (max_head_size != 0) {
                static int p = originman::PWM_CENTER;
                int center_x = max_head_target.rois[0].rect.x_offset + max_head_target.rois[0].rect.width / 2;
                RCLCPP_DEBUG(get_logger(), "Head detected, center_x: %d", center_x);
                // Uncomment if head tracking is needed
                // if (center_x > (320 + offset_)) {
                //     p = static_cast<int>(p - (center_x - 320) * ratio_);
                //     p = (p < 1200) ? 1200 : p;
                //     order_interpreter_->control_pwm_servo(2, p, 100);
                //     RCLCPP_INFO(get_logger(), "Head tracking: PWM2 set to %d (left)", p);
                // } else if (center_x < (320 - offset_)) {
                //     p = static_cast<int>(p + (320 - center_x) * ratio_);
                //     p = (p > 1800) ? 1800 : p;
                //     order_interpreter_->control_pwm_servo(2, p, 100);
                //     RCLCPP_INFO(get_logger(), "Head tracking: PWM2 set to %d (right)", p);
                // }
            }
        }

        // Hand detection and gesture control
        ai_msgs::msg::Target max_hand_target;
        int max_hand_size = 0;
        for (const auto& target : targets_msg.targets) {
            if (!target.rois.empty() && target.rois[0].type == "hand") {
                int size = target.rois[0].rect.height * target.rois[0].rect.width;
                if (size > max_hand_size) {
                    max_hand_target = target;
                    max_hand_size = size;
                }
            }
        }

        if (max_hand_size == 0 || max_hand_target.attributes.empty()) {
            RCLCPP_DEBUG(get_logger(), "No valid hand target or attributes found");
            continue;
        }

        int gesture_val = static_cast<int>(max_hand_target.attributes[0].value);
        RCLCPP_INFO(get_logger(), "Detected gesture value: %d", gesture_val);

        switch (gesture_val) {
            case 11: // OK
                {
                    std::unique_lock<std::mutex> lock(start_mutex_);
                    start_num_++;
                    RCLCPP_INFO(get_logger(), "OK gesture detected, start_num: %d", start_num_);
                    if (start_num_ > 3) {
                        start_control_ = true;
                        order_interpreter_->control_pwm_servo(1, pluse_ - 300, 300);
                        order_interpreter_->control_pwm_servo(1, pluse_, 300);
                        RCLCPP_INFO(get_logger(), "Start action: PWM1 moved to %d and back to %d",
                                    pluse_ - 300, pluse_);
                        stop_python_script();
                        if (python_thread_.joinable()) {
                            python_thread_.detach();
                            py_thread_start_ = false;
                            RCLCPP_INFO(get_logger(), "Python script stopped due to start_control");
                        }
                    }
                }
                end_num_ = 0;
                break;
            case 2: // Good
            case 14:
                if (!start_control_) {
                    RCLCPP_DEBUG(get_logger(), "Good gesture ignored: not in control mode");
                    continue;
                }
                end_num_++;
                RCLCPP_INFO(get_logger(), "Good gesture detected, end_num: %d", end_num_);
                if (end_num_ > 10) {
                    order_interpreter_->control_serial_servo("stand");
                    order_interpreter_->control_serial_servo("bow");
                    order_interpreter_->control_serial_servo("stand");
                    order_interpreter_->control_pwm_servo(2, originman::PWM_CENTER, 200);
                    {
                        std::unique_lock<std::mutex> lock(start_mutex_);
                        start_control_ = false;
                    }
                    gesture_control_ = false;
                    imitating_control_ = false;
                    RCLCPP_INFO(get_logger(), "End action: stand -> bow -> stand, PWM2 reset to %d",
                                originman::PWM_CENTER);
                }
                start_num_ = 0;
                break;
            default:
                start_num_ = 0;
                end_num_ = 0;
                RCLCPP_DEBUG(get_logger(), "Gesture counters reset");
        }

        if (!start_control_) {
            RCLCPP_DEBUG(get_logger(), "Skipping further actions: start_control_ is false");
            continue;
        }

        // Body detection
        ai_msgs::msg::Target max_body_target;
        int max_body_size = 0;
        for (const auto& target : targets_msg.targets) {
            if (!target.rois.empty() && target.rois[0].type == "body") {
                int size = target.rois[0].rect.height * target.rois[0].rect.width;
                if (size > max_body_size) {
                    max_body_target = target;
                    max_body_size = size;
                }
            }
        }

        if (max_body_size == 0) {
            RCLCPP_DEBUG(get_logger(), "No body target detected");
            continue;
        }

        // Mode switching
        switch (gesture_val) {
            case 0: // Palm
            case 5:
                gesture_control_ = false;
                imitating_control_ = true;
                order_interpreter_->control_serial_servo(18, originman::DEFAULT_PULSE, 100);
                RCLCPP_INFO(get_logger(), "Palm gesture: Entered imitating mode, Servo 18 set to %d",
                            originman::DEFAULT_PULSE);
                break;
            default:
                gesture_control_ = true;
                imitating_control_ = false;
                order_interpreter_->control_serial_servo(18, originman::DEFAULT_PULSE, 100);
                RCLCPP_INFO(get_logger(), "Default gesture: Entered gesture mode, Servo 18 set to %d",
                            originman::DEFAULT_PULSE);
        }

        RCLCPP_DEBUG(get_logger(), "Control states - start: %d, gesture: %d, imitating: %d",
                     start_control_, gesture_control_, imitating_control_);

        // Imitation mode - Arm movements
        if (imitating_control_) {
            // Right arm
            Point p8(max_body_target.points[0].point[8].x, max_body_target.points[0].point[8].y);
            Point p6(max_body_target.points[0].point[6].x, max_body_target.points[0].point[6].y);
            Point p12(max_body_target.points[0].point[12].x, max_body_target.points[0].point[12].y);
            Point p10(max_body_target.points[0].point[10].x, max_body_target.points[0].point[10].y);

            double angle1 = angle_calculator(p8, p6, p12);
            if (angle1 != -1) {
                angle_mean_filter(angle1, num1_, angles1_, filter_result1_);
                int pluse1 = originman::MAX_PULSE_RIGHT - (750 / 180) * filter_result1_;
                pluse1 = std::max(originman::MIN_PULSE, pluse1);

                double angle2 = angle_calculator(p6, p8, p10);
                if (angle2 == -1) angle2 = 180;
                angle_mean_filter(angle2, num2_, angles2_, filter_result2_);
                double slope = static_cast<double>(p8.y - p6.y) / (p8.x - p6.x);
                double val_y = slope * (p10.x - p6.x);
                int pluse2 = (val_y > (p10.y - p6.y)) ?
                             (originman::MIN_PULSE + (500.0 / 120) * (filter_result2_ - 60)) :
                             (originman::MAX_PULSE_RIGHT - (300.0 / 120) * (filter_result2_ - 60));
                pluse2 = std::max(originman::MIN_PULSE, pluse2);

                RCLCPP_DEBUG(get_logger(), "Right arm - angle1: %.2f, pluse1: %d, angle2: %.2f, pluse2: %d",
                             angle1, pluse1, angle2, pluse2);

                if (!collision_detection_pluse_r(pluse2, pluse1)) {
                    order_interpreter_->control_serial_servo(7, pluse1, 0);
                    order_interpreter_->control_serial_servo(6, pluse2, 0);
                    RCLCPP_INFO(get_logger(), "Right arm moved: Servo 7=%d, Servo 6=%d", pluse1, pluse2);
                } else {
                    RCLCPP_WARN(get_logger(), "Right arm collision detected");
                }
            }

            // Left arm
            Point p11(max_body_target.points[0].point[11].x, max_body_target.points[0].point[11].y);
            Point p5(max_body_target.points[0].point[5].x, max_body_target.points[0].point[5].y);
            Point p7(max_body_target.points[0].point[7].x, max_body_target.points[0].point[7].y);
            Point p9(max_body_target.points[0].point[9].x, max_body_target.points[0].point[9].y);

            double angle3 = angle_calculator(p7, p5, p11);
            if (angle3 != -1) {
                angle_mean_filter(angle3, num3_, angles3_, filter_result3_);
                int pluse3 = 100 + (750 / 180) * filter_result3_;
                pluse3 = std::max(originman::MIN_PULSE, pluse3);

                double angle4 = angle_calculator(p5, p7, p9);
                if (angle4 == -1) angle4 = 180;
                angle_mean_filter(angle4, num4_, angles4_, filter_result4_);
                double slope = static_cast<double>(p7.y - p5.y) / (p7.x - p5.x);
                double val_y = slope * (p9.x - p5.x);
                int pluse4 = (val_y > (p9.y - p5.y)) ?
                             (originman::MAX_PULSE_RIGHT - (400.0 / 120) * (filter_result4_ - 60)) :
                             (originman::MIN_PULSE + (500.0 / 120) * (filter_result4_ - 60));
                pluse4 = std::max(originman::MIN_PULSE, pluse4);

                RCLCPP_DEBUG(get_logger(), "Left arm - angle3: %.2f, pluse3: %d, angle4: %.2f, pluse4: %d",
                             angle3, pluse3, angle4, pluse4);

                if (!collision_detection_pluse_l(pluse4, pluse3)) {
                    order_interpreter_->control_serial_servo(15, pluse3, 0);
                    order_interpreter_->control_serial_servo(14, pluse4, 0);
                    RCLCPP_INFO(get_logger(), "Left arm moved: Servo 15=%d, Servo 14=%d", pluse3, pluse4);
                } else {
                    RCLCPP_WARN(get_logger(), "Left arm collision detected");
                }
            }
        }

        // Gesture control mode
        if (gesture_control_) {
            switch (gesture_val) {
                case 3: // Stop
                    left_control_num_ = 0;
                    right_control_num_ = 0;
                    order_interpreter_->control_serial_servo(18, originman::MAX_PULSE_RIGHT, 100);
                    RCLCPP_INFO(get_logger(), "Stop gesture: Servo 18 set to %d", originman::MAX_PULSE_RIGHT);
                    break;
                case 12: // ThumbLeft
                    right_control_num_++;
                    RCLCPP_INFO(get_logger(), "ThumbLeft gesture count: %d", right_control_num_);
                    if (right_control_num_ > 2) {
                        order_interpreter_->control_serial_servo("right_move_10");
                        right_control_num_ = 0;
                        RCLCPP_INFO(get_logger(), "Executed right_move_10 action");
                    }
                    left_control_num_ = 0;
                    break;
                case 13: // ThumbRight
                    left_control_num_++;
                    RCLCPP_INFO(get_logger(), "ThumbRight gesture count: %d", left_control_num_);
                    if (left_control_num_ > 2) {
                        order_interpreter_->control_serial_servo("left_move_10");
                        left_control_num_ = 0;
                        RCLCPP_INFO(get_logger(), "Executed left_move_10 action");
                    }
                    right_control_num_ = 0;
                    break;
                default:
                    left_control_num_ = 0;
                    right_control_num_ = 0;
                    RCLCPP_DEBUG(get_logger(), "Gesture counters reset");
            }
        }
    }
}

bool ActionImitationNode::collision_detection(double degree1, double degree2) {
    if (degree1 > 90) return false;
    double angle1 = degree1 * originman::PI / 180.0;
    double sin1 = std::sin(angle1);
    float length1 = 1 + sin1 * 6;
    int degree = degree2 - (90 - degree1);
    double angle2 = degree * originman::PI / 180.0;
    float length2 = 12 * std::cos(angle2);
    return length2 > length1;
}

bool ActionImitationNode::collision_detection_pluse_r(int p6, int p7) {
    if (p6 > originman::DEFAULT_PULSE && p7 > originman::DEFAULT_PULSE) {
        double r1 = 6.0 * std::cos((p7 - originman::DEFAULT_PULSE) * originman::PI / 800.0);
        double r2 = 12.0 * std::sin(((p6 - originman::DEFAULT_PULSE) * 90.0 / 400.0 -
                                    (90 - (p7 - originman::DEFAULT_PULSE) * 90 / 400)) * originman::PI / 180.0);
        return r2 > r1;
    }
    return false;
}

bool ActionImitationNode::collision_detection_pluse_l(int p14, int p15) {
    if (p14 < originman::DEFAULT_PULSE && p15 < originman::DEFAULT_PULSE) {
        double l1 = 6.0 * std::cos((originman::DEFAULT_PULSE - p15) * originman::PI / 800.0);
        double l2 = 12.0 * std::sin(((originman::DEFAULT_PULSE - p14) * 90.0 / 400.0 -
                                    (90 - (originman::DEFAULT_PULSE - p15) * 90 / 400)) * originman::PI / 180.0);
        return l2 > l1;
    }
    return false;
}

double ActionImitationNode::angle_calculator(const Point& point_1, const Point& point_2, const Point& point_3) {
    double l1 = Point::distance(point_2, point_3);
    double l2 = Point::distance(point_1, point_3);
    double l3 = Point::distance(point_1, point_2);
    double cos_2 = (l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3);
    if (cos_2 < -1.0 || cos_2 > 1.0) {
        RCLCPP_ERROR(get_logger(), "Cosine value out of range, unable to calculate angle!");
        return -1;
    }
    double radian = std::acos(cos_2);
    return radian * (180.0 / originman::PI);
}

void ActionImitationNode::angle_mean_filter(double angle, int& num, std::vector<int>& angles, int& filter_result) {
    angles[num % originman::FILTER_WINDOW_SIZE] = static_cast<int>(angle);
    if (num > originman::FILTER_WINDOW_SIZE) {
        filter_result = (angles[0] + angles[1]) / 2;
    }
    num++;
}

void ActionImitationNode::run_python_script() {
    RCLCPP_INFO(get_logger(), "Starting Python script: %s", originman::SCRIPT_PATH);
    system((std::string("python3 ") + originman::SCRIPT_PATH + " &").c_str());
}

void ActionImitationNode::stop_python_script() {
    RCLCPP_INFO(get_logger(), "Stopping Python script");
    system("ps aux | grep -i joystick | grep -v grep | awk '{print $2}' | xargs kill -9");
}