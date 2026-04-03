#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode()
    : Node("obstacle_avoidance_node"),
      turn_direction_(0),
      clear_cycles_(0)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ObstacleAvoidanceNode::scan_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObstacleAvoidanceNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node started.");
    }

private:
    static constexpr int kRequiredClearCycles = 4;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
    }

    float get_min_distance_in_range(int start_index, int end_index)
    {
        if (!latest_scan_) {
            return std::numeric_limits<float>::infinity();
        }

        const auto & ranges = latest_scan_->ranges;
        if (ranges.empty()) {
            return std::numeric_limits<float>::infinity();
        }

        const int size = static_cast<int>(ranges.size());

        start_index = std::max(0, start_index);
        end_index = std::min(size - 1, end_index);

        if (start_index > end_index) {
            return std::numeric_limits<float>::infinity();
        }

        float min_distance = std::numeric_limits<float>::infinity();

        for (int i = start_index; i <= end_index; ++i) {
            if (std::isfinite(ranges[i])) {
                min_distance = std::min(min_distance, ranges[i]);
            }
        }

        return min_distance;
    }

    float get_average_distance_in_range(int start_index, int end_index)
    {
        if (!latest_scan_) {
            return 0.0f;
        }

        const auto & ranges = latest_scan_->ranges;
        if (ranges.empty()) {
            return 0.0f;
        }

        const int size = static_cast<int>(ranges.size());

        start_index = std::max(0, start_index);
        end_index = std::min(size - 1, end_index);

        if (start_index > end_index) {
            return 0.0f;
        }

        float sum = 0.0f;
        int count = 0;

        for (int i = start_index; i <= end_index; ++i) {
            if (std::isfinite(ranges[i])) {
                sum += ranges[i];
                ++count;
            }
        }

        if (count == 0) {
            return 0.0f;
        }

        return sum / static_cast<float>(count);
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (!latest_scan_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        const auto & ranges = latest_scan_->ranges;
        const int size = static_cast<int>(ranges.size());

        if (size == 0) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        // Assumed scan layout for TurtleBot3:
        // index 0           -> front
        // increasing index  -> counterclockwise
        // around size/4     -> left
        // around 3*size/4   -> right

        // Safety-focused sectors (minimum distance)
        const float front_min = std::min(
            get_min_distance_in_range(0, 29),
            get_min_distance_in_range(size - 30, size - 1)
        );

        const float front_left_min =
            get_min_distance_in_range(size / 8, size / 4);

        const float front_right_min =
            get_min_distance_in_range(size * 3 / 4, size * 7 / 8);

        const float left_side_min =
            get_min_distance_in_range(size / 4, size * 3 / 8);

        const float right_side_min =
            get_min_distance_in_range(size * 5 / 8, size * 3 / 4);

        // Open-space sectors (average distance)
        const float left_open =
            get_average_distance_in_range(size / 6, size / 3);

        const float right_open =
            get_average_distance_in_range(size * 2 / 3, size * 5 / 6);

        // Tunable constants
        const float obstacle_enter_threshold = 0.42f;
        const float obstacle_exit_threshold = 0.55f;
        const float front_corner_threshold = 0.30f;
        const float side_wall_threshold = 0.22f;

        const float forward_speed = 0.16f;
        const float gentle_turn_forward_speed = 0.06f;
        const float strong_turn_forward_speed = 0.00f;

        const float gentle_turn_speed = 0.28f;
        const float strong_turn_speed = 0.40f;

        const float direction_bias = 0.05f;

        const bool front_blocked_enter = front_min < obstacle_enter_threshold;
        const bool front_clear_exit = front_min > obstacle_exit_threshold;

        const bool left_front_blocked = front_left_min < front_corner_threshold;
        const bool right_front_blocked = front_right_min < front_corner_threshold;
        const bool left_wall_too_close = left_side_min < side_wall_threshold;
        const bool right_wall_too_close = right_side_min < side_wall_threshold;

        const bool wall_risk = left_front_blocked || right_front_blocked ||
                               left_wall_too_close || right_wall_too_close;

        const bool avoidance_needed = (turn_direction_ == 0)
            ? (front_blocked_enter || wall_risk)
            : (!front_clear_exit || wall_risk);

        if (avoidance_needed) {
            clear_cycles_ = 0;

            // Choose direction only when not already committed.
            if (turn_direction_ == 0) {
                if (left_wall_too_close || left_front_blocked) {
                    // obstacle on left/front-left -> turn right
                    turn_direction_ = -1;
                } else if (right_wall_too_close || right_front_blocked) {
                    // obstacle on right/front-right -> turn left
                    turn_direction_ = 1;
                } else if (left_open > right_open + direction_bias) {
                    turn_direction_ = 1;   // left is more open
                } else if (right_open > left_open + direction_bias) {
                    turn_direction_ = -1;  // right is more open
                } else {
                    // default when nearly equal
                    turn_direction_ = 1;
                }
            }

            // Stronger response if obstacle is very close
            const bool very_close = front_min < 0.26f;

            if ((turn_direction_ > 0 && left_wall_too_close) ||
                (turn_direction_ < 0 && right_wall_too_close)) {
                cmd.linear.x = strong_turn_forward_speed;
            } else {
                cmd.linear.x = very_close ? strong_turn_forward_speed
                                          : gentle_turn_forward_speed;
            }

            cmd.angular.z = turn_direction_ * (very_close ? strong_turn_speed
                                                          : gentle_turn_speed);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Front: %.2f | Left open: %.2f | Right open: %.2f | Turning %s",
                front_min,
                left_open,
                right_open,
                turn_direction_ > 0 ? "left" : "right");
        } else {
            ++clear_cycles_;

            if (clear_cycles_ >= kRequiredClearCycles) {
                turn_direction_ = 0;
            }

            cmd.linear.x = forward_speed;
            cmd.angular.z = 0.0;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Path clear. Moving forward.");
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    int turn_direction_;   // 1 = left, -1 = right, 0 = none
    int clear_cycles_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}