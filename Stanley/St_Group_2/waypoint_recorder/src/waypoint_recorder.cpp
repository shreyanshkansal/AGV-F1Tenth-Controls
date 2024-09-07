#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>

class WaypointRecorder : public rclcpp::Node {
public:
    WaypointRecorder() : Node("waypoint_recorder"), waypoint_counter_(0), rate_(2.0) { 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ego_racecar/odom", 1, std::bind(&WaypointRecorder::odomCallback, this, std::placeholders::_1));
        
        waypoint_file_.open("/home/goyal/f1_waypoints.csv", std::ios::app);
        if (!waypoint_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file for writing.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Waypoint file opened successfully.");
        }
    }

    ~WaypointRecorder() {
        if (waypoint_file_.is_open()) {
            waypoint_file_.close(); 
            RCLCPP_INFO(this->get_logger(), "Waypoint file closed.");
        }
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (waypoint_file_.is_open()) {
            waypoint_counter_++;  // waypoint counter

            waypoint_file_ << msg->pose.pose.position.x << ", " 
                           << msg->pose.pose.position.y << "\n";
            
            waypoint_file_.flush(); // Ensure data is written to the file
            RCLCPP_INFO(this->get_logger(), "Waypoint %d written to file.", waypoint_counter_);

            rate_.sleep(); // Sleep
        } else {
            RCLCPP_ERROR(this->get_logger(), "Waypoint file is not open.");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream waypoint_file_;  // File stream 
    int waypoint_counter_;  // Counter 
    rclcpp::Rate rate_; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();
    return 0;
}
