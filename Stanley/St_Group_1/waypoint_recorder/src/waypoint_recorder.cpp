#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>

class WaypointRecorder : public rclcpp::Node {
public:
    WaypointRecorder() : Node("waypoint_recorder"), waypoint_counter_(0) {  // Initialize waypoint_counter_
        // Initialize the odometry subscriber
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "ego_racecar/odom", 1, std::bind(&WaypointRecorder::odomCallback, this, std::placeholders::_1));
        
        // Open the waypoint file in append mode
        waypoint_file_.open("/home/goyal/f1_waypoints.csv", std::ios::app);
        if (!waypoint_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file for writing.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Waypoint file opened successfully.");
        }

        rate_ = std::make_shared<rclcpp::Rate>(2.0);
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
            waypoint_counter_++;  // Increment the waypoint counter

            waypoint_file_ << ", (";   //Add the serial number
            waypoint_file_ << msg->pose.pose.position.x << ", " 
                           << msg->pose.pose.position.y << ")";
            
            waypoint_file_.flush(); // Ensure data is written to the file
            RCLCPP_INFO(this->get_logger(), "Waypoint %d written to file.", waypoint_counter_);
            rate_->sleep();

        } else {
            RCLCPP_ERROR(this->get_logger(), "Waypoint file is not open.");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream waypoint_file_;  // File stream for waypoints
    int waypoint_counter_;  // Counter for the waypoint serial numbers
    std::shared_ptr<rclcpp::Rate> rate_; 

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointRecorder>());
    rclcpp::shutdown();
    return 0;
}