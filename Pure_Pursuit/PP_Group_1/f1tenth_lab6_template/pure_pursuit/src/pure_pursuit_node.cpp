#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!



private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;

    vector<double> base_position = {0,0,0};
    vector<double> goal_position = {0,0,0};

    float base_angle = 0;
    float L;
    double steering_angle(float L, float alpha, float D){
        return atan(2*L*sin(alpha)/D);
    }

    
    void get_transform()
    {
        geometry_msgs::msg::TransformStamped transformStamped,transformStamped_L,transformStamped_R;
        
        try
        {
            // Get the transform from 'map' to 'base_link'
            transformStamped = tf_buffer_.lookupTransform("map", "ego_racecar/base_link", tf2::TimePointZero);
            transformStamped_L = tf_buffer_.lookupTransform("ego_racecar/base_link", "ego_racecar/front_left_hinge", tf2::TimePointZero);
            transformStamped_R= tf_buffer_.lookupTransform("ego_racecar/base_link", "ego_racecar/front_right_hinge", tf2::TimePointZero);

            float del_x = (transformStamped_L.transform.translation.x + transformStamped_R.transform.translation.x)/2;
            float del_y = (transformStamped_L.transform.translation.y + transformStamped_R.transform.translation.y)/2;

            L = sqrt(del_x*del_x + del_y*del_y);

            // Extract translation (coordinates) from the transform
            base_position[0]= transformStamped.transform.translation.x;
            base_position[1]= transformStamped.transform.translation.y;
            base_position[2]= transformStamped.transform.translation.z;

            // Extract the rotation in quaternion form
            double qx = transformStamped.transform.rotation.x;
            double qy = transformStamped.transform.rotation.y;
            double qz = transformStamped.transform.rotation.z;
            double qw = transformStamped.transform.rotation.w;
            
            // Convert the quaternion to roll, pitch, yaw (optional)
            tf2::Quaternion q(qx, qy, qz, qw);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw); // Extract the roll, pitch, and yaw
            base_angle = yaw;

            RCLCPP_INFO(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f", base_position[0],base_position[1],base_position[2]);
            RCLCPP_INFO(this->get_logger(), "Rotation: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);

            // auto message = std_msgs::msg::String();
            // message.data = "Hello, ROS 2!";
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            // publisher_->publish(message);
        


        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from 'map' to 'base_link 11': %s", ex.what());
        }
    }
public:
    PurePursuit() : Node("pure_pursuit_node"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
    {
        // TODO: create ROS subscribers and publishers
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        // timer_ = this->create_wall_timer(500ms, std::bind(&PurePursuit::publish_message, this));

        subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));

        // timer_ = this->create_wall_timer(500ms, std::bind(&PurePursuit::get_transform, this));
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {

        get_transform();
        // TODO: find the current waypoint to track using methods mentioned in lecture
        
        goal_position[0] =  pose_msg->pose.position.x;
        goal_position[1] =  pose_msg->pose.position.y;
        goal_position[2] =  pose_msg->pose.position.z;
        double alpha = atan(goal_position[1]-base_position[1]/goal_position[0]-base_position[1]) + base_angle;
        double LA_dist = sqrt((goal_position[1]-base_position[1])*(goal_position[1]-base_position[1]) + (goal_position[0]-base_position[0])*(goal_position[0]-base_position[0]));

        // TODO: transform goal point to vehicle frame of reference

    

        // TODO: calculate curvature/steering angle
        float delta = steering_angle(L,alpha,LA_dist);

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.speed=0.7;
        // message.drive.steering_angle = (fabs(delta)>0.17)? delta:0;
        message.drive.steering_angle = delta;

        publisher_->publish(message);

    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}