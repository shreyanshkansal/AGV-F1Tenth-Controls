#include <sstream>
#include <fstream>
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
    vector<double> goal_position;
    // vector<double> nearest_wp;

    vector<pair<double,double>> waypoints;

    float base_angle = 0;
    float L; //baselink to wheel hinge midpoint distance
    float bot_speed=8; //initial speed without rectification
    double alpha=0;
    int target_index=0;
    float lad =2; //look-ahead distance
    float K=1.25 ; //angle rectification factor
    float LF=0.2;
    double steering_angle(float L, float alpha, float D){
        // cout << alpha << endl;
        // cout << D << endl;
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

        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform from 'map' to 'base_link 11': %s", ex.what());
        }
    }

    void readCoordinates(const std::string &filePath) {
        std::ifstream file(filePath);
        std::string line;
        int index = 0;

        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file!" << filePath << std::endl;
            
        }

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> coordinates;


            // Split the line by comma and extract x, y, z
            while (std::getline(ss, value, ',')) {
                try {
                    coordinates.push_back(std::stod(value));
                } catch (const std::invalid_argument &e) {
                    std::cerr << "Error: Invalid number format at line " << index << std::endl;
                    break;
                }
            }

            if (coordinates.size()==2) {
                waypoints.push_back({coordinates[0],coordinates[1]});
                
            } else {
                std::cerr << "Error: Incorrect data format at line " << index << std::endl;
            }

            ++index;
        }
        file.close();

    }   

    void initiate_target_index(){
        // return;     //comment out this return to use the function
        cout<<"RESET ACTIVATED"<<endl;
        // int size = (int)waypoints.size();
        float min = 12000;
        for(int i=0;i<150;i++){

            float dx = waypoints[i].first - base_position[0];
            float dy = waypoints[i].second - base_position[1] ; 
            float dist = sqrt(pow(dx,2)+pow(dy,2));
            float local_x = dx * cos(base_angle) + dy * sin(base_angle);
            float local_y = -dx * sin(base_angle) + dy * cos(base_angle);
            float temp_alpha = atan(local_y/local_x);
            
            if(dist>lad and fabs(temp_alpha)<1.56 and dist<min) { 
                cout << min << endl;
                min=dist;
                target_index=i;
            }
            
        }
    }

    vector<double> choose_waypoint(){
        int size = (int)waypoints.size();
        if(target_index>=size) initiate_target_index();
        
        for(int i=target_index;i<size;i++){

            float dx = waypoints[i].first - base_position[0];
            float dy = waypoints[i].second - base_position[1] ; 
            float dist = sqrt(pow(dx,2)+pow(dy,2));
            float local_x = dx * cos(base_angle) + dy * sin(base_angle);
            float local_y = -dx * sin(base_angle) + dy * cos(base_angle);
            alpha = atan(local_y/local_x);
            // lad = LF*bot_speed;
            if(dist>lad and fabs(alpha)<1.56) { 
                // target_index = i;
                cout << target_index <<endl;
                break;
            }
            target_index++;
        };
        return {waypoints[target_index].first,waypoints[target_index].second,0};
    }



public:
    PurePursuit() : Node("pure_pursuit_node"),tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
    {
        //Generates list of waypoints from the csv
        readCoordinates("/home/utsab/ROS2_Workspaces/sim_ws/src/f1tenth_lab6_template/pure_pursuit/src/Spielberg_waypoints.csv");

        // initiate_target_index();

        //Defined the publisher
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        //timer which loops every 500ms and updates stuff for the bot
        timer_ = this->create_wall_timer(20ms, std::bind(&PurePursuit::navigator, this));
    }


    void navigator()
    {
        //gets the bot position and yaw
        get_transform();

        //chooses the appropriate waypoint based on the different parameters and updates the target goal
        goal_position = choose_waypoint();

        RCLCPP_INFO(this->get_logger(), "Target Waypoint: (%f,%f,%f)",goal_position[0],goal_position[1],goal_position[2]);
        RCLCPP_INFO(this->get_logger(), "Bot Position: (%f,%f,%f)",base_position[0],base_position[1],base_position[2]);

        // TODO: calculate curvature/steering angle
        float delta = steering_angle(L,alpha,lad);

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.speed=bot_speed*(1-K*fabs(alpha)/1.57); //speed rectified based on deviation from target. Allows the car to be stable at high speed turns.
        // message.drive.steering_angle = (fabs(delta)>0.5)? 0.5*(delta)/fabs(delta):delta;
        message.drive.steering_angle = delta;
        
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Speed: %.2f,Steering Angle : %.2f,Base Angle: %.2f, Alpha = %.2f",message.drive.speed,message.drive.steering_angle,base_angle,alpha);
        RCLCPP_INFO(this->get_logger(), "XXX=============================================XXX");
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