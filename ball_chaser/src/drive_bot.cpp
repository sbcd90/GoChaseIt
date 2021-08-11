#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"

class DriveBot {
private:
    ros::Publisher motor_command_publisher;

public:
    DriveBot(int argc, char** argv) {
        ros::init(argc, argv, "drive_bot");
        ros::NodeHandle n;

        this->motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        bool (DriveBot::*func)(ball_chaser::DriveToTarget::Request&,
                ball_chaser::DriveToTarget::Response&);
        func = &DriveBot::handle_command_robot_request;

        ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", func, this);

        ROS_INFO("Ready to send wheel velocity commands");
        ros::spin();
    }

    bool handle_command_robot_request(ball_chaser::DriveToTarget::Request& req,
                                      ball_chaser::DriveToTarget::Response& resp) {
        ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", (float) req.linear_x, (float) req.angular_z);

        geometry_msgs::Twist motor_command;
        motor_command.linear.x = (float) req.linear_x;
        motor_command.angular.z = (float) req.angular_z;

        motor_command_publisher.publish(motor_command);

        resp.msg_feedback = "Linear & Angular velocities set - linear_x: " + std::to_string((float) req.linear_x) + " , angular_z: " + std::to_string((float) req.angular_z);
        ROS_INFO_STREAM(resp.msg_feedback);

        return true;
    }
};

int main(int argc, char** argv) {
    DriveBot driveBot{argc, argv};
    return 0;
}