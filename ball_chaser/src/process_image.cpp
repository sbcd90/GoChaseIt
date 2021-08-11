#include <ros/ros.h>
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage {
private:
    ros::ServiceClient client;
public:
    ProcessImage(int argc, char** argv) {
        ros::init(argc, argv, "process_image");
        ros::NodeHandle n;

        client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        void (ProcessImage::*func)(const sensor_msgs::Image);
        func = &ProcessImage::process_image_callback;

        ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, func, this);

        ros::spin();
    }

    void drive_robot(float lin_x, float ang_z) {
        ROS_INFO_STREAM("Moving the robot towards the ball");

        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        if (!client.call(srv)) {
            ROS_ERROR("Failed to call service command_robot");
        }
    }

    void process_image_callback(const sensor_msgs::Image image) {
        auto pixel_range = std::make_pair(1, 255);

        auto left = std::make_pair(0, image.step/3);
        auto mid = std::make_pair(image.step/3, (2 * image.step)/3);
        auto right = std::make_pair((2 * image.step)/3, image.step);

        bool ball_found = false;
        for (int i = 0; i < image.height * image.step; ++i) {
            if (image.data[i] == 255 || image.data[i] == 224) {
                int cur_step = i % image.step;

                if (cur_step >= left.first && cur_step < left.second) {
                    drive_robot(0.0, 0.5);
                } else if (cur_step >= mid.first && cur_step < mid.second) {
                    drive_robot(0.5, 0.0);
                } else if (cur_step >= right.first && cur_step < right.second) {
                    drive_robot(0.0, -0.5);
                }

                ball_found = true;
                break;
            }
        }

        if (!ball_found) {
            drive_robot(0.0, 0.0);
        }
    }
};

int main(int argc, char** argv) {
    ProcessImage processImage{argc, argv};
    return 0;
}