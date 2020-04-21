#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        sub_ = n_.subscribe("/camera/rgb/image_raw", 3, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::Image img) {
        srv_.request.linear_x = 0.0;
        srv_.request.linear_y = 0.0;
        srv_.request.angular_z = 0.0;
        int white_pixel = 255;
        int black_pixel = 0;
        for (int n = 0; n < img.height * img.width; n++) {
            int i = n * 3;
            if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
                int row = n / img.width;
                int column = n - (row * img.width);

                if (column <= (img.width / 4)) {
                    srv_.request.angular_z = 0.5;
                    srv_.request.linear_y = -0.001;
                } else if (column >= (img.width * 3 / 4)) {
                    srv_.request.angular_z = -0.5;
                    srv_.request.linear_y = 0.001;
                } else {
                    srv_.request.angular_z = 0.0;
                    srv_.request.linear_x = 0.15;
                }
                break;
            }
        }


        int check_row = img.height * 12 / 16;
        int check_pixel = check_row * img.step;
        for (int m = 0; m < img.width * (img.height - check_row); m++) {
            if (img.data[check_pixel + m] != black_pixel) {
                int row = m / img.width;
                int column = m - (row * img.width);
                if (column <= img.width / 2) {
                    srv_.request.linear_x = 0.0;
                    srv_.request.linear_y = -0.1;
                    srv_.request.angular_z = 0.05;
                } else {
                    srv_.request.linear_x = 0.0;
                    srv_.request.linear_y = 0.1;
                    srv_.request.angular_z = -0.05;
                }
                break;
            }
        }
        if (!client_.call(srv_))
            ROS_ERROR("Failed to call service safe_move");
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Subscriber odom_;
    ros::ServiceClient client_;
    ball_chaser::DriveToTarget srv_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_and_publish");
    SubscribeAndPublish SAPObject;
    ros::spin();
    return 0;
}
