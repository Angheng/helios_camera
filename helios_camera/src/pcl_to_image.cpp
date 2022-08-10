#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

cv::Mat tmp;

void callback(sensor_msgs::PointCloud2ConstPtr msg) {
    tmp = cv::Mat((int) msg->height, (int) msg->width, CV_32FC4, (void*)msg->data.data());
    cv::imshow("test", tmp);
    cv::waitKey(1);

}

void mouseCallback(int event, int x, int y, int flags, void* user_data) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        ROS_INFO("Clicked => ( %d, %d )\n\n <distance>\nX : %f\nY : %f\nZ : %f\nIntensity : %f",
            x, y, tmp.data[4 * (x + 640*y)], tmp.data[4 * (x + 640*y) + 1], tmp.data[4 * (x + 640*y) + 2], tmp.data[4 * (x + 640*y) + 3]
         );
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_2_img");
    ros::NodeHandle node;

    ros::Subscriber sb = node.subscribe<sensor_msgs::PointCloud2>("/rdv_helios_0001/depth/point", 1, callback);

    ros::spin();

    return 0;
}