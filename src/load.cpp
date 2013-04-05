#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// YAML
#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>

// Boost
#include <boost/foreach.hpp>
//#include <boost/filesystem.hpp>
//#include "boost/format.hpp"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc,argv,"NODE_NAME");
    ros::NodeHandle nh;

    if (argc != 2) {
        cout << "Usage: load FILENAME" << endl;
        return 1;
    }

    // Load input bag file
    rosbag::Bag bag_in;
    bag_in.open(argv[1], rosbag::bagmode::Read);
    ROS_INFO("Opened %s", argv[1]);

    // Get topics and time from input bag file
    rosbag::View view(bag_in);

    sensor_msgs::Image rgb;
    sensor_msgs::Image depth;
    sensor_msgs::CameraInfo cam_info;

    ros::Publisher pub_rgb = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1000);
    ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image>("/camera/depth_registered/image", 1000);
    ros::Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1000);

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == "rgb") {
            cout << "RGB" << endl;
            rgb = *m.instantiate<sensor_msgs::Image>();
            cout << "RGB loaded: " << rgb.encoding << endl;
        } else if (m.getTopic() == "depth") {
            cout << "DEPTH" << endl;
            depth = *m.instantiate<sensor_msgs::Image>();
            cout << "DEPTH loaded: " << depth.encoding << endl;
        } else if (m.getTopic() == "camera_info") {
            cout << "INFO" << endl;
            cam_info = *m.instantiate<sensor_msgs::CameraInfo>();
            cout << "CAM INFO loaded: " << cam_info.header << endl;
        }
    }

    string frame_id = "/openni_rgb_optical_frame";

    ros::Rate r(10);
    while(ros::ok()) {
        rgb.header.stamp = ros::Time::now();
        rgb.header.frame_id = frame_id;
        depth.header.stamp = ros::Time::now();
        depth.header.frame_id = frame_id;
        cam_info.header.stamp = ros::Time::now();
        cam_info.header.frame_id = frame_id;

        pub_rgb.publish(rgb);
        pub_depth.publish(depth);
        pub_cam_info.publish(cam_info);
        r.sleep();
    }


}
