#include "virtual_cam/Loader.h"

using namespace std;

ImageLoader::ImageLoader(const std::string& rgb_topic, const std::string& depth_topic, const std::string& info_topic, const std::string& frame_id) : frame_id_(frame_id) {
    nh_ = new ros::NodeHandle();
    pub_rgb = nh_->advertise<sensor_msgs::Image>(rgb_topic, 1000);
    pub_depth = nh_->advertise<sensor_msgs::Image>(depth_topic, 1000);
    pub_cam_info = nh_->advertise<sensor_msgs::CameraInfo>(info_topic, 1000);
}

ImageLoader::~ImageLoader() {
    pub_rgb.shutdown();
    pub_depth.shutdown();
    pub_cam_info.shutdown();
    delete nh_;
}

void ImageLoader::load(const std::string& filename) {
    // Load input bag file
    rosbag::Bag bag_in;
    bag_in.open(filename, rosbag::bagmode::Read);

    // Get topics and time from input bag file
    rosbag::View view(bag_in);

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == "rgb") {
            rgb = *m.instantiate<sensor_msgs::Image>();
            rgb.header.frame_id = frame_id_;
        } else if (m.getTopic() == "depth") {
            depth = *m.instantiate<sensor_msgs::Image>();
            depth.header.frame_id = frame_id_;
        } else if (m.getTopic() == "camera_info") {
            cam_info = *m.instantiate<sensor_msgs::CameraInfo>();
            cam_info.header.frame_id = frame_id_;
        }
    }
}

void ImageLoader::publish() const {
    ros::Time time = ros::Time::now();

    rgb.header.stamp = time;
    depth.header.stamp = time;
    cam_info.header.stamp = time;

    pub_rgb.publish(rgb);
    pub_depth.publish(depth);
    pub_cam_info.publish(cam_info);
}
