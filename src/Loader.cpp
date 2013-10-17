#include "virtual_cam/Loader.h"

#include <sensor_msgs/image_encodings.h>

using namespace std;

ImageLoader::ImageLoader() {
    nh_ = new ros::NodeHandle();
}

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
        } else if (m.getTopic() == "depth_camera_info") {
            cam_info_depth = *m.instantiate<sensor_msgs::CameraInfo>();
            cam_info_depth.header.frame_id = frame_id_;
        } else if (m.getTopic() == "tf") {
            geometry_msgs::TransformStamped transform_msg = *m.instantiate<geometry_msgs::TransformStamped>();
            tf::transformStampedMsgToTF(transform_msg, transform);
            transform.child_frame_id_ = frame_id_;
        }
    }
}

void ImageLoader::setDepthCameraTopic(const std::string& topic) {
    pub_cam_info_depth = nh_->advertise<sensor_msgs::CameraInfo>(topic, 1000);
}

cv::Mat ImageLoader::getDepthImage() const {
    // Convert depth image
    try {
        cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(depth, "32FC1");
        return depth_img_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

cv::Mat ImageLoader::getRGBImage() const {
    // Convert RGB image
    try {
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
        return img_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception in imageCallback: %s", e.what());
        return cv::Mat();
    }
}

void ImageLoader::publish() const {
    ros::Time time = ros::Time::now();

    rgb.header.stamp = time;
    depth.header.stamp = time;
    cam_info.header.stamp = time;    
    cam_info_depth.header.stamp = time;

    pub_rgb.publish(rgb);
    pub_depth.publish(depth);
    pub_cam_info.publish(cam_info);
    pub_cam_info_depth.publish(cam_info_depth);

    if (transform.child_frame_id_ != "") {
        transform.stamp_ = time;
        tf_broadcaster.sendTransform(transform);
    }

}
