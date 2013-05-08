#include "virtual_cam/Image.h"

#include <sensor_msgs/image_encodings.h>

Image Image::loadFromFile(const std::string& filename) {
    // Load input bag file
    rosbag::Bag bag_in;
    bag_in.open(filename, rosbag::bagmode::Read);

    // Get topics and time from input bag file
    rosbag::View view(bag_in);

    Image image;

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
        if (m.getTopic() == "rgb") {
            image.rgb = *m.instantiate<sensor_msgs::Image>();
            //image.rgb.header.frame_id = frame_id_;
        } else if (m.getTopic() == "depth") {
            image.depth = *m.instantiate<sensor_msgs::Image>();
            //image.depth.header.frame_id = frame_id_;
        } else if (m.getTopic() == "camera_info") {
            image.cam_info = *m.instantiate<sensor_msgs::CameraInfo>();
            //image.cam_info.header.frame_id = frame_id_;
        } else if (m.getTopic() == "mask") {
            image.mask = *m.instantiate<sensor_msgs::Image>();
        }
    }

    return image;
}

Image::Image() {
}

Image::~Image() {
}

void Image::saveToFile(const std::string& filename) {
    ros::Time time = ros::Time::now();

    rosbag::Bag bag_out;
    bag_out.open(filename, rosbag::bagmode::Write);

    if (!rgb.data.empty()) {
        bag_out.write("rgb", time, rgb);
    }

    if (!depth.data.empty()) {
        bag_out.write("depth", time, depth);
    }

    if (!mask.data.empty()) {
        bag_out.write("mask", time, mask);
    }

    bag_out.write("camera_info", time, cam_info);
}

cv::Mat Image::getDepthImage() const {
    // Convert depth image
    if (!depth.data.empty()) {
        try {
            cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(depth, "32FC1");
            return depth_img_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    return cv::Mat();
}

cv::Mat Image::getRGBImage() const {
    // Convert RGB image
    if (!rgb.data.empty()) {
        try {
            cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
            return img_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    return cv::Mat();
}

cv::Mat Image::getMaskImage() const {
    // Convert mask image
    if (!mask.data.empty()) {
        try {
            cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::TYPE_8UC1);
            return depth_img_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    return cv::Mat();
}

void Image::setDepthImage(const cv::Mat& image) {
    cv_bridge::CvImage cvi;
    cvi.header.stamp = ros::Time::now();
    cvi.header.frame_id = "";
    cvi.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cvi.image = image;

    cam_info.header.stamp = cvi.header.stamp;
    cam_info.header.frame_id = cvi.header.frame_id;
    cam_info.width = image.cols;
    cam_info.height = image.rows;

    cvi.toImageMsg(depth);
}

void Image::setRGBImage(const cv::Mat& image) {
    cv_bridge::CvImage cvi;
    cvi.header.stamp = ros::Time::now();
    cvi.header.frame_id = "";
    cvi.encoding = sensor_msgs::image_encodings::BGR8;
    cvi.image = image;

    cam_info.header.stamp = cvi.header.stamp;
    cam_info.header.frame_id = cvi.header.frame_id;
    cam_info.width = image.cols;
    cam_info.height = image.rows;

    cvi.toImageMsg(rgb);
}

void Image::setMaskImage(const cv::Mat& image) {
    cv_bridge::CvImage cvi;
    cvi.header.stamp = ros::Time::now();
    cvi.header.frame_id = "";
    cvi.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    cvi.image = image;

    cam_info.header.stamp = cvi.header.stamp;
    cam_info.header.frame_id = cvi.header.frame_id;
    cam_info.width = image.cols;
    cam_info.height = image.rows;

    cvi.toImageMsg(mask);
}
