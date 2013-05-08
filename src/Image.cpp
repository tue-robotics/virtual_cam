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
            image.rgb_msg_ = *m.instantiate<sensor_msgs::Image>();
            try {
                cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image.rgb_msg_, sensor_msgs::image_encodings::BGR8);
                image.rgb_image_ = img_ptr->image;
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        } else if (m.getTopic() == "depth") {
            image.depth_msg_ = *m.instantiate<sensor_msgs::Image>();
            try {
                cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(image.depth_msg_, "32FC1");
                image.depth_image_ = depth_img_ptr->image;
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());

            }
        } else if (m.getTopic() == "mask") {
            image.mask_msg_ = *m.instantiate<sensor_msgs::Image>();
            try {
                cv_bridge::CvImagePtr depth_img_ptr = cv_bridge::toCvCopy(image.mask_msg_, sensor_msgs::image_encodings::TYPE_8UC1);
                image.mask_image_ = depth_img_ptr->image;
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        } else if (m.getTopic() == "camera_info") {
            image.cam_info = *m.instantiate<sensor_msgs::CameraInfo>();
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

    if (rgb_image_.cols > 0 ) {
        cv_bridge::CvImage cvi;
        cvi.header.stamp = time;
        cvi.header.frame_id = "";
        cvi.encoding = sensor_msgs::image_encodings::BGR8;
        cvi.image = rgb_image_;
        cvi.toImageMsg(rgb_msg_);
        bag_out.write("rgb", time, rgb_msg_);
    }

    if (depth_image_.cols > 0) {
        cv_bridge::CvImage cvi;
        cvi.header.stamp = time;
        cvi.header.frame_id = "";
        cvi.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        cvi.image = depth_image_;
        cvi.toImageMsg(depth_msg_);
        bag_out.write("depth", time, depth_msg_);
    }

    if (mask_image_.cols > 0) {
        cv_bridge::CvImage cvi;
        cvi.header.stamp = time;
        cvi.header.frame_id = "";
        cvi.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        cvi.image = mask_image_;
        cvi.toImageMsg(mask_msg_);
        bag_out.write("mask", time, mask_msg_);
    }

    cam_info.header.stamp = time;
    cam_info.header.frame_id = "";
    cam_info.width = rgb_image_.cols; // todo: check if size of all modes (rgb, depth, ...) is equal
    cam_info.height = rgb_image_.rows;
    bag_out.write("camera_info", time, cam_info);
}

cv::Mat Image::getDepthImage() const {
    return depth_image_;
}

cv::Mat Image::getRGBImage() const {
    return rgb_image_;
}

cv::Mat Image::getMaskImage() const {
    return mask_image_;
}

void Image::setDepthImage(const cv::Mat& image) {
    depth_image_ = image;
}

void Image::setRGBImage(const cv::Mat& image) {
    rgb_image_ = image;
}

void Image::setMaskImage(const cv::Mat& image) {
    mask_image_ = image;
}
