#include "virtual_cam/Image.h"

#include <opencv2/highgui/highgui.hpp>

#include <rgbd/serialization.h>


std::vector<cv::Point2d> mouse_points;

cv::Mat rgb;
cv::Mat rgb_resized;
cv::Mat depth;
double factor;

// ----------------------------------------------------------------------------------------------------

void Draw()
{
    cv::Mat img = rgb_resized.clone();
    for(std::vector<cv::Point2d>::const_iterator it = mouse_points.begin(); it != mouse_points.end(); ++it)
    {
        cv::circle(img, *it * img.cols, 3, cv::Scalar(0, 0, 255),2);
    }

    if (mouse_points.size() == 2)
    {
        cv::rectangle(img, mouse_points[0] * img.cols, mouse_points[1] * img.cols, cv::Scalar(255, 0, 0), 1);
    }

    cv::imshow("rgb", img);
    cv::imshow("depth", depth / 7);
}

// ----------------------------------------------------------------------------------------------------

Image Crop()
{
    cv::Point2i p1 = rgb_resized.cols * mouse_points[0];
    cv::Point2i p2 = rgb_resized.cols * mouse_points[1];
    cv::Mat rgb_cropped = rgb_resized(cv::Rect(p1, p2));

    cv::Mat depth_cropped = depth(cv::Rect(p1, p2));

    cv::imshow("crop", rgb_cropped);
    cv::imshow("depth_cropped", depth_cropped);
    cv::waitKey();

    Image image;
    image.setDepthImage(depth_cropped);
    image.setRGBImage(rgb_cropped);

    cv::Mat mask(rgb_cropped.rows, rgb_cropped.cols, CV_8UC1, cv::Scalar(255));
    image.setMaskImage(mask);

    return image;
}

// ----------------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    cv::Point2d mouse_pos(x / (double)rgb_resized.cols, y / (double)rgb_resized.cols);

    if  ( event == cv::EVENT_LBUTTONDOWN )
    {
        if (mouse_points.size() == 2)
            mouse_points.clear();

        mouse_points.push_back(mouse_pos);
    }/*
    else if  ( event == cv::EVENT_RBUTTONDOWN )
    {
        //          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if  ( event == cv::EVENT_MBUTTONDOWN )
    {
        //          std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
//        mouse_pos = cv::Vec2i(x, y);
    }*/
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "Usage: crop FILENAME" << std::endl;
        return 1;
    }

    std::string filename = argv[1];

    // Old
//    Image img = Image::loadFromFile(argv[1]);
//    rgb = img.getRGBImage();
//    depth = img.getDepthImage();

    // -----------------------------------------

    rgbd::ImagePtr image(new rgbd::Image);

    std::ifstream f_in;
    f_in.open(filename.c_str(), std::ifstream::binary);

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << filename << "'." << std::endl;
        return 1;
    }

    tue::serialization::InputArchive a_in(f_in);
    rgbd::deserialize(a_in, *image);

    rgb = image->getRGBImage();
    cv::resize(rgb, rgb_resized, cv::Size(rgb.cols / 2, rgb.rows / 2));

    depth = image->getDepthImage();

    // -----------------------------------------

    factor = (double)rgb_resized.cols / depth.cols;

    //Create a window
    cv::namedWindow("rgb", 1);

    //set the callback function for any mouse event
    cv::setMouseCallback("rgb", CallBackFunc, NULL);

    while(true)
    {
        Draw();
        int key = (unsigned char)cv::waitKey(30);

        if (key != 255)
        {
            Image image = Crop();
            image.saveToFile(filename + ".cropped");
            break;
        }
    }

}
