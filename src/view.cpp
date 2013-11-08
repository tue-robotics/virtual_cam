#include "virtual_cam/Image.h"

#include <opencv2/highgui/highgui.hpp>

using namespace std;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: view FILENAME" << endl;
        return 1;
    }

    Image img = Image::loadFromFile(argv[1]);
    cv::Mat rgb = img.getRGBImage();
    cv::Mat depth = img.getDepthImage();

    cv::imshow("rgb", rgb);
    cv::imshow("depth", depth / 7);
    cv::waitKey();
}
