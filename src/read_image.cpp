#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"

using namespace cv;
using namespace std;

int main()
{
    std::cout << "Hello, World!" << std::endl;
    cv::Mat src;
    src = imread("../cmake-build-debug/ubuntu.png", CV_LOAD_IMAGE_COLOR);
    namedWindow("Display", WINDOW_AUTOSIZE);
    imshow("Display", src);
    waitKey(0);
    return 0;
}
