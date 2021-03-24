# include <opencv2/opencv.hpp>
# include <string>

using namespace std;

string image_file = "../cmake-build-debug/distorted.png";

int main(int argc, char **argv)
{
    cv::Mat image = cv::imread(image_file, 0);   // 图像是灰度图，CV_8UC1
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 去畸变以后的图
    cv::Size imageSize(cols, rows);

    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
    const double alpha = 1.0;
    const cv::Mat K = ( cv::Mat_<double> (3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    const cv::Mat D = ( cv::Mat_<double> (5, 1) << k1, k2, p1, p2, 0);
    cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0);

    cv::undistort(image, image_undistort, K, D, K);
    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    cv::waitKey();
    return 0;
}