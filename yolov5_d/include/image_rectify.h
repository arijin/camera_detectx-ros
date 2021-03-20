#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
using namespace std;
using namespace cv;

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



void infoInit(std::string camera_info_dir);
cv::Point2d project3dToPixel(const cv::Point3d &xyz);
cv::Point3d projectPixelTo3dRay(const cv::Point2d &uv_rect);
void rectifyImage(const sensor_msgs::Image &msg,  cv::Mat &raw, cv::Mat &undistort);
cv::Point2d rectifyPoint(const cv::Point2d &uv_raw);
cv::Point2d unrectifyPoint(const cv::Point2d &uv_rect);
cv::Rect rectifyRoi(const cv::Rect& roi_raw);
cv::Rect unrectifyRoi(const cv::Rect& roi_rect);


