#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>         // std::mutex, std::unique_lock
#include <cmath>
#define OPENCV

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "yolo_v2_class.hpp"    // imported functions from DLL
#include "image_rectify.h"

#define CONF_THRESH_DEFAULT 0.5

std::string camera_info_dir;
std::string image_topic_name;
double  conf_thresh;
std::string  names_file;
std::string  cfg_file;
std::string  weights_file;

std::vector<std::__cxx11::string> obj_names;
Detector *detector=NULL;

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter(): it_(nh_){
    image_sub_ = it_.subscribe(image_topic_name, 1, &ImageConverter::imageCb, this);
  }
 
  ~ImageConverter(){}

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat cap_frame;
    std_msgs::Header current_header = msg->header;

    // undistort the raw image.
    // cv::Mat cv_image_raw;
    // rectifyImage(*msg, cv_image_raw, cap_frame);

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cap_frame = cv_ptr->image;
    
    ros::Time begin = ros::Time::now();
    std::vector<bbox_t> result_vec_;
    result_vec_ = process_func(cap_frame);
    double time_cost = (ros::Time::now()-begin).toSec();

    //TODO: publish object topic.

    ROS_INFO("time cost:%lfs, detect the number of objects:%d, object:", time_cost,  result_vec_.size());
    for(size_t i=0;i<result_vec_.size();i++){
      std::cout << obj_names[result_vec_[i].obj_id] << " ";
    }
    std::cout << std::endl;
    return;
  }

 std::vector<bbox_t> process_func(cv::Mat frame){
    // pre-processing video frame including "resize", "convertion".
    std::shared_ptr<image_t> det_image;
    det_image = detector->mat_to_image_resize(frame);

    // detection by Yolo
    std::vector<bbox_t> result_vec;
    result_vec = detector->detect_resized(*det_image, frame.cols, frame.rows, conf_thresh, true);

    return result_vec;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_d");
  ros::NodeHandle _pnh("~");

  std::string base_dir;
  if(!(_pnh.getParam("base_dir", base_dir))){
    ROS_INFO("Unknown base address!");
    return 0 ;
  }
  
  if(_pnh.getParam("camera_info_dir", camera_info_dir)){
    infoInit(camera_info_dir);
  }  
  _pnh.param("image_topic_name", image_topic_name, std::string("/image_raw"));
  if(!(_pnh.getParam("names_file", names_file))){
    ROS_INFO("Lack objects name file!");
    return 0 ;
  }
  if(!(_pnh.getParam("cfg_file", cfg_file))){
    ROS_INFO("Lack model cfg file!");
    return 0 ;
  }
  if(!(_pnh.getParam("weights_file", weights_file))){
    ROS_INFO("Lack weight file!");
    return 0 ;
  }
  _pnh.param("conf_thresh", conf_thresh, CONF_THRESH_DEFAULT);

  std::cout << camera_info_dir << std::endl;
  std::cout << names_file << std::endl;
  std::cout << cfg_file << std::endl;
  std::cout << weights_file << std::endl;

  obj_names = objects_names_from_file(names_file);
  detector = new Detector(cfg_file, weights_file);

  ImageConverter ic;
  ros::spin();

  return 0;
}
