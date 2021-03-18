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

#include "me120_msgs/img_obj.h"
#include "me120_msgs/img_objs.h"
#include "me120_msgs/DarknetObj.h"
#include "me120_msgs/DarknetObjArray.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "yolo_v2_class.hpp"    // imported functions from DLL

#include "image_rectify.h"

static const std::string OPENCV_WINDOW = "darknet window";

std::string image_topic_name;
double  thresh;
Detector *detector=NULL;

std::vector<std::__cxx11::string> obj_names;

double radians(double degree){
    double rad = degree / 180 * M_PI;
    return rad;
}

double degrees(double radian){
    double deg = radian / M_PI * 180;
    return deg;
}

void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names)
{
    int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };

    for (auto &i : result_vec) {
        cv::Scalar color = obj_id_to_color(i.obj_id);
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
        if (obj_names.size() > i.obj_id) {
            std::string obj_name = obj_names[i.obj_id];
            std::cout << obj_name << ":" << i.x << " " << i.y << " " << i.w << " " << i.h <<  std::endl;
            //if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
            cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
            int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
            max_width = std::max(max_width, (int)i.w + 2);
            //max_width = std::max(max_width, 283);

            // std::string coords_3d;
            // if (!std::isnan(i.z_3d)) {
            //     std::stringstream ss;
            //     ss << std::fixed << std::setprecision(2) << "x:" << i.x_3d << "m y:" << i.y_3d << "m z:" << i.z_3d << "m ";
            //     coords_3d = ss.str();
            //     cv::Size const text_size_3d = getTextSize(ss.str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, 1, 0);
            //     int const max_width_3d = (text_size_3d.width > i.w + 2) ? text_size_3d.width : (i.w + 2);
            //     if (max_width_3d > max_width) max_width = max_width_3d;
            // }

            cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
                cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)),
                color, CV_FILLED, 8, 0);
            putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
            // if(!coords_3d.empty()) putText(mat_img, coords_3d, cv::Point2f(i.x, i.y-1), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0, 0, 0), 1);
        }
    }
}

std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

struct Object{
  cv::Point cent_point_2d_rect;
  cv::Point3d cent_point_3d_rect;
  double orientation_angle_;
  int label;
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher darknet_objects_pub;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(image_topic_name, 1, 
      &ImageConverter::imageCb, this);
    darknet_objects_pub = nh_.advertise<me120_msgs::DarknetObjArray>("/darknet_angle_objs", 1);
 
    cv::namedWindow(OPENCV_WINDOW);
  }
 
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
 


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    // cv_bridge::CvImagePtr cv_ptr;
    // try
    // {
    //   cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }

    cv::Mat cv_image_raw;
    cv::Mat cap_frame;
    // rectifyImage(*msg, cv_image_raw, cap_frame);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cap_frame = cv_ptr->image;

    // pre-processing video frame (resize, convertion)
    std::shared_ptr<image_t> det_image;
    det_image = detector->mat_to_image_resize(cap_frame);

    // detection by Yolo
    std::vector<bbox_t> result_vec;
    result_vec = detector->detect_resized(*det_image, cap_frame.cols, cap_frame.rows, thresh, true);  // true

    // Transform roi to image_raw's ordinary.
    // for(auto& result_solo: result_vec){
    //   cv::Rect Roi(result_solo.x, result_solo.y, result_solo.w, result_solo.h);
    //   cv::Rect Roi_rectified;
    //   std::cout << "ori:" << Roi.x << ' ' << Roi.y << ' ' << Roi.width << ' ' << Roi.height << std::endl;
    //   Roi_rectified = unrectifyRoi(Roi);
    //   std::cout << "trans:" << Roi_rectified.x << ' ' << Roi_rectified.y << ' ' << Roi_rectified.width << ' ' << Roi_rectified.height << std::endl;
    //   result_solo.x = Roi_rectified.x;
    //   result_solo.y = Roi_rectified.y;
    //   result_solo.w = Roi_rectified.width;
    //   result_solo.h = Roi_rectified.height;
    // }
    std::vector<Object> objects;
    for(auto& result_solo: result_vec){
      Object solo;
      cv::Point2d cent_point_2d;
      cent_point_2d.x = int(result_solo.x + result_solo.w / 2);
      cent_point_2d.y = int(result_solo.y + result_solo.h / 2);
      solo.cent_point_2d_rect = rectifyPoint(cent_point_2d);
      solo.cent_point_3d_rect = projectPixelTo3dRay(solo.cent_point_2d_rect);
      solo.orientation_angle_ = atan2(solo.cent_point_3d_rect.x, solo.cent_point_3d_rect.z);
      solo.label = result_solo.obj_id;
      std::cout << "label:" << result_solo.obj_id << std::endl;
      objects.push_back(solo);
    }
    
    // draw rectangles (and track objects)
    cv::Mat draw_frame = cap_frame.clone();
    draw_boxes(draw_frame, result_vec, obj_names); 
 
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, draw_frame);
    cv::waitKey(1);
    
    //Publish topic.
    me120_msgs::DarknetObjArray darknet_objects_msg;
    darknet_objects_msg.header = msg->header;

    std::vector<me120_msgs::DarknetObj> darknet_objects;
    // std::cout << "size is:" << objects.size() <<std::endl;
    for(size_t i=0;i<objects.size();i++){
      me120_msgs::DarknetObj darknet_object;
      darknet_object.label = objects[i].label;
      darknet_object.orientation_angle_ = objects[i].orientation_angle_;
      darknet_objects.push_back(darknet_object);
      // std::cout << "angle2:" << darknet_object.orientation_angle_ << ",";
      // std::cout << "label2:" << darknet_object.label << std::endl;
    }
    darknet_objects_msg.objects = darknet_objects;
    darknet_objects_pub.publish(darknet_objects_msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_detect");
  ros::NodeHandle private_node_handle("~");//to receive args

  std::string union_info_url_;
  private_node_handle.param("union_info_url", union_info_url_, std::string("/home/arijin/dataset/BELUGA/calib/latest.yaml"));
  std::string camera_info_url_;
  private_node_handle.param("camera_info_url", camera_info_url_, std::string("/home/arijin/catkin_ws/src/Diaoyu/driver/mono_leopard/usb_cam/camera_info/leopard_left.yaml"));
  infoInit(union_info_url_, camera_info_url_, cv::Size(1920,1020));

	if (private_node_handle.getParam("image_raw_topic", image_topic_name)) 
  {
		ROS_INFO("Setting image topic to %s", image_topic_name.c_str());
	}
  else
  {
		ROS_INFO("No image topic received, defaulting to image_raw, you can use _image_raw_topic:=YOUR_NODE");
		image_topic_name = "/image_raw";
	}

    std::string  names_file;
    if (private_node_handle.getParam("names_file", names_file))
    {
        ROS_INFO("Object Species  File (Config): %s", names_file.c_str());
    }
    else
    {
        ROS_INFO("No Object Species  File was received. Finishing execution.");
        return 0 ;
    }

    std::string  cfg_file;
    if (private_node_handle.getParam("cfg_file", cfg_file))
    {
        ROS_INFO("Network Definition File (Config): %s", cfg_file.c_str());
    }
    else
    {
        ROS_INFO("No Network Definition File was received. Finishing execution.");
        return 0 ;
    }

    std::string  weights_file;
    if (private_node_handle.getParam("weights_file", weights_file))
    {
        ROS_INFO("Pretrained Model File (Weights): %s", weights_file.c_str());
    }
    else
    {
        ROS_INFO("No Pretrained Model File was received. Finishing execution.");
        return 0 ;
    }

    if (private_node_handle.getParam("thresh", thresh))
    {
        ROS_INFO("thresh: %f", thresh);
    }
    else
    {
        ROS_INFO("Probablity treshold default is be set 0.5.");
        thresh=0.3;
    }

    detector = new Detector(cfg_file, weights_file);
    obj_names = objects_names_from_file(names_file);
    // std::cout << obj_names.size() << " " << obj_names[0] << obj_names[1] << obj_names[5] << std::endl;
    ImageConverter ic;
    ros::spin();

    return 0;
}
