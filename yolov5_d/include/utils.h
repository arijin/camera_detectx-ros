#ifndef TRTX_YOLOV5_UTILS_H_
#define TRTX_YOLOV5_UTILS_H_

#include <dirent.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

static inline cv::Mat preprocess_img(cv::Mat& img, int input_w, int input_h) {
    int w, h, x, y;
    float r_w = input_w / (img.cols*1.0);
    float r_h = input_h / (img.rows*1.0);
    if (r_h > r_w) {
        w = input_w;
        h = r_w * img.rows;
        x = 0;
        y = (input_h - h) / 2;
    } else {
        w = r_h * img.cols;
        h = input_h;
        x = (input_w - w) / 2;
        y = 0;
    }
    cv::Mat re(h, w, CV_8UC3);
    cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(input_h, input_w, CV_8UC3, cv::Scalar(128, 128, 128));
    re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
    return out;
}

static inline int read_files_in_dir(const char *p_dir_name, std::vector<std::string> &file_names) {
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr) {
        return -1;
    }

    struct dirent* p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr) {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0) {
            //std::string cur_file_name(p_dir_name);
            //cur_file_name += "/";
            //cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }

    closedir(p_dir);
    return 0;
}

//--------------------------plots
std::vector<cv::Scalar> color_list = {cv::Scalar(31, 119, 180),
                                                                         cv::Scalar(255, 127, 14),
                                                                         cv::Scalar(44, 160, 44),
                                                                         cv::Scalar(214, 39, 40),
                                                                         cv::Scalar(148, 103, 189),
                                                                         cv::Scalar(140, 86, 75),
                                                                         cv::Scalar(227, 119, 194),
                                                                         cv::Scalar(127, 127, 127),
                                                                         cv::Scalar(188, 189, 34),
                                                                         cv::Scalar(23, 190, 207)};

void plot_one_box(cv::Mat img, cv::Rect bbox, std::string label, cv::Scalar color, int line_thickness=2){
    //bbox
    cv::rectangle(img, bbox, color, line_thickness);
    //label
    int fontFace = cv::FONT_HERSHEY_SIMPLEX  ;
    double fontScale = 0.8;
    int baseline=0;
    cv::Size textSize = cv::getTextSize(label, fontFace, fontScale, line_thickness, &baseline);
    cv::rectangle(img, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x+textSize.width, bbox.y-textSize.height-3), color, -1);
    cv::putText(img, label, cv::Point(bbox.x, bbox.y-2), fontFace, fontScale, cv::Scalar::all(255), line_thickness, 8);
}


void plot_one_image(cv::Mat image, std::vector<cv::Rect> bboxes, 
                                              std::vector<std::string> labels, std::vector<int> obj_ids, int max_size=640){
  for(int i=0;i<labels.size();i++){
      cv::Scalar color = color_list[obj_ids[i]%color_list.size()];
      plot_one_box(image, bboxes[i], labels[i], color);
  }
  int width = image.size().width;
  int height = image.size().height;
  float scale_factor = max_size / (float)max(width, height);
  if(scale_factor<1){
      cv::resize(image, image, cv::Size((int)(scale_factor*width), (int)(scale_factor*height)));
  }
}


#endif  // TRTX_YOLOV5_UTILS_H_

