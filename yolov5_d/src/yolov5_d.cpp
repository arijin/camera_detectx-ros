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
using namespace std;

#include "NvInfer.h"
#include "cuda_utils.h"
#include "yololayer.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
using namespace nvinfer1;

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#define OPENCV

#include "image_rectify.h"

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
#define DISPLAY true

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1

const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;

std::string camera_info_dir;
std::string image_topic_name;
std::string  names_file;
std::string  engine_file;

std::vector<std::string> obj_names;
IExecutionContext* context=nullptr;
void* buffers[2];
cudaStream_t stream;

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
    std::vector<cv::Mat> cap_frames;
    cap_frames.push_back(cap_frame);
    process_func(cap_frames);
    double time_cost = (ros::Time::now()-begin).toSec();

    //TODO: publish object topic.

    return;
  }

void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
  }

  // single image input
  void process_func(std::vector<cv::Mat> frames){
    // prepare input data ---------------------------
    static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    for (int b = 0; b < BATCH_SIZE; b++) {
      cv::Mat img = frames[b];
      if (img.empty()) continue;
      cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H); // letterbox BGR to RGB
      int i = 0;
      for (int row = 0; row < INPUT_H; ++row) {
        uchar* uc_pixel = pr_img.data + row * pr_img.step;
        for (int col = 0; col < INPUT_W; ++col) {
          data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
          data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
          data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
          uc_pixel += 3;
          ++i;
        }
      }
    }
    // forward ---------------------
    auto start = std::chrono::system_clock::now();
    doInference(*context, stream, buffers, data, prob, BATCH_SIZE);  // 输出prob，prob[0]是输出的个数。
    auto end = std::chrono::system_clock::now();
    std::vector<std::vector<Yolo::Detection>> batch_res(BATCH_SIZE);
    for (int b = 0; b < BATCH_SIZE; b++) {
        auto& res = batch_res[b];
        nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
    }
    // display -----------------------
    if(DISPLAY){
      for (int b = 0; b < BATCH_SIZE; b++){
        std::vector<cv::Rect> bboxes;
        std::vector<std::string> labels;
        std::vector<int> class_ids;
        auto& res = batch_res[b];
        for (size_t j = 0; j < res.size(); j++){
          cv::Rect r = get_rect(frames[b], res[j].bbox);
          std::string label = obj_names[(int)res[j].class_id];
          char str[10];
          sprintf(str, "%.2f", label, res[j].conf);
          std::string prob_text(str);
          std::string label_text = label + " " + prob_text;
          bboxes.push_back(r);
          labels.push_back(label_text);
          std::cout << label_text << std::endl;
          class_ids.push_back((int)res[j].class_id);
        }
        cv::Mat display=frames[b];
        plot_one_image(display, bboxes, labels, class_ids);
        cv::imshow("display window", display);
        cv::waitKey(1);
      }
    }
    return;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yolov5_d");
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
  if(!(_pnh.getParam("engine_file", engine_file))){
    ROS_INFO("Lack engine file!");
    return 0 ;
  }


  std::cout << camera_info_dir << std::endl;
  std::cout << names_file << std::endl;
  std::cout << engine_file << std::endl;

//---------------------CUDA engine deserilize to detect.
  cudaSetDevice(DEVICE);
  // deserialize the .engine and run inference
  std::ifstream file(engine_file, std::ios::binary);
  if (!file.good()) {
      std::cerr << "read " << engine_file << " error!" << std::endl;
      return -1;
  }
  char *trtModelStream = nullptr;
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  trtModelStream = new char[size];
  assert(trtModelStream);
  file.read(trtModelStream, size);
  file.close();

  static float prob[BATCH_SIZE * OUTPUT_SIZE];
  IRuntime* runtime = createInferRuntime(gLogger);
  assert(runtime != nullptr);
  ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
  assert(engine != nullptr);
  context = engine->createExecutionContext();
  assert(context != nullptr);
  delete[] trtModelStream;
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
  const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
  // Create stream
  CUDA_CHECK(cudaStreamCreate(&stream));

  obj_names = objects_names_from_file(names_file);

  std::cout << "------------start to detect!--------------" << std::endl;
  ImageConverter ic;
  ros::spin();

  return 0;
}
