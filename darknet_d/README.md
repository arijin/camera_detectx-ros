1、用ROS节点打开摄像头

首先下载usb_cam包，是ROS开源的相机驱动程序，用于相机图像读取。

```bash
$cd ~
$mkdir catkin_ws
$cd catkin_ws
$mkdir src
$cd src
$git clone https://github.com/ros-drivers/usb_cam
$ cd ../
$catkin_make
$source devel/setup.bash
$roslaunch usb_cam usb_cam-test.launch
```

如果是外接摄像头需更改`usb_cam-test.launch`中的端口号（video_device）和分辨率（image_width, image_height）。

2、识别(以coco数据集为例)

权重文件：[yolov4-tiny.weight](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights)或者[yolov4.weight](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights)需自行下载；模型配置文件及coco数据集类别文件，都已经提供了。

使用`rostopic list`查看图像所在话题。

```bash
$git clone https://github.com/arijin/camera_detectx-ros
$cd camera_detectx-ros
$cp darknet_d $YOUR_WORKSPACE$/catkin_ws/src
$cd $YOUR_WORKSPACE$/catkin_ws
$catkin_make -DCATKIN_WHITELIST_PACKAGES="darknet_d"

```


