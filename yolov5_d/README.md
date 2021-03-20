1、用ROS节点打开摄像头(以Intel Realsense D435i为例)

D435i相机需接USB3.0。

```bash
# 安装realsense的ros代码库
sudo apt-get install ros-$ROS_VERSION-realsense2-camera  # 这里我的$ROS_VERSION用melodic替代
sudo apt-get install ros-$ROS_VERSION-realsense2-camera
# 启动图像读取节点
$roslaunch realsense2_camera rs_camera.launch
```

- `roslaunch`时可能会有以下问题导致rgb图像读取不出：

	```
	[WARN] Hardware Notification:USB_CAM overflow,1.61622e+12,Error ,Hardware Error
	```

  这可能是数据输入量过大引起，解决方式是替换上述`roslaunch realsense2_camera rs_camera.launch`为`roslaunch realsense2_camera rs_camera.launch depth_fps:=3 infra_fps:=3 color_fps:=3`。

此节点发布的图像话题有`/camera/color/image_raw`和`/camera/depth/image_rect_raw`分别代表了rgb图像和深度图像。

说明：如果是外接别的USB摄像头使用`ll /dev/video*`找到对应相机的端口号，还是使用ROS官方提供的`usb_cam`包。需更改`usb_cam-test.launch`中的端口号（video_device）和分辨率（image_width, image_height）。

2、识别（这里使用tensorrt对yolov5生成的`.engine`模型参数）

权重模型的engine文件需在当前设备下生成，将其放入`$Your Workspace/src/camera_detectx-ros/yolov5_d/model/`文件目录下替换掉我的`.engine`文件，yolov5的tensorrt的engine文件使用[这个代码](https://github.com/wang-xinyu/tensorrtx)生成。接下来进行图像的订阅和识别：

```bash
$git clone https://github.com/arijin/camera_detectx-ros
$cd camera_detectx-ros
$cp yolov5_d $YOUR_WORKSPACE$/catkin_ws/src
$cd $YOUR_WORKSPACE$/catkin_ws
$catkin_make -DCATKIN_WHITELIST_PACKAGES="yolov5_d"
$source devel/setup.bash
$roslaunch yolov5_d yolov5_d.launch
```

注意`yolov5_d.launch`中的图像话题名字是否正确。
