### **1、Intel RealSense D435i 深度相机图像读取（ROS版）**

##### a. Rviz 安装

```shell
$sudo apt-get install ros-melodic-rviz
$rosdep install rviz
$rosmake rviz
```

##### b. RealSense的ROS库安装

首先保证安装好了ROS，这里不讲ROS的安装了。直接在终端输入下面两句话即可：

```bash
$sudo apt-get install ros-$ROS_VER-realsense2-camera  # $ROS_VER改称对应ROS的版本，这里我是melodic
$sudo apt-get install ros-$ROS_VER-realsense2-description
```

##### c. 相机数据读入并发布图像话题

```bash
$roslaunch realsense2_camera rs_camera.launch
```

注意可能会报以下错误警告：

```bash
[WARN] Hardware Notification:USB CAM overflow,1.61589e+12,Error,Hardware Error
```

根据文字表述，这是由于单USB口数据量过大影起的，因此在`roslaunch`中直接定义`rs_camera.launch`中的帧率：

```bash
$roslaunch realsense2_camera rs_camera.launch depth_fps:=3 infra_fps:=3 color_fps:=3
```

正常以后，利用Rviz可视化验证图像读取正常,使用Image组件订阅`/camera/color/image_raw`话题:

```bash
$rosrun rviz rviz
```

- 注意点

1、Intel D435i相机需接USB3.0接口

2、此节点发布的图像话题有`/camera/color/image_raw`和`/camera/depth/image_rect_raw`分别代表了rgb图像和深度图像。

3、说明：如果是外接别的USB摄像头使用`ll /dev/video*`找到对应相机的端口号，还是使用ROS官方提供的`usb_cam`包。需更改`usb_cam-test.launch`中的端口号（video_device）和分辨率（image_width, image_height）。

### **Jetson  Xavier NX搭载上Realsense D434i进行实时识别**

##### a. **在性能好的电脑上使用yolov5训练自己的数据集**

推荐使用的是这个[代码](github.com/ultralytics/yolov5)。其中给出了官方教程[参考网址](github.com/ultralytics/yolov5/wiki/Train-Custom-Data)，讲解非常详细。

##### b. **yolov5的模型格式转换**

这里使用了[代码](https://github.com/wang-xinyu/tensorrtx)，其中的步骤讲解也非常详细。

**1、模型格式转换.pt-->.wts**

将pytorch下的`.pt`模型参数文件转换为`.wts`格式，建议直接在大电脑上完成。以下也以大电脑为例。

```bash
$git clone https://github.com/wang-xinyu/tensorrtx
$cd ./tensorrtx/yolov5
$git clone https://github.com/ultralytics/yolov5
$cp gen_wts.py ./yolov5/
$cd yolov5
# 将之前自己训练好的pt参数文件拷贝到ultralytics的yolov5的根目录下，我的为best.pt。
$gedit gen_wts.py 
# 将model = torch.load('weights/yolov5s.pt', map_location=device)['model'].float()  # load to FP32改为
#     model = torch.load('best.pt', map_location=device)['model'].float()  # load to FP32
$python gen_wts.py 
# 最后将生成一个yolov5s.wts，因为训练时选择的是s大小的模型，所以这样取名。
```

**2、编译tensorrtx下的yolov5，进行模型转换.wts-->.engine，并测试自己的图片。**

必须在边缘设备Jetson  Xavier NX上完成。编译成功需要安装opencv，版本>=3.3。在下面`cmake`时可以注意一下系统寻找到的opencv版本。

```bash
$git clone https://github.com/wang-xinyu/tensorrtx
$cd ./tensorrtx/yolov5
# important! 在编译之前，修改当前目录下的yololayer.h中的CLASS_NUM变量为你自己数据集的类别数。
# 其中yololayer.h中INPUT_H和INPUT_W，以及yolov5.cpp中的USE_FP16, DEVICE, NMS_THRESH, CONF_THRESH, BATCH_SIZE都可根据你的需求改动。这里仅修改yololayer.h中的CLASS_NUM。
$mkdir build
$cd build
$cmake ..
$make
# 把之前生成的yolov5s.wts拷贝到当前build目录下。
$./yolov5 -s yolov5s.wts yolov5s.engine s  # 第一个-s代表序列化，第二个s代表选择yolov5的s模型。
$./yolov5 -d yolov5s.engine ../custom_samples  # 先把要测试的图片拷贝到根目录下新建的custom_samples下，然后运行这句话，输出图片直接放在build目录下。
```

###  **订阅图像话题进行图像识别**

下载识别[代码](https://github.com/arijin/camera_detectx-ros)。

权重模型的engine文件需在当前设备下用上一节的方法生成，将其放入`/camera_detectx-ros/yolov5_d/model/`文件目录下替换掉我的`.engine`文件，

- 如果你是在NVIDIA Jetson系列的边缘设备上运行，要修改`camera_detectx-ros/yolov5_d/`中的`CMakeLists.txt`文件，将

    ```cmake
    include_directories(/usr/include/x86_64-linux-gnu/)
    link_directories(/usr/lib/x86_64-linux-gnu/)
    ```

    替换为：

    ```cmake
    include_directories(/usr/include/aarch64-linux-gnu/)
    link_directories(/usr/lib/aarch64-linux-gnu/)
    ```

接下来图像的订阅和识别程序：

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
