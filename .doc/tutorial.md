## 手把手教学-在NVIDIA Jetson  Xavier NX上使用TensorRT加速的yolov5训练自己数据集做目标识别

### **注意！** 
- 所有安装包，包括github里的代码，建议使用科学上网先下载下来；github的代码要注意解压后的命名以及放置位置。
- (optional)建议在硬件配置相同的情况下，在配置好以下必要软件环境后，制作自己的系统镜像，之后重装系统会很方便。

### **前期工作**

#### 1、pip换源
必须完成，否则后续使用`pip install`经常会卡住下载不动。
```ruby
$cd ~
$mkdir .pip
$cd .pip
$touch pip.conf
```
在`pip.conf`文件中放入以下文字即可：
```
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
```

#### 2、更新cmake到更新的版本
在cmake的[官网](cmake.org/download)下载最新版本的cmake，注意要选择arm64架构的压缩包。
```bash
tar -xzvf cmake-3.19.7-Linux-aarch64.tar.gz

# 解压出来的包，将其放在 /opt 目录下，其他目录也可以，主要别以后不小心删了
sudo mv cmake-3.19.7-Linux-aarch64 /opt/cmake-3.19.7

# 建立软链接
sudo ln -sf /opt/cmake-3.19.7/bin/*  /usr/bin/

# 查看 cmake 版本
cmake --version
```


### **必备软件安装**

#### 1、VS Code（Markdown）

从官网下载ubuntu下的**arm64**的deb包-[Download](https://code.visualstudio.com/download#)。
``` ruby
$sudo dpkg -i code_1.54.3-1615805708_arm64.deb
```
VS Code安装完成，非常简便。

打开VS Code，点击最左侧图标最下方一个，安装扩展应用（必装：c/c++，python，Markdown）。

#### 2、中文拼音输入法

参考[网址](https://my.oschina.net/u/4256916/blog/3311770)按步骤完成即可。安装完成重启后，在屏幕右上角的输入法中进入文本输入设置，找到chinese其中有一个"汉语(Intelligent Pinyin)iBus"，将其添加即可。**注意**：最常用的搜狗拼音没有arm内核的安装包。

#### 3、pyenv 安装

为了保护系统python的环境，并且为每一个项目建立各自的python环境，安装pyenv做环境隔离。

安装非常简便，github上安装[source zip](github.com/pyenv/pyenv)，根据readme中的Installation的步骤即可，下面按步骤说明：
``` bash
$git clone https://github.com/pyenv/pyenv.git ~/.pyenv
```
``` bash
$echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
$echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
```
```bash
$echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.bashrc
```
```bash
$exec "$SHELL"
```
再使用例如以下命令进行指定python版本的安装：
``` bash
$pyenv install 3.6.3
```
安装完成，用于python环境管理的常用命令有以下一些：
``` bash
$pyenv install -l  # 查看所有能安装的python版本。
$pyenv versions  # 查看当前终端使用的python环境，以及所有已安装的环境名称。
$pyenv shell/local/global 环境名称  # 更换python环境到 当前终端/当前目录层级以下打开的所有终端/所有地方打开的终端。
```

#### 4、Pytorch 安装

根据[官方网址](https://elinux.org/Jetson_Zoo)给的教程步骤安装，在其中找到Pytorch栏。这里将其写出，主要分为两个步骤：

a. 下载对应JetPack版本的`.wheel`文件（必须使用科学上网下载）。

b. 使用以下命令行安装：（注意切换到指定的pyenv的python环境中！）

```bash
# install OpenBLAS and OpenMPI
$ sudo apt-get install libopenblas-base libopenmpi-dev

# Python 2.7 (download pip wheel from above)
$ pip install future torch-1.4.0-cp27-cp27mu-linux_aarch64.whl

# Python 3.6 (download pip wheel from above)
$ sudo apt-get install python3-pip
pip3 install Cython
pip3 install numpy torch-1.6.0-cp36-cp36m-linux_aarch64.whl
```
安装完成，使用终端下的python做一些检查：
```python
import torch
print(torch.cuda.is_available())  # 输出 True
print(torch.cuda.device_count())  # 输出1
print(torch.cuda.current_device())  # 输出0
print(torch.cuda.get_device_name())  # 输出Xavier
```




### **Intel RealSense D435i 深度相机图像读取（ROS版）**

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

此节点发布的图像话题有`/camera/color/image_raw`和`/camera/depth/image_rect_raw`分别代表了rgb图像和深度图像。

说明：如果是外接别的USB摄像头使用`ll /dev/video*`找到对应相机的端口号，还是使用ROS官方提供的`usb_cam`包。需更改`usb_cam-test.launch`中的端口号（video_device）和分辨率（image_width, image_height）。