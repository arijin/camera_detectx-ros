## 手把手教学2-在NVIDIA Jetson  Xavier NX上使用TensorRT加速的yolov5训练自己数据集做目标识别

### **在性能好的电脑上使用yolov5训练自己的数据集**

推荐使用的是这个[代码](github.com/ultralytics/yolov5)。其中给出了官方教程[参考网址](github.com/ultralytics/yolov5/wiki/Train-Custom-Data)，讲解非常详细。

### **yolov5的模型格式转换**

这里使用了[代码](https://github.com/wang-xinyu/tensorrtx)，其中的步骤讲解也非常详细。

### 1、模型格式转换.pt-->.wts

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

### 2、编译tensorrtx下的yolov5，进行模型转换.wts-->.engine，并测试自己的图片。

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
### **Jetson  Xavier NX搭载上Realsense D434i进行实时识别**
