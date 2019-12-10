[EN](README.en.md)|CN

## 人脸识别demo

本人脸识别demo运行于Atlas 300产品，接收视频流数据，对视频中的人脸进行检测并进行特征匹配，输出匹配信息并在前端显示，主要流程如下：

(1)Host侧接受RTSP视频流，发送到Device侧，通过硬件解码模块进行解码，同时将解码结果传送回host侧；
(2)通过目标检测模型进行人脸检测；
(3)对于检测到的人脸通过关键点检测模型提取关键点，并进行仿射变换矫正；
(4)使用模型提取人脸特征；
(5)使用一种人脸跟踪算法通过减少(3)和(4)的计算量来显著降低端到端计算量；
(6)输出视频流和人脸识别结果的结构化信息进行同步并显示。

[TOC]

### 支持的产品

Atlas 300 (Model 3000), Atlas 300 (Model 3010)

### 支持的版本

1.31.T15.B150 或 1.3.2.B893

可通过执行以下命令获取

```
npu-smi info
```

### 操作系统

Ubuntu 16.04 或 CentOS 7.4

### 目录结构

```
├── build
│   ├── build_local.sh
│   └── cmake
├── depository
│   ├── config
├── src
│   ├── common
│   ├── device
│   └── host
```

所有源文件位于src目录中；在depository中包含所有依赖项目；build目录包含编译脚本。

### 环境依赖

(1) FFmpeg

拉流和推流功能基于FFmpeg，请参考FFmpeg开源工程进行源码编译和安装，并确保FFmpeg的安装目录下包含include和lib文件夹，如下所示：

```
├── FFmpeg
|   ├── include
|   ├── lib
```



### 运行方法

#### 1.建立标准人脸库

在depository目录下新建featurelib文件夹，并在depository/featurelib目录下新建名为Featurelib.bin的人脸库。Featurelib.bin为二进制文件，内容为多个人脸特征（每个人脸512维，float32类型）的连续排列存放，如下所示：

```
特征向量1向量特征向量2特征向量3...特征向量n
```

用户可通过开源人脸特征提取模型提取人脸特征向量，并写入二进制文件；或使用python脚本随机生成特定字节长度（人脸个数x512x4）的二进制文件。

注：本人脸识别demo的人脸比对功能仅做调试使用。

#### 2.修改配置文件

本工程共有两个配置文件，位于depository/config下。其中graph.config为Matrix引擎使用的graph配置文件，setup.config 为demo输入视频流的配置文件。下两表分别总结了这两个配置文件的主要配置项及说明。

​                                                                       表1：graph.config配置说明

| 配置项                 | 所属engine                                               | 说明                                                         |
| ---------------------- | -------------------------------------------------------- | ------------------------------------------------------------ |
| rtsp_link              | StreamDataOutputEngine                                   | 设定输出内容接收地址，使用rtsp协议。格式为rtsp://aa.bbb.ccc.ddd/live.sdp，其中aa.bbb.ccc.ddd为接收rtsp视频流的ip地址 |
| feature_lib_path       | StreamDataOutputEngine                                   | 特征库二进制文件路径                                         |
| feature_len            | StreamDataOutputEngine                                   | 特征库中每个特征的长度（特征向量元素个数）                   |
| feature_num            | StreamDataOutputEngine                                   | 特征库特征数量                                               |
| batch_size             | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | 引擎使用的模型输入的batchsize，用于校验模型的实际值          |
| input_channel          | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | 引擎使用的模型输入的通道数，用于校验模型的实际值             |
| input_width            | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | 引擎使用的模型输入的宽，用于校验模型的实际值                 |
| input_height           | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | 引擎使用的模型输入的高，用于校验模型的实际值                 |
| max_face_num_per_frame | FaceDetectionEngine FaceLandmarkEngine                   | 每帧最大保留人脸框数量，使用时必须确保FaceDetectionEngine和FaceLandmarkEngine两个引擎中的值保持一致 |
| adjacencyThreshold     | SORTEngine                                               | SORT算法中匈牙利匹配算法的阈值，建议设定为0.2左右            |
| trackThreshold         | SORTEngine                                               | 跟踪策略阈值，建议设定为0.6~0.8                              |

​                                                                            表2：setup.config 配置说明

| 配置项          | 说明                                                         |
| --------------- | ------------------------------------------------------------ |
| device_id       | 指定所运行的Ascend 310设备ID，请确保device_id不超过您设备上的Ascend 310 id号 |
| used_cam_num    | 指定从cam list中的前used_cam_num个视频流拉取视频流,本程序最多支持12路视频流输入 |
| cam#（#为编号） | 每个视频流的地址，只支持H.264或H.265视频流                   |

#### 3.模型转换

本程序的FaceDetectionEngine 、FaceLandmarkEngine 和FaceFeatureEngine分别依赖三个不同的DaVinci模型，请于depository/models目录下的三个子文件夹中分别运行转换脚本进行模型转换。下表列出了这三个模型的说明。

| 模型文件名称                                 | 放置目录                     | 主要功能       |
| -------------------------------------------- | ---------------------------- | -------------- |
| resnet18.pb                                  | depository/models/resnet18   | 人脸特征提取   |
| resnet10.pb                                  | depository/models/resnet10   | 人脸关键点检测 |
| yolov3tiny_b4.prototxt/yolov3tiny.caffemodel | depository/models/yolov3tiny | 人脸检测       |

需要特别说明的是，resnet18.pb和resnet10.pb均需要转换为batchsize分别等于1、4、8的3个不同DaVinci模型。因此本程序共有7个DaVinci模型文件。

#### 4.编译

(1) 修改build/build_local.sh：
	将export DDK_HOME=/home/xxx/DDK/ 修改为实际DDK目录；
    将FFMPEG_PATH="/home/xxx/ffmpeg/" 修改为实际FFmpeg目录，并确保该目录下包含lib和include两个用于存放so和头文件的文件夹。
(2) 于build目录下运行build_local.sh，编译后会生成bin文件夹，编译脚本会将depository/下的数据全部拷贝到bin/data目录下。

```
cd build 
./build_local.sh
```

#### 5.本地开启RTSP媒体服务器

请使用具有RTSP媒体服务功能的第三方软件以将本地H.264或H.265的视频进行推流。

#### 6.开启流媒体转发服务

本demo已提供基于RTSP的可视化视频流输出。若用户需要观看可视化结果，需在运行人脸识别demo前开启流媒体转发服务。一种较为便捷的方法是在桌面端（ip地址为graph.config中rtsp_link中的地址）安装并运行具有流媒体转发功能的第三方软件。

注意，若用户不需要或不具备上述条件，必须在运行人脸识别demo时显式关闭视频流输出功能，关闭方式请看下一节说明。

#### 7.运行

于bin目录下运行facedemo_main。运行前请确保当前用户已加入HwHiAiUser用户组并已切换到该用户组。

```
cd bin
./facedemo_main
```

运行可使用的参数如下表所示：

| 选项   | 意义                                                         | 默认值              |
| ------ | ------------------------------------------------------------ | ------------------- |
| -graph | graph.config路径，字符串                                     | ./data/graph.config |
| -setup | setup.config路径，字符串                                     | ./data/setup.config |
| -disp  | 是否将人脸识别结果通过视频流输出，整数。大于等于1为是，其他为否。 | 1                   |

#### 8.可视化

确保disp大于等于1，且已在桌面端开启流媒体转发服务。在桌面端安装并使用支持RTSP流媒体播放的播放器打开graph.config中的rtsp_link。

