EN|[CN](README.zh.md)

## Face recognition demo

This repository contains a face recognition demo running on the Atlas 300 product. The steam data is pulled using FFmpeg, then the faces in the stream are detected with extracted features. The steam data as well as the detection results will be pushed using FFmpeg to host to display. Specifically, the demo plays with the following features:

(1) The RTSP video stream is sent to device side from host side. The stream is decoded through hardware decoding module. The decoded stream will be sent back to the host side.

(2)The face boxes are detected using object detection model.

(3)A face landmarks model is used to extract the faces' landmarks, with which  to correct the face attitude through affine transformation.

(4)The  representation features of faces are extracted using image classification model.

(5)A face tracking algorithm is adopted to significantly reduce the end-to-end calculation workload by reducing the calculation workload of (3) and (4).

(6) The output steam data is synchronized with detection results and pushed to display.

[TOC]

### Supported Products

Atlas 300 (Model 3000), Atlas 300 (Model 3010)

### Supported Version

1.31.T15.B150 or 1.3.2.B893

It can be obtained by executing the following command

```
npu-smi info
```

### Compatible Operating System

Ubuntu 16.04 or CentOS 7.4

### Directory Structure

The directory structure of this demo is shown as follows：

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

All source files are stored in the src directory. All dependent projects are stored in the depository directory, and the compilation scripts are stored in the build directory.

### Dependence

(1) FFmpeg

The stream pull  and push function is based on FFmpeg library.  Compile and install the source code by referring to the FFmpeg open source project. Please ensure the installed FFmpeg has the following directory format:

```
├── FFmpeg
|   ├── include
|   ├── lib
```

### How to run

#### 1. Set up face database

Users need to create a new face database named featurelib.bin in the /depository/featurelib directory after creating directory featurelib in depository. The Featurelib.bin is a binary file, content of which is the continuous storage of all the personal face features (512 dimensions with float32 type). The structure is as follows:

```
feature1feature2feature3...featuren
```

Users can extract face feature vector through open source face feature extraction model and write it into binary file, or use Python script to randomly generate binary file with specific byte length (face number x 512 x 4).

Note: The face comparison function in the face recognition demo is used only for debugging.

#### 2. Modify the configuration file 

There are two configuration files in this project, which are located in the depository/config. One is graph.config, which is the graph configuration file used by the Matrix engine, and the other is setup.config, which is the configuration file of the input video stream of the demo. The following tables describe the main configuration items in the two configuration files.

​                                                       Table 1：Configuration items in the graph.config file

| Configuration Item     | Owning Engine                                            | Description                                                  |
| ---------------------- | -------------------------------------------------------- | ------------------------------------------------------------ |
| rtsp_link              | StreamDataOutputEngine                                   | The address of output stream using RTSP protocol. The format is rtsp://aa.bbb.ccc.ddd/live.sdp，in which aa.bbb.ccc.ddd is the IP adress |
| feature_lib_path       | StreamDataOutputEngine                                   | The path to store the binary file in the feature database    |
| feature_len            | StreamDataOutputEngine                                   | The element number of each feature                           |
| feature_num            | StreamDataOutputEngine                                   | The number of features in the feature database               |
| batch_size             | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | The input batch size of the model used by the engine. This parameter is used to verify the actual value of the model. |
| input_channel          | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | The input channel number of the model used by the engine. This parameter is used to verify the actual value of the model. |
| input_width            | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | The input width of the model used by the engine. This parameter is used to verify the actual value of the model. |
| input_height           | FaceDetectionEngine FaceLandmarkEngine FaceFeatureEngine | The input height of the model used by the engine. This parameter is used to verify the actual value of the model. |
| max_face_num_per_frame | FaceDetectionEngine FaceLandmarkEngine                   | The maximum number of reserved faces per frame. The values of max_face_num_per_frame of the FaceDetectionEngine and FaceLandmarkEngine must be the same. |
| adjacencyThreshold     | SORTEngine                                               | The threshold of Hungarian matching algorithm. The recommended value is about 0.2 |
| trackThreshold         | SORTEngine                                               | The threshold of tracking strategy. The recommended value is 0.6~0.8 |

​                                                                 Table 2：Configuration Descriptions of setup.config 

| Item                  | Descriptions                                                 |
| --------------------- | ------------------------------------------------------------ |
| device_id             | Specify the ID of the Ascend 310 device to configure. Ensure that the value of device_id does not exceed the value of Ascend 310 ID on your device. |
| used_cam_num          | Specify that video streams are pulled from the cam list from the first to the used_cam_num. This program supports a maximum of 12 video streams. |
| cam# (# is the index) | The stream address. Only H.264 or H.265 video streams are supported. |

#### 3. Model conversion

The FaceDetectionEngine ,FaceLandmarkEngine and FaceFeatureEngine depend on different DaVinci models respectively. Use the conversion scripts to in the three subfolders under the depository/models directory to convert the models into DaVinci models. The following table describes the three models.

| Model File Name                              | Directory                    | Function                  |
| -------------------------------------------- | ---------------------------- | ------------------------- |
| resnet18.pb                                  | depository/models/resnet18   | Facial feature extraction |
| resnet10.pb                                  | depository/models/resnet10   | Face landmark extraction  |
| yolov3tiny_b4.prototxt/yolov3tiny.caffemodel | depository/models/yolov3tiny | Face detection            |

Note that: both resnet18.pb and resnet10.pb model need to be converted into three different DaVinci models whose batch size is 1, 4, and 8 respectively. Therefore, this program has seven DaVinci model files.

#### 4. Compilation

(1) Modify the build/build_local.sh file:

change the line "export DDK_HOME=/home/xxx/DDK/" to the actual DDK path in your environment.
change the line "FFMPEG_PATH="/home/xxx/ffmpeg/" to the exact FFmpeg path in your environment and ensure that the directory contains lib and include subdirectories for storing so and header files.

(2) run the build script in directory build directory. After compilation, a bin directory is generated, and the compilation script copies all data in the depository/ directory to the bin/data directory.

```
cd build 
./build_local.sh
```

#### 5. Enabling RTSP media server locally

Please use third-party software with RTSP media server function to stream the local H.264 or h.265 video.

#### 6. Enabling Streaming Media Forwarding Service

This demo provides visual video stream output based on the RTSP. To view the visualization result, you need to enable the streaming media forwarding service before running the facial recognition demo. A convenient method is to install and run the third-party software with the streaming media forwarding function on the desktop (the IP address is the address of rtsp_link in the graph.config file).

Note: If the preceding conditions are not required or not met, you must explicitly disable the video stream output function when running the facial recognition demo. For details about how to disable the video stream output function, see the next section.

#### 7. run

Run facedemo_main in the bin directory. Please make sure that the current user has joined the HwHiAiUser user group and switched to it.

```
cd bin
./facedemo_main
```

The following table lists the parameters that can be used during the operation.

| Parameter | meaning                                                      | Default value       |
| --------- | ------------------------------------------------------------ | ------------------- |
| -graph    | graph.config path, character string                          | ./data/graph.config |
| -setup    | setup.config path, character string                          | ./data/setup.config |
| -disp     | Determine whether to output the facial recognition result by using video streams. The value is an integer. If the value is greater than or equal to 1, the result is Yes. Otherwise, the result is No. | 1                   |

#### 8. Visualization 

Ensure that the value of disp is greater than or equal to 1 and the streaming media forwarding service is enabled on the desktop. Install and use a player that supports RTSP streaming media playback on the desktop to open rtsp_link in the graph.config file.

