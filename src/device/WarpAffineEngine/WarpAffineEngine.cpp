/**
 * ============================================================================
 *
 * Copyright (C) 2019, Huawei Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#include "WarpAffineEngine.h"
#include "engine_tools.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <fstream>
#include <hiaiengine/api.h>
#include <hiaiengine/data_type.h>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#define LANDMARK_NUM 10
#define AFFINE_LEN (6)
/**
 * @brief: init, inherited from hiaiengine lib
 */
HIAI_StatusT WarpAffineEngine::Init(
    const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    HIAI_ENGINE_LOG(this, HIAI_OK, "[WarpAffineEngine] start init");
    return HIAI_OK;
}

/*
 *@brief: calculate the affine matrix with the given five couple key points
 *@in: keyPointBefore: matirx containing the five key points in the image before warp
 *affine
 *@in: keyPointAfter: matirx containing the five key points in the image after warp
 *affine
 *@in: affineMatrix: the 2x3 affine matrix data, arranged in the order of
 *X00,X01,X02,X10,X11,X12
 *@return: calculate state
 */
int CalAffineMatrix(float* keyPointBefore, int keyPointBeforeSize, float keyPointAfter[], int keyPointAfterSize,
    float* affineMatrix, int affineMatrixSize)
{
    if (keyPointBeforeSize != keyPointAfterSize) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "The size of keypoint must be the same");
        return -1;
    }
    int kPBefore[AFFINE_LEN];
    int kPAfter[AFFINE_LEN];
    float deno0;
    int flag = 0;
    for (uint32_t idxKp0 = 0; idxKp0 <= 2; idxKp0++) {
        for (uint32_t idxKp1 = idxKp0 + 1; idxKp1 <= 3; idxKp1++) {
            for (uint32_t idxKp2 = idxKp1 + 1; idxKp2 <= 4; idxKp2++) {
                // chose 3 couple of points to cal
                kPBefore[0] = keyPointBefore[idxKp0 * 2];
                kPBefore[1] = keyPointBefore[idxKp0 * 2 + 1];
                kPBefore[2] = keyPointBefore[idxKp1 * 2];
                kPBefore[3] = keyPointBefore[idxKp1 * 2 + 1];
                kPBefore[4] = keyPointBefore[idxKp2 * 2];
                kPBefore[5] = keyPointBefore[idxKp2 * 2 + 1];

                kPAfter[0] = keyPointAfter[idxKp0 * 2];
                kPAfter[1] = keyPointAfter[idxKp0 * 2 + 1];
                kPAfter[2] = keyPointAfter[idxKp1 * 2];
                kPAfter[3] = keyPointAfter[idxKp1 * 2 + 1];
                kPAfter[4] = keyPointAfter[idxKp2 * 2];
                kPAfter[5] = keyPointAfter[idxKp2 * 2 + 1];
                // cal the denominator which shared by the first row of affineMatrix
                deno0 = (kPBefore[4] - kPBefore[0]) * (kPBefore[3] - kPBefore[1]) -
                        (kPBefore[2] - kPBefore[0]) * (kPBefore[5] - kPBefore[1]);
                if (deno0 != 0) {
                    flag = 1;
                    break;
                }
                // the current chosen 3 couple of key points are collinear,resort to the next
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                    "a0=%d,a1=%d,a2=%d,a3=%d,a4=%d,a5=%d",
                    kPBefore[0],
                    kPBefore[1],
                    kPBefore[2],
                    kPBefore[3],
                    kPBefore[4],
                    kPBefore[5]);
            }
        }
    }

    if (flag == 0) {
        return -1;
    }

    // cal the denominator which shared by the second row of affineMatrix
    float deno1 = -deno0;

    // cal the first row of affineMatrix
    affineMatrix[0] = ((kPAfter[4] - kPAfter[0]) * (kPBefore[3] - kPBefore[1]) -
                          (kPAfter[2] - kPAfter[0]) * (kPBefore[5] - kPBefore[1])) *
                      1.0 / deno0;

    affineMatrix[1] = ((kPAfter[4] - kPAfter[0]) * (kPBefore[2] - kPBefore[0]) -
                          (kPAfter[2] - kPAfter[0]) * (kPBefore[4] - kPBefore[0])) *
                      1.0 / deno1;

    affineMatrix[2] = kPAfter[0] - affineMatrix[0] * kPBefore[0] - affineMatrix[1] * kPBefore[1];

    // cal the second row of affineMatrix
    affineMatrix[3] = ((kPAfter[5] - kPAfter[1]) * (kPBefore[3] - kPBefore[1]) -
                          (kPAfter[3] - kPAfter[1]) * (kPBefore[5] - kPBefore[1])) *
                      1.0 / deno0;

    affineMatrix[4] = ((kPAfter[5] - kPAfter[1]) * (kPBefore[2] - kPBefore[0]) -
                          (kPAfter[3] - kPAfter[1]) * (kPBefore[4] - kPBefore[0])) *
                      1.0 / deno1;

    affineMatrix[5] = kPAfter[1] - affineMatrix[3] * kPBefore[0] - affineMatrix[4] * kPBefore[1];

    return 0;
}

/**
 * @brief do warpaffine
 * @[in]: face: faceobject
 * @return: HIAI_StatusT
 */
HIAI_StatusT WarpAffineEngine::ApplyWarpAffine(FaceObject* face)
{
    int width = face->imgCroped.widthAligned;
    int height = face->imgCroped.heightAligned;
    if (width * height * 3 / 2 != face->imgCroped.buf.len_of_byte) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[WarpAffineEngine] nv12 data size is wrong!");
        return HIAI_ERROR;
    }

    /* do nv12 to rgb888 convert */
    cv::Mat srcNV12Mat(height * 3 / 2, width, CV_8UC1, face->imgCroped.buf.data.get());
    cv::Mat dstRGB888(height, width, CV_8UC3);
    cvtColor(srcNV12Mat, dstRGB888, cv::COLOR_YUV2RGB_NV12);

    /* calculate the affine matrix */
    float* affineMatrix = new float[AFFINE_LEN];  // 6 values to cal, x00,x01,x02,x10,x11,x12
    float* kPBefore = new float[LANDMARK_NUM];    // five key points before warp affine, arranged by x0,y0,x1,y1...

    // five standard key points after warp affine ,arranged by x0,y0,x1,y1..., scale 112*112
    float kPAfter[LANDMARK_NUM] = {
        30.2946, 51.6963, 65.5318, 51.5014, 48.0252, 71.7366, 33.5493, 92.3655, 62.7299, 92.2041};

    // map the standard key points into  the image size
    // according to insightface open source //
    // https://github.com/deepinsight/insightface/blob/master/src/align/align_facescrub.py
    for (int i = 0; i < LANDMARK_NUM / 2; i++) {
        kPAfter[i * 2] = face->imgCroped.width / 112.0 * kPAfter[i * 2] + 8.0;
        kPAfter[i * 2 + 1] = face->imgCroped.height / 112.0 * kPAfter[i * 2 + 1];
    }
    memcpy_s(kPBefore, LANDMARK_NUM * sizeof(float), face->landmarks.data.get(), face->landmarks.len_of_byte);

    // map the infered key points into  the image size
    for (int i = 0; i < 5; i++) {
        kPBefore[i * 2] = face->imgCroped.width * (kPBefore[i * 2]);
        kPBefore[i * 2 + 1] = face->imgCroped.height * (kPBefore[i * 2 + 1]);
    }

    if (CalAffineMatrix(kPBefore, LANDMARK_NUM, kPAfter, LANDMARK_NUM, affineMatrix, AFFINE_LEN) == -1) {
        HIAI_ENGINE_LOG(
            HIAI_IDE_ERROR, "[WarpAffineEngine] can not find 3 couple of key points which are not collinear!");
        delete[] affineMatrix;
        delete[] kPBefore;
        affineMatrix = NULL;
        kPBefore = NULL;

        // when affinematrix cannot be calculated, do nothing
        uint8_t* tmpAffine = new uint8_t[width * height * 3];
        face->imgAffine.buf.data.reset(tmpAffine);
        memcpy_s(face->imgAffine.buf.data.get(), width * height * 3, dstRGB888.data, width * height * 3);
        face->imgAffine.buf.len_of_byte = width * height * 3;
        face->imgCroped.buf.data.reset();
        face->imgCroped.buf.len_of_byte = 0;
        face->landmarks.data.reset();
        face->landmarks.len_of_byte = 0;
        return HIAI_ERROR;
    }

    // define the affine matrix, the matrix require 6 number */
    cv::Mat warpMat = (cv::Mat_<double>(2, 3) << affineMatrix[0],
        affineMatrix[1],
        affineMatrix[2],
        affineMatrix[3],
        affineMatrix[4],
        affineMatrix[5]);
    cv::Mat warpDst = cv::Mat::zeros(height, width, CV_8UC3);
    warpAffine(dstRGB888, warpDst, warpMat, warpDst.size());

    /* use new to allocate memeory */
    std::shared_ptr<u_int8_t> tmp(new uint8_t[width * height * 3]);
    face->imgAffine.buf.data = tmp;
    memcpy_s(face->imgAffine.buf.data.get(), width * height * 3, warpDst.data, width * height * 3);
    face->imgAffine.buf.len_of_byte = width * height * 3;

    // release data which will never be used
    face->imgCroped.buf.data.reset();
    face->imgCroped.buf.len_of_byte = 0;
    face->landmarks.data.reset();
    face->landmarks.len_of_byte = 0;

    delete[] affineMatrix;
    delete[] kPBefore;
    kPBefore = NULL;
    affineMatrix = NULL;
    return HIAI_OK;
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("WarpAffineEngine", WarpAffineEngine, WARP_AFFINE_INPUT_SIZE)
{
    if (arg0 == NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[WarpAffineEngine] invalid input");
        return HIAI_ERROR;
    }

    std::shared_ptr<DeviceStreamData> dataInput = std::static_pointer_cast<DeviceStreamData>(arg0);
    HIAI_StatusT ret = HIAI_OK;
    for (int i = 0; i < dataInput->face.size(); i++) {
        ret = ApplyWarpAffine(&(dataInput->face[i]));
        if (HIAI_OK != ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[WarpAffineEngine] applywarpaffine error, skip warpaffine!");
        }
    }

    // send data to FeatureExtract engine according to the face num, this trick
    // may reduce inference time to some degree
    if (dataInput->face.size() <= 1) {  // batchsize is 1
        ret = SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
    } else if (dataInput->face.size() <= 4) {  // batchsize is 4
        ret = SendData(1, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
    } else {  // batchsize is 8
        ret = SendData(2, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
    }

    if (HIAI_OK != ret) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[WarpAffineEngine] send output error!");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}