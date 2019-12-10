/**
 * ============================================================================
 *
 * Copyright (C) Huawei Technologies Co., Ltd. 2019. All Rights Reserved.
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

#include "SORTEngine.h"
#include "engine_tools.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <hiaiengine/api.h>
#include <hiaiengine/data_type.h>
#include <iostream>
#include <map>
#include <math.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

HIAI_REGISTER_SERIALIZE_FUNC("RawDataBufferHigh", RawDataBufferHigh, GetTransSearPtr, GetTransDearPtr);
HIAI_REGISTER_DATA_TYPE("DeviceStreamData", DeviceStreamData);

/**
 * @brief: init, inherited from hiaiengine lib
 */
HIAI_StatusT SORTEngine::Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    HIAI_ENGINE_LOG(this, HIAI_OK, "[SORTEngine] start init");
    std::map<string, string> keyValueConfig = KvMap(config);

    // parse parameters
    // continuityThreshold
    if (CheckEmpty(keyValueConfig["continuityThreshold"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    if (!(std::stringstream(keyValueConfig["continuityThreshold"]) >> continuityThreshold)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "continuityThreshold is not digit,please check!");
        return HIAI_ERROR;
    }
    if (continuityThreshold < 1) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] continuityThreshold must be larger than 1");
        return HIAI_ERROR;
    }

    // adjacencyThreshold
    if (CheckEmpty(keyValueConfig["adjacencyThreshold"]) != HIAI_OK)
        return HIAI_ERROR;
    if (!(std::stringstream(keyValueConfig["adjacencyThreshold"]) >> adjacencyThreshold)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "adjacencyThreshold is not digit,please check!");
        return HIAI_ERROR;
    }
    if ((adjacencyThreshold >= 1) || (adjacencyThreshold <= 0)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] adjacencyThreshold must be in internel (0,1)");
        return HIAI_ERROR;
    }

    // trackThreshold
    if (CheckEmpty(keyValueConfig["trackThreshold"]) != HIAI_OK)
        return HIAI_ERROR;
    if (!(std::stringstream(keyValueConfig["trackThreshold"]) >> trackThreshold)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "trackThreshold is not digit,please check!");
        return HIAI_ERROR;
    }
    if ((trackThreshold >= 1) || (trackThreshold <= 0)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] trackThreshold must be in internel (0,1)");
        return HIAI_ERROR;
    }

    HIAI_ENGINE_LOG(this, HIAI_OK, "[SORTEngine] init success");
    return HIAI_OK;
}

/**
 * @brief Computes IOU between two bounding boxes
 * @[in]: bbox_1: the first bounding boxes, bbox_2: the second bounding boxes
 * $return: IOU value
 */
double CalIOU(cv::Rect_<float> bbox1, cv::Rect_<float> bbox2)
{
    float intersectionArea = (bbox1 & bbox2).area();
    float unionArea = bbox1.area() + bbox2.area() - intersectionArea;
    if (unionArea < DBL_EPSILON) {
        return 0;
    }
    return (double)(intersectionArea / unionArea);
}

/**
 * @brief: Computes the 256-length histgram feature for a bounding box
 * @[in]: ltx: x of left top, lty: y of left top, rbx: x of right bottom, rby: y of right bottom, imgOrigin: the
 * original image, histFeature: histfeature data
 */
void CalHistFeature(int ltx, int lty, int rbx, int rby, ImageInfo imgOrigin, double* histFeature)
{
    uint8_t* buf = imgOrigin.buf.data.get();
    int tmpy;
    for (int y = lty; y <= rby; y++) {
        tmpy = y * imgOrigin.widthAligned;
        for (int x = ltx; x <= rbx; x++) {
            ++(*(histFeature + *(buf + tmpy + x)));
        }
    }
}

/**
 * @brief: Computes the cosine distance between two hist vector,larger cosine distance indicates they are more similar
 * @[in]: histFeature1: the first hist vector,histFeature2: the second hist vector
 * @return: cosine distance
 */
double CalHistCosine(double* histFeature1, double* histFeature2, int featureLen)
{
    if (featureLen <= 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] hist featureLen is invalid.");
        return HIAI_ERROR;
    }
    cv::Mat histFeature1Mat(1, featureLen, CV_64F, histFeature1);
    cv::Mat histFeature2Mat(1, featureLen, CV_64F, histFeature2);
    double histFeature1Norm = cv::norm(histFeature1Mat);
    double histFeature2Norm = cv::norm(histFeature2Mat);
    double dotSum = histFeature1Mat.dot(histFeature2Mat);
    if (histFeature1Norm != 0 && histFeature2Norm != 0) {
        return dotSum / (histFeature1Norm * histFeature2Norm);
    } else {
        return 0;
    }
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("SORTEngine", SORTEngine, SORT_INPUT_SIZE)
{
    /*continuousTracked is the threashold that control how the tracker is
      sensitive to target mis-tracked,
      for example, if continuousTracked=1 ,then only with 1 continuous frames
      mis-tracked, the kalman filter will be
      reset using the incoming frame,which will be sent to landmark and
      warpaffine
      engines, otherwise SORT will try to
      do tracking
     */
    if (arg0 == NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] invalid input");
        return HIAI_ERROR;
    }

    if (continuousTracked <= 0) {
        std::shared_ptr<DeviceStreamData> keyFrameData = std::static_pointer_cast<DeviceStreamData>(arg0);

        // if the face number is 0, this is fake key frame
        if (keyFrameData->detectResult.size() == 0) {
            keyFrameData->isKeyFrame == 0;
            keyFrameData->trkFrameID = -1;
            keyFrameData->isTracked == 0;
            if (SendData(1, "DeviceStreamData", std::static_pointer_cast<void>(keyFrameData)) != HIAI_OK) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] send data error");
                return HIAI_ERROR;
            }
            return HIAI_OK;
        }
        keyFrameData->isKeyFrame = 1;
        keyFrameDetectResult.clear();
        keyFrameDetectResult = keyFrameData->detectResult;
        keyFrameId = keyFrameData->info.frameId;

        // reset KalmanTracker
        faceTrackers.clear();
        keepTrackerIdx.clear();
        keyHistFeatures.clear();

        // init the kalman faceTrackers
        for (unsigned int i = 0; i < keyFrameDetectResult.size(); i++) {
            cv::Rect_<float> rect = cv::Rect_<float>(cv::Point_<float>(keyFrameDetectResult[i].location.anchor_lt.x,
                                                         keyFrameDetectResult[i].location.anchor_lt.y),
                cv::Point_<float>(
                    keyFrameDetectResult[i].location.anchor_rb.x, keyFrameDetectResult[i].location.anchor_rb.y));
            KalmanTracker trk = KalmanTracker(rect);
            faceTrackers.push_back(trk);
            keepTrackerIdx.push_back(i);  // init the idx to keep

            std::shared_ptr<double> keyHistFeature(new double[HIST_FEATURE_LEN]());
            CalHistFeature(keyFrameDetectResult[i].location.anchor_lt.x,
                keyFrameDetectResult[i].location.anchor_lt.y,
                keyFrameDetectResult[i].location.anchor_rb.x,
                keyFrameDetectResult[i].location.anchor_rb.y,
                keyFrameData->imgOrigin,
                keyHistFeature.get());
            keyHistFeatures.push_back(keyHistFeature);
        }
        continuousTracked = continuityThreshold;

        if (SendData(1, "DeviceStreamData", std::static_pointer_cast<void>(keyFrameData)) != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] send data error");
        }
        return HIAI_OK;
    }
    std::shared_ptr<DeviceStreamData> detFrameData = std::static_pointer_cast<DeviceStreamData>(arg0);
    // if the face number is 0, this is fake key frame
    if (detFrameData->detectResult.size() == 0) {
        detFrameData->isKeyFrame == 0;
        detFrameData->trkFrameID = -1;
        detFrameData->isTracked == 0;
        if (SendData(1, "DeviceStreamData", std::static_pointer_cast<void>(detFrameData)) != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] send data error");
            return HIAI_ERROR;
        }
        return HIAI_OK;
    }

    // do precict
    bboxPred.clear();
    auto it = faceTrackers.begin();
    while (it != faceTrackers.end()) {
        cv::Rect_<float> pBox = (*it).Predict();
        if (pBox.x >= 0 && pBox.y >= 0) {
            bboxPred.push_back(pBox);
            it++;
        } else {
            keepTrackerIdx.erase(keepTrackerIdx.begin() + (it - faceTrackers.begin()));  // delete the corresponding idx
            keyHistFeatures.erase(keyHistFeatures.begin() + (it - faceTrackers.begin()));
            it = faceTrackers.erase(it);
        }
    }

    trkNum = bboxPred.size();
    detNum = detFrameData->detectResult.size();
    disMatrix.clear();
    disMatrix.resize(trkNum, vector<int>(detNum, 0));
    // use iou as distance metic
    double histDistance = 0;
    double iouDistance = 0;
    const int COEFFICIENT_GAIN = 1000;
    for (unsigned int j = 0; j < detNum; j++) {
        double* detFeature = new double[HIST_FEATURE_LEN]();
        int ltx = detFrameData->detectResult[j].location.anchor_lt.x;
        int lty = detFrameData->detectResult[j].location.anchor_lt.y;
        int rbx = detFrameData->detectResult[j].location.anchor_rb.x;
        int rby = detFrameData->detectResult[j].location.anchor_rb.y;
        CalHistFeature(ltx, lty, rbx, rby, detFrameData->imgOrigin, detFeature);
        for (unsigned int i = 0; i < trkNum; i++) {
            histDistance = CalHistCosine(keyHistFeatures[i].get(), detFeature, HIST_FEATURE_LEN);
            // use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
            iouDistance =
                CalIOU(bboxPred[i], cv::Rect_<float>(cv::Point_<float>(ltx, lty), cv::Point_<float>(rbx, rby)));
            disMatrix[i][j] = (0.5 * iouDistance + 0.5 * histDistance) *
                              COEFFICIENT_GAIN;  // the hungarian solver only accept int type
        }
        delete[] detFeature;
        detFeature = NULL;
    }

    // solve the assignment problem using hungarian
    HungarianSolve(&hungarianHandleObj, disMatrix, trkNum, detNum);

    // filter out matched with low IOU
    matchedPairs.clear();
    for (unsigned int i = 0; i < trkNum; ++i) {
        if ((hungarianHandleObj.resX[i] != -1) &&
            (disMatrix[i][hungarianHandleObj.resX[i]] >= adjacencyThreshold * COEFFICIENT_GAIN)) {
            matchedPairs.push_back(cv::Point(i, hungarianHandleObj.resX[i]));
        }
    }

    // updating faceTrackers
    int detIdx, trkIdx;
    for (unsigned int i = 0; i < matchedPairs.size(); i++) {
        trkIdx = matchedPairs[i].x;
        detIdx = matchedPairs[i].y;
        faceTrackers[trkIdx].Update(
            cv::Rect_<float>(cv::Point_<float>(detFrameData->detectResult[detIdx].location.anchor_lt.x,
                                 detFrameData->detectResult[detIdx].location.anchor_lt.y),
                cv::Point_<float>(detFrameData->detectResult[detIdx].location.anchor_rb.x,
                    detFrameData->detectResult[detIdx].location.anchor_rb.y)));
    }

    // if more than n% faces of one frame are tracked, then the frame will be sent to host engine directly
    if (matchedPairs.size() * 1.0 / detFrameData->detectResult.size() >= trackThreshold) {
        continuousTracked = continuityThreshold;
        for (unsigned int i = 0; i < matchedPairs.size(); i++) {
            detFrameData->trkIdx.push_back(keepTrackerIdx[matchedPairs[i].x]);
            detFrameData->detIdx.push_back(matchedPairs[i].y);
        }
        detFrameData->isTracked = 1;
        detFrameData->trkFrameID = keyFrameId;
        detFrameData->isKeyFrame = 0;
        detFrameData->imgOrigin.buf.data.reset();
        detFrameData->imgOrigin.buf.len_of_byte = 0;
        if (SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(detFrameData)) != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] send data error");
            return HIAI_ERROR;
        }

    } else {
        // if matchedPairs is not 0, we split the detect results into matched and unmatched, send the matched to host
        // and send the unmatched to facelandmark engine
        if (matchedPairs.size() > 0) {
            std::shared_ptr<DeviceStreamData> detFrameDataSplitHost = std::make_shared<DeviceStreamData>();
            detFrameDataSplitHost->isTracked = 1;
            detFrameDataSplitHost->trkFrameID = keyFrameId;
            detFrameDataSplitHost->isKeyFrame = 0;
            detFrameDataSplitHost->imgOrigin.buf.len_of_byte = 0;
            detFrameDataSplitHost->imgOrigin.widthAligned = detFrameData->imgOrigin.widthAligned;
            detFrameDataSplitHost->imgOrigin.heightAligned = detFrameData->imgOrigin.heightAligned;
            detFrameDataSplitHost->detectImg.buf.len_of_byte = 0;
            detFrameDataSplitHost->detectResult = detFrameData->detectResult;
            detFrameDataSplitHost->isSplited = 1;
            detFrameDataSplitHost->info = detFrameData->info;

            for (unsigned int i = 0; i < matchedPairs.size(); i++) {
                detFrameDataSplitHost->trkIdx.push_back(keepTrackerIdx[matchedPairs[i].x]);
                detFrameDataSplitHost->detIdx.push_back(matchedPairs[i].y);
            }
            std::vector<DetectInfo> detectResultSplitDevice;
            for (int i = 0; i < detFrameData->detectResult.size(); i++) {
                if (find(detFrameDataSplitHost->detIdx.begin(), detFrameDataSplitHost->detIdx.end(), i) ==
                    detFrameDataSplitHost->detIdx.end()) {
                    detectResultSplitDevice.push_back(detFrameData->detectResult[i]);
                }
            }

            detFrameData->detectResult = detectResultSplitDevice;  // delete the matched detections

            if (SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(detFrameDataSplitHost)) != HIAI_OK) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] send data error");
                return HIAI_ERROR;
            }
        }
        continuousTracked--;
        detFrameData->isTracked = 0;
        detFrameData->trkFrameID = -1;
        detFrameData->isKeyFrame = 0;
        if (SendData(1, "DeviceStreamData", std::static_pointer_cast<void>(detFrameData)) != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[SORTEngine] send data error");
            return HIAI_ERROR;
        }
    }
    return HIAI_OK;
}
