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

#ifndef ATLASFACEDEMO_SORTENGINE_H
#define ATLASFACEDEMO_SORTENGINE_H
#include "hiaiengine/engine.h"
#include "Hungarian.h"
#include "KalmanTracker.h"
#include "stream_data.h"
#include <vector>

#define SORT_INPUT_SIZE 1
#define SORT_OUTPUT_SIZE 2

class SORTEngine : public Engine {
public:
    SORTEngine()
    {}

    ~SORTEngine()
    {}
    /**
     * @ingroup SORTEngine
     * @brief SORTEngine 初始化函数
     * @param [in]：config, 配置参数
     * @param [in]: model_desc, 模型描述
     * @param [out]: HIAI_StatusT
     */
    HIAI_StatusT Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc);

    /**
     * @ingroup hiaiengine
     * @brief HIAI_DEFINE_PROCESS : 重载Engine Process处理逻辑
     * @[in]: 定义一个输入端口，一个输出端口
     */
    HIAI_DEFINE_PROCESS(SORT_INPUT_SIZE, SORT_OUTPUT_SIZE)
private:
    std::vector<DetectInfo> keyFrameDetectResult;
    uint64_t keyFrameId = 0;
    std::vector<int> keepTrackerIdx;
    double continuityThreshold = 1;
    double adjacencyThreshold = 0.15;
    double trackThreshold = 0.6;
    const int HIST_FEATURE_LEN = 256;
    std::vector<KalmanTracker> faceTrackers;

    std::vector<cv::Rect_<float>> bboxPred;
    std::vector<std::vector<int>> disMatrix;
    std::vector<std::shared_ptr<double>> keyHistFeatures;
    std::vector<cv::Point> matchedPairs;
    HungarianHandle hungarianHandleObj;
    unsigned int trkNum = 0;
    unsigned int detNum = 0;
    int continuousTracked = 0;
};

#endif
