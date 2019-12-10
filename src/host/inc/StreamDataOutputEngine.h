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
#ifndef ATLASFACEDEMO_STREAMDATAOUTPUT_H
#define ATLASFACEDEMO_STREAMDATAOUTPUT_H
#include "stream_data.h"
#include <deque>
#include "hiaiengine/engine.h"
#include <map>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
#include <libswscale/swscale.h>
}
#include "register.h"

#define SDO_INPUT_SIZE 2
#define SDO_OUTPUT_SIZE 1

class StreamDataOutputEngine : public hiai::Engine {
public:
    HIAI_StatusT Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc);

    ~StreamDataOutputEngine();

    HIAI_DEFINE_PROCESS(SDO_INPUT_SIZE, SDO_OUTPUT_SIZE)
private:
    HIAI_StatusT PrepareToPush(std::shared_ptr<DeviceStreamData> imageInput);
    HIAI_StatusT FramePusher(uint8_t* pYUV);
    HIAI_StatusT SendFrame();
    HIAI_StatusT ParamsCheck(const hiai::AIConfig& config, std::string& link);
    HIAI_StatusT AvInit(std::string& rtspLink);

    std::vector<std::map<uint64_t, std::shared_ptr<RawDataBufferHigh>>> frameDataMap;
    std::vector<std::map<uint64_t, std::shared_ptr<DeviceStreamData>>> structuredInfoMap;
    std::vector<std::deque<cv::Mat>> frameReadyQueue;

    uint8_t* readyFramYuv = NULL;
    AVFormatContext* pFormatCtx = NULL;
    AVCodecContext* pCodecCtx = NULL;
    AVCodec* pEncoder = NULL;
    AVStream* video_steam = NULL;
    AVFrame* frame = NULL;
    AVPacket packet;
    bool isHeaderWrite = false;
    bool isPacketSet = false;
    int iFrame = 0;
    std::map<int, std::shared_ptr<float>> featureLib;
};

#endif