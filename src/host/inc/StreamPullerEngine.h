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

#ifndef ATLASFACEDEMO_STREAMPULLER_H
#define ATLASFACEDEMO_STREAMPULLER_H

#include "stream_data.h"
#include "hiaiengine/engine.h"
#include <atomic>
#include <mutex>
#include <thread>

extern "C" {
#include "libavformat/avformat.h"
}

#define RP_INPUT_SIZE 1
#define RP_OUTPUT_SIZE 1

class StreamPullerEngine : public hiai::Engine {
public:
    HIAI_StatusT Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc);
    HIAI_DEFINE_PROCESS(RP_INPUT_SIZE, RP_OUTPUT_SIZE)
    ~StreamPullerEngine();

private:
    int GetStreamInfo();
    void PullStreamDataLoop();
    void StopStream();
    HIAI_StatusT StartStream(const string& streamName);

    std::shared_ptr<AVFormatContext> pFormatCtx;
    // stream info
    uint64_t blockId = 0;
    uint32_t mWidth = 1280;
    uint32_t mHeight = 720;
    uint32_t channelId = 0;
    uint32_t format = 0;
    int videoIndex = -1;
    std::atomic<int> stop = {0};
    std::thread sendDataRunner;
    RawDataBufferHigh dataBuffer;
    uint64_t curBlockId = 0;
    uint64_t frameNum = 0;
    uint32_t failedCount = 0;
};

#endif