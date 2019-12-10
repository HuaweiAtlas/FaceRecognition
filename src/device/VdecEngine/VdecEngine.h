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

#ifndef ATLASFACEDEMO_VDECENGINE_H
#define ATLASFACEDEMO_VDECENGINE_H
#include <hiaiengine/ai_model_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "dvpp/Vpc.h"
#include "dvpp/dvpp_config.h"
#include "dvpp/idvppapi.h"
#include "dvpp/Vdec.h"
#include "hiaiengine/api.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/engine.h"
#include "hiaiengine/multitype_queue.h"
#include "stream_data.h"

#define INPUT_SIZE 1
#define OUTPUT_SIZE 2

class DecodeH26xInfo : public HIAI_DATA_SP {
public:
    void* vdecEnginePtr;
    StreamInfo info;
    timespec time;
    ~DecodeH26xInfo()
    {}
};

// define struct that used to send frame and frame ID from get_vdec_result() to
// host process
typedef struct {
    StreamInfo info;
    std::vector<ImageInfo> cropImageList;
} FRAME_BUFFER;

namespace hiai {

class VdecEngine : public Engine {
public:
    VdecEngine() : input_que_(INPUT_SIZE){};
    ~VdecEngine();
    static void VdecResultCallback(FRAME*, void*);
    static void VdecErrorCallback(VDECERR* vdecErr);
    HIAI_StatusT Init(const AIConfig&, const std::vector<AIModelDescription>&);

    HIAI_StatusT DecodeH26xVideo(std::shared_ptr<StreamRawData> streamRawData);
    HIAI_StatusT VdecImageResize(FRAME* frame, std::vector<ImageInfo>& cropImageList);

    /**
     * @ingroup hiaiengine
     */
    HIAI_DEFINE_PROCESS(INPUT_SIZE, OUTPUT_SIZE);

private:
    // common parameters
    hiai::MultiTypeQueue input_que_;

    /* define DVPP object */
    IDVPPAPI* piDvppApiVdec = NULL;
    IDVPPAPI* piDvppApiVpc = NULL;

    /* transfer data between the callback functiong and main process */
    queue<FRAME_BUFFER> frameQueue;

    int vdecCounter = 0;
    int vdecFailedCounter = 0;

    // vpc output
    uint8_t* output1Buffer = nullptr;
};
}  // namespace hiai
#endif
