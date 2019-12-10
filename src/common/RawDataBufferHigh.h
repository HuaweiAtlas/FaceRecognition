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
#ifndef ATLASFACEDEMO_RAWDATABUFFER_H
#define ATLASFACEDEMO_RAWDATABUFFER_H

#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include <memory>

struct RawDataBufferHigh {
    std::shared_ptr<uint8_t> data;
    uint32_t len_of_byte;  // buffer size
    uint64_t frameId;
    uint32_t channelID;
};

template <class Archive>
void serialize(Archive& ar, RawDataBufferHigh& data)
{
    ar(data.len_of_byte);
    ar(data.frameId);
    ar(data.channelID);
    if (data.len_of_byte > 0 && data.data.get() == nullptr) {
        HIAI_SHARED_PTR_RESET(
            data.data, new (std::nothrow) uint8_t[data.len_of_byte], "serialize RawDataBuffer reset fail");
    }
    ar(cereal::binary_data(data.data.get(), data.len_of_byte * sizeof(uint8_t)));
}

inline void GetTransSearPtr(void* dataPtr, std::string& structStr, uint8_t*& buffer, uint32_t& bufferSize)
{
    RawDataBufferHigh* engineTrans = (RawDataBufferHigh*)dataPtr;
    structStr = std::string((const char*)dataPtr, sizeof(RawDataBufferHigh));
    buffer = (uint8_t*)engineTrans->data.get();
    bufferSize = engineTrans->len_of_byte;
}

inline std::shared_ptr<void> GetTransDearPtr(
    const char* ctrlPtr, const uint32_t& ctrlLen, const unsigned char* dataPtr, const uint32_t& dataLen)
{
    std::shared_ptr<RawDataBufferHigh> engineTrans = std::make_shared<RawDataBufferHigh>();
    engineTrans->len_of_byte = ((RawDataBufferHigh*)ctrlPtr)->len_of_byte;
    engineTrans->frameId = ((RawDataBufferHigh*)ctrlPtr)->frameId;
    engineTrans->channelID = ((RawDataBufferHigh*)ctrlPtr)->channelID;
    engineTrans->data.reset((uint8_t*)dataPtr, hiai::Graph::ReleaseDataBuffer);
    return std::static_pointer_cast<void>(engineTrans);
}

#endif