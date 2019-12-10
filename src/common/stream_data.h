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
#ifndef ATLASFACEDEMO_STREAMDATA_H
#define ATLASFACEDEMO_STREAMDATA_H
#include "RawDataBufferHigh.h"
#include "cereal/types/utility.hpp"
#include "dvpp/dvpp_config.h"
#include "hiaiengine/ai_memory.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include <array>
#include <chrono>
#include <memory>
#include <time.h>
#include <unordered_map>
#include <vector>
typedef struct timespec Time;
template <class Archive>
void serialize(Archive& ar, Time& data)
{
    ar(data.tv_sec, data.tv_nsec);
}
using namespace hiai;
using hiai::BatchInfo;
using hiai::ImageData;
using std::unordered_map;
using time_pair = std::pair<Time, Time>;
using std::string;
using time_table = unordered_map<string, time_pair>;
using hiai::Point2D;
using hiai::Rectangle;
using std::array;
using std::shared_ptr;
using std::vector;

struct StreamInfo {
    uint64_t frameId;   /* frame id, in video stream channel */
    uint32_t mode;      /* Operate mode: register, normal, decode H26* */
    uint32_t format;    /* raw data format: jpg, png, h264, h265 */
    uint32_t channelId; /* video stream channel, corresponding to camera */
    uint32_t isEOS;     /* flag of video stream reception */
    uint32_t isDisplayChannel;
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(frameId, mode, format, channelId, isEOS, isDisplayChannel);
    }
};

struct StreamRawData {
    RawDataBufferHigh buf;
    StreamInfo info;
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(info);
    }
};
inline void StreamRawDataSerialize(void* input, std::string& control, std::uint8_t*& data, std::uint32_t& data_length)
{
    auto streamData = static_cast<StreamRawData*>(input);
    control = std::string(static_cast<char*>(input), sizeof(StreamRawData));
    data = streamData->buf.data.get();
    data_length = streamData->buf.len_of_byte;
}
inline std::shared_ptr<void> StreamRawDataDeserialize(const char* control, const std::uint32_t& control_length,
    const std::uint8_t* data, const std::uint32_t& data_length)
{
    auto streamData = std::make_shared<StreamRawData>();
    streamData->info = reinterpret_cast<StreamRawData*>(const_cast<char*>(control))->info;
    streamData->buf.len_of_byte = reinterpret_cast<StreamRawData*>(const_cast<char*>(control))->buf.len_of_byte;

    if (streamData->buf.len_of_byte != 0) {
        streamData->buf.data =
            std::shared_ptr<std::uint8_t>(const_cast<std::uint8_t*>(data), hiai::Graph::ReleaseDataBuffer);
    }
    return std::static_pointer_cast<void>(streamData);
}

struct DetectInfo {
    int32_t classId;
    float confidence;
    Rectangle<Point2D> location;
};
template <class Archive>
void serialize(Archive& ar, DetectInfo& data)
{
    ar(data.classId, data.confidence, data.location);
}

struct ImageInfo {
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint32_t widthAligned;
    uint32_t heightAligned;
    RawDataBufferHigh buf;  // data buffer for image
};
template <class Archive>
void serialize(Archive& ar, ImageInfo& data)
{
    ar(data.format, data.width, data.height, data.widthAligned, data.heightAligned, data.buf);
}

struct FaceObject {
    DetectInfo info;
    RawDataBufferHigh landmarks;
    RawDataBufferHigh featureVector;
    ImageInfo imgCroped;
    ImageInfo imgAffine;
};
template <class Archive>
void serialize(Archive& ar, FaceObject& data)
{
    ar(data.info, data.landmarks, data.featureVector, data.imgCroped, data.imgAffine);
}

struct DeviceStreamData {
    StreamInfo info;
    vector<DetectInfo> detectResult;
    vector<FaceObject> face;
    ImageInfo imgOrigin;
    ImageInfo detectImg;
    std::vector<int> detIdx;
    std::vector<int> trkIdx;
    int isKeyFrame;
    int isTracked;
    int trkFrameID;
    int isSplited;
    int isOutputStream;
};
template <class Archive>
void serialize(Archive& ar, DeviceStreamData& data)
{
    ar(data.info,
        data.imgOrigin,
        data.detectImg,
        data.detectResult,
        data.face,
        data.isKeyFrame,
        data.isTracked,
        data.detIdx,
        data.trkIdx,
        data.trkFrameID,
        data.isSplited,
        data.isOutputStream);
}

#define USE_DEFINE_ERROR 0x6001
enum { HIAI_IDE_ERROR_CODE, HIAI_IDE_INFO_CODE, HIAI_IDE_WARNING_CODE };
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_ERROR, HIAI_IDE_ERROR, "");
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_INFO, HIAI_IDE_INFO, "");
HIAI_DEF_ERROR_CODE(USE_DEFINE_ERROR, HIAI_WARNING, HIAI_IDE_WARNING, "");

#ifndef ALIGN_UP
#define ALIGN_UP(x, align) ((((x) + ((align)-1)) / (align)) * (align))
#endif

#endif