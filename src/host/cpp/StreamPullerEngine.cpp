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

#include "StreamPullerEngine.h"
#include "engine_tools.h"
#include "hiaiengine/ai_memory.h"
#include <chrono>
#include "inc/common.h"
std::shared_ptr<AVFormatContext> CreateFormatContext(const std::string& streamName)
{
    AVFormatContext* formatContext = nullptr;
    AVDictionary* options = nullptr;
    av_dict_set(&options, "rtsp_transport", "tcp", 0);
    av_dict_set(&options, "stimeout", "3000000", 0);
    int ret = avformat_open_input(&formatContext, streamName.c_str(), nullptr, &options);
    if (options != nullptr) {
        av_dict_free(&options);
    }
    if (ret != 0) {
        printf("Couldn't open input stream %s, ret=%d\n", streamName.c_str(), ret);
        return nullptr;
    }
    ret = avformat_find_stream_info(formatContext, nullptr);
    if (ret != 0) {
        printf("Couldn't find stream information\n");
        return nullptr;
    }
    return std::shared_ptr<AVFormatContext>(formatContext, [](AVFormatContext* p) {
        if (p) {
            avformat_close_input(&p);
        }
    });
}

StreamPullerEngine::~StreamPullerEngine()
{
    StopStream();
}

int StreamPullerEngine::GetStreamInfo()
{
    if (pFormatCtx != nullptr) {
        videoIndex = -1;
        for (int i = 0; i < pFormatCtx->nb_streams; i++) {
            AVStream* inStream = pFormatCtx->streams[i];
            if (inStream->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                videoIndex = i;
                mHeight = inStream->codecpar->height;
                mWidth = inStream->codecpar->width;
                AVCodecID codecId = inStream->codecpar->codec_id;
                if (codecId == AV_CODEC_ID_H264) {
                    format = 0;
                    return 0;
                } else if (codecId == AV_CODEC_ID_H265) {
                    format = 1;
                    return 0;
                } else {
                    printf("\033[0;31mError unsupported format %d\033[0m\n", codecId);
                    return -1;
                }
            } else {
                printf("Error codec_type %d\n", inStream->codecpar->codec_type);
                return -1;
            }
        }
        if (videoIndex == -1) {
            printf("Didn't find a video stream\n");
            return -1;
        }
    } else {
        printf("pFormatCtx is null!\n");
        return -1;
    }
}

void StreamPullerEngine::PullStreamDataLoop()
{
    AVPacket pkt;
    while (1) {
        if (stop || pFormatCtx == nullptr || failedCount >5) {
            g_isResume = true;
            printf("channel %d begin to reconnect...\n", channelId);
            break;
        }
        av_init_packet(&pkt);
        int ret = av_read_frame(pFormatCtx.get(), &pkt);
        if (ret != 0) {
            failedCount++;
            printf("channel %d Read frame failed, continue!\n", channelId);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        } else if (pkt.stream_index == videoIndex) {
            if (pkt.size <= 0) {
                printf("channel %d Invalid pkt.size %d\n", channelId, pkt.size);
                continue;
            }
            std::shared_ptr<StreamRawData> output = std::make_shared<StreamRawData>();
            frameNum++;
            output->info.channelId = channelId;
            output->info.format = format;
            output->info.isEOS = 0;
            output->info.isDisplayChannel = 1;
            output->info.frameId = frameNum;

            uint8_t* buffer = nullptr;
            ret = hiai::HIAIMemory::HIAI_DMalloc(pkt.size, (void*&)buffer);
            if (ret != HIAI_OK || buffer == nullptr) {
                printf("channel %d HIAI_DMalloc buffer faild\n", channelId);
                av_packet_unref(&pkt);
                break;
            } else {
                ret = memcpy_s(buffer, pkt.size, pkt.data, pkt.size);
                if (ret != 0) {
                    printf("channel %d memory copy faild\n", channelId);
                    av_packet_unref(&pkt);
                    hiai::HIAIMemory::HIAI_DFree(buffer);
                    break;
                }
                output->buf.data.reset(buffer, [](uint8_t* p) {});
                output->buf.len_of_byte = pkt.size;
            }
            HIAI_StatusT ret = SendData(0, "StreamRawData", std::static_pointer_cast<void>(output));
            if (ret != HIAI_OK) {
                failedCount++;
                printf("channel %d StreamPuller send data failed %d\n", channelId, ret);
                printf("channel %d sleep for 1 seconds\n", channelId);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            av_packet_unref(&pkt);
        } else {
            av_packet_unref(&pkt);
        }
    }

    printf("channel %d PullStreamDataLoop end of stream\n", channelId);
    stop = 1;
}

void StreamPullerEngine::StopStream()
{
    stop = 1;
    if (sendDataRunner.joinable()) {
        sendDataRunner.join();
    }
}

HIAI_StatusT StreamPullerEngine::StartStream(const string& streamName)
{
    StopStream();
    stop = 0;
    pFormatCtx = CreateFormatContext(streamName);
    if (pFormatCtx == nullptr) {
        return HIAI_ERROR;
    }
    // for debug dump
    av_dump_format(pFormatCtx.get(), 0, streamName.c_str(), 0);
    // get stream infomation
    if (GetStreamInfo() == -1) {
        return HIAI_ERROR;
    }
    // set up loop threads
    sendDataRunner = std::thread(&StreamPullerEngine::PullStreamDataLoop, this);
    return HIAI_OK;
}

HIAI_StatusT StreamPullerEngine::Init(
    const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    auto aimap = KvMap(config);

    if (!(std::stringstream(aimap["channel_id"]) >> channelId)) {
        std::cout<< "channel_id is not digit,please check!" << std::endl;
        return HIAI_ERROR;
    }
    if (!(channelId >= 0 && channelId < g_NUM_CAMERAS)) {
        std::cout<< "channelId must be larger than or equal than 0 and less than "<<g_NUM_CAMERAS << std::endl;
        return HIAI_ERROR;
    }

    avformat_network_init();

    return HIAI_OK;
}

HIAI_IMPL_ENGINE_PROCESS("StreamPullerEngine", StreamPullerEngine, RP_INPUT_SIZE)
{
    if (arg0 != nullptr) {
        shared_ptr<string> inputArg = std::static_pointer_cast<string>(arg0);
        if (!inputArg->empty()) {
            StartStream(*inputArg);
        }
    }
    return HIAI_OK;
}