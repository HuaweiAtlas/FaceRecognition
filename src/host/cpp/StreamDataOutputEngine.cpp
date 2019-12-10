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
#include "StreamDataOutputEngine.h"
#include "engine_tools.h"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include "inc/common.h"

using std::shared_ptr;
using std::to_string;

#define IMAGE_WIDTH_1920 (1920)
#define IMAGE_HEIGHT_1080 (1080)
#define IMAGE_RESIZE_360 (360)
#define IMAGE_RESIZE_480 (480)
#define MAX_PIXEL_255 (255)

HIAI_StatusT StreamDataOutputEngine::ParamsCheck(const hiai::AIConfig& config, std::string& link)
{
    std::map<string, string> keyValueConfig = KvMap(config);
    if (CheckEmpty(keyValueConfig["rtsp_link"]) != HIAI_OK) {
        std::cout << "rtsp_link is empty, please check!" << std::endl;
        return HIAI_ERROR;
    }
    link = keyValueConfig["rtsp_link"];

    if (CheckEmpty(keyValueConfig["feature_lib_path"]) != HIAI_OK) {
        std::cout << "feature_lib_path is empty, please check!" << std::endl;
        return HIAI_ERROR;
    }
    std::string featureLibPath = keyValueConfig["feature_lib_path"];

    if (CheckEmpty(keyValueConfig["feature_len"]) != HIAI_OK) {
        std::cout << "feature_len is empty, please check!" << std::endl;
        return HIAI_ERROR;
    }
    uint32_t featureLen = 0;
    if (!(std::stringstream(keyValueConfig["feature_len"]) >> featureLen)) {
        std::cout << "feature_len is not digit,please check!" << std::endl;
        return HIAI_ERROR;
    }

    if (CheckEmpty(keyValueConfig["feature_num"]) != HIAI_OK) {
        std::cout << "feature_num is empty, please check!" << std::endl;
        return HIAI_ERROR;
    }
    uint32_t featureNum = 0;
    if (!(std::stringstream(keyValueConfig["feature_num"]) >> featureNum)) {
        std::cout << "feature_num is not digit,please check!" << std::endl;
        return HIAI_ERROR;
    }

    const int FEATURE_LEN = 512;
    if (featureLen != FEATURE_LEN || featureNum <= 0) {
        std::cout << "feature_len or feature_num in graph.config is invalid, please check!" << std::endl;
        return HIAI_ERROR;
    }

    if (FeaturelibRead(featureLib, featureLibPath, featureLen, featureNum) != 0) {
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

HIAI_StatusT StreamDataOutputEngine::AvInit(std::string& rtspLink)
{
    // init to push frame
    avcodec_register_all();
    av_register_all();
    avformat_network_init();
    pEncoder = avcodec_find_encoder(AV_CODEC_ID_MPEG4);
    if (pEncoder == NULL) {
        return HIAI_ERROR;
    }
    pCodecCtx = avcodec_alloc_context3(pEncoder);
    if (pCodecCtx == NULL) {
        return HIAI_ERROR;
    }
    pCodecCtx->codec_id = pEncoder->id;
    pCodecCtx->thread_count = 8;
    pCodecCtx->bit_rate = 1 * 1024 * 1024 * 12;
    pCodecCtx->width = IMAGE_WIDTH_1920;
    pCodecCtx->height = IMAGE_HEIGHT_1080;
    pCodecCtx->time_base.num = 1;
    pCodecCtx->time_base.den = 12;
    pCodecCtx->gop_size = 100;
    pCodecCtx->max_b_frames = 0;
    pCodecCtx->pix_fmt = AV_PIX_FMT_YUV420P;
    avcodec_open2(pCodecCtx, pEncoder, 0);
    avformat_alloc_output_context2(&pFormatCtx, NULL, "rtsp", rtspLink.c_str());
    if (pFormatCtx == NULL) {
        std::cout << "StreamDataOutputEngine allocate output context failed!" << std::endl;
        return HIAI_ERROR;
    }
    video_steam = avformat_new_stream(pFormatCtx, NULL);
    if (video_steam == NULL) {
        return HIAI_ERROR;
    }
    video_steam->codecpar->codec_tag = 0;
    avcodec_parameters_from_context(video_steam->codecpar, pCodecCtx);
    av_dump_format(pFormatCtx, 0, rtspLink.c_str(), 1);
    AVDictionary* format_opts = NULL;
    av_dict_set(&format_opts, "rtsp_transport", "tcp", 0);

    frame = av_frame_alloc();
    if (frame == NULL) {
        return HIAI_ERROR;
    }
    frame->format = AV_PIX_FMT_NV12;
    frame->width = IMAGE_WIDTH_1920;
    frame->height = IMAGE_HEIGHT_1080;
    frame->pts = 0;
    if (avformat_write_header(pFormatCtx, &format_opts) != 0) {
        std::cout << "StreamDataOutputEngine write header failed!" << std::endl;
        return HIAI_ERROR;
    }
    isHeaderWrite = true;

    int mss = memset_s(&packet, sizeof(packet), 0, sizeof(AVPacket));
    if (mss != 0){
        return HIAI_ERROR;
    }
    isPacketSet = true;

    readyFramYuv = new uint8_t[IMAGE_WIDTH_1920 * IMAGE_HEIGHT_1080 * 3 / 2];
    if (readyFramYuv == NULL) {
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

HIAI_StatusT StreamDataOutputEngine::Init(
    const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    std::string rtspLink;
    if (ParamsCheck(config, rtspLink) != HIAI_OK) {
        return HIAI_ERROR;
    }

    if (g_isDisplay == 1) {
        if (HIAI_OK != AvInit(rtspLink)) {
            return HIAI_ERROR;
        }
    }

    const int DISPLAY_NUM = 12;
    frameDataMap.resize(DISPLAY_NUM);
    structuredInfoMap.resize(DISPLAY_NUM);
    frameReadyQueue.resize(DISPLAY_NUM);

    std::cout << "StreamDataOutputEngine init SUCCESS!" << std::endl;
    return HIAI_OK;
}

StreamDataOutputEngine::~StreamDataOutputEngine()
{
    if (pFormatCtx && isHeaderWrite) {
        if (av_write_trailer(pFormatCtx) != 0) {
            std::cout << "StreamDataOutputEngine failed to write tailer!" << std::endl;
        }
    }
	
    if (pEncoder) {
        pEncoder = NULL;
    }

    if (video_steam) {
        avcodec_close(pCodecCtx);
        video_steam = NULL;
    }

    if (pFormatCtx) {
        avio_closep(&pFormatCtx->pb);
        avformat_free_context(pFormatCtx);
        pFormatCtx = NULL;
    }

    if (pCodecCtx) {
        avcodec_free_context(&pCodecCtx);
    }

    if (frame) {
        av_frame_free(&frame);
    }

    if (isPacketSet) {
        av_free_packet(&packet);
    }

    if (readyFramYuv) {
        delete[] readyFramYuv;
        readyFramYuv = NULL;
    }
}

static void BRGInterleave2YUV(uint8_t* pBGR, uint8_t* pYUV, int32_t width, int32_t height)
{
    uint32_t i, j;
    int32_t valueTemp;
    int32_t widthUV = width / 2;
    uint8_t* pU = pYUV + width * height;
    uint8_t* pV = pU + (width / 2) * (height / 2);
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            int32_t offset = width * i + j;
            float bValue = (float)*(pBGR + offset * 3);
            float gValue = (float)*(pBGR + offset * 3 + 1);
            float rValue = (float)*(pBGR + offset * 3 + 2);

            bValue += (float)226.688;
            gValue -= (float)135.7;
            rValue += (float)179.48;

            // cal Y value;
            valueTemp = (int32_t)(rValue * 0.2989 + gValue * 0.5866 + bValue * 0.1145);
            pYUV[(size_t)(i * width + j)] = valueTemp < 0 ? 0 : (valueTemp > MAX_PIXEL_255 ? MAX_PIXEL_255 : valueTemp);
            // cal U value
            int32_t indexUV = (i >> 1) * widthUV + (j >> 1);
            valueTemp = (int32_t)(rValue * (-0.1688) + gValue * (-0.3312) + bValue * (0.5));
            pU[indexUV] = valueTemp < 0 ? 0 : (valueTemp > MAX_PIXEL_255 ? MAX_PIXEL_255 : valueTemp);
            // cal V value
            valueTemp = (int32_t)(rValue * (0.5) + gValue * (-0.4184) + bValue * (-0.0816));
            pV[indexUV] = valueTemp < 0 ? 0 : (valueTemp > MAX_PIXEL_255 ? MAX_PIXEL_255 : valueTemp);
        }
    }
}

HIAI_StatusT StreamDataOutputEngine::FramePusher(uint8_t* pYUV)
{
    uint8_t* buff = pYUV;
    int size = IMAGE_WIDTH_1920 * IMAGE_HEIGHT_1080;
    frame->pts = iFrame;
    iFrame++;
    frame->data[0] = buff;
    frame->data[1] = buff + size;
    frame->data[2] = buff + size * 5 / 4;
    frame->linesize[0] = IMAGE_WIDTH_1920;
    frame->linesize[1] = IMAGE_WIDTH_1920 / 2;
    frame->linesize[2] = IMAGE_WIDTH_1920 / 2;

    int ret = avcodec_send_frame(pCodecCtx, frame);
    ret = avcodec_receive_packet(pCodecCtx, &packet);
    if (ret != 0 || packet.size > 0) {
        ;
    } else {
        return HIAI_ERROR;
    }
    packet.pts = av_rescale_q(packet.pts, pCodecCtx->time_base, video_steam->time_base);
    packet.dts = av_rescale_q(packet.dts, pCodecCtx->time_base, video_steam->time_base);
    packet.duration = av_rescale_q(packet.duration, pCodecCtx->time_base, video_steam->time_base);
    av_interleaved_write_frame(pFormatCtx, &packet);
    return HIAI_OK;
}

HIAI_StatusT StreamDataOutputEngine::PrepareToPush(std::shared_ptr<DeviceStreamData> imageInput)
{
    int height = imageInput->imgOrigin.heightAligned;
    int width = imageInput->imgOrigin.widthAligned;
    cv::Mat srcNv12Mat(height * 3 / 2, width, CV_8UC1, imageInput->imgOrigin.buf.data.get());
    cv::Mat dstRGB888(height, width, CV_8UC3);
    cv::cvtColor(srcNv12Mat, dstRGB888, cv::COLOR_YUV2BGR_NV12);
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.5f, 1.5f, 0, 2, CV_AA);
    CvPoint leftTopPoint, rightBottomPoint;
    for (int i = 0; i < imageInput->face.size(); i++) {

        int32_t featureNum = imageInput->face[i].featureVector.len_of_byte / sizeof(float);
        float* featureVectorPtr = (float*)imageInput->face[i].featureVector.data.get();
        float similarity = 0;

        int32_t objectId = SearchFeatureLib(featureLib, featureVectorPtr, featureNum, similarity);

        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << similarity;

        std::string strID;
        if (objectId < 0) {
            strID = stream.str() + ",ID=Unknown";
        } else {
            strID = stream.str() + ",ID=" + std::to_string(objectId);
        }

        leftTopPoint.x = imageInput->face[i].info.location.anchor_lt.x;
        leftTopPoint.y = imageInput->face[i].info.location.anchor_lt.y;
        rightBottomPoint.x = imageInput->face[i].info.location.anchor_rb.x;
        rightBottomPoint.y = imageInput->face[i].info.location.anchor_rb.y;

        printf("channel: %u, frame id: #%u, bound box: (%d, %d),(%d, %d), largest "
               "similarity:%s, isTracked=%d\n",
            imageInput->info.channelId,
            (uint32_t)imageInput->info.frameId,
            leftTopPoint.x,
            leftTopPoint.y,
            rightBottomPoint.x,
            rightBottomPoint.y,
            strID.c_str(),
            imageInput->isTracked);

        cv::rectangle(dstRGB888, leftTopPoint, rightBottomPoint, cv::Scalar(0, 0, MAX_PIXEL_255), 2);
        cv::putText(dstRGB888,
            strID,
            cvPoint(leftTopPoint.x, leftTopPoint.y - 10),
            CV_FONT_HERSHEY_PLAIN,
            1,
            cv::Scalar(0, 0, MAX_PIXEL_255),
            1);
    }

    if (g_isDisplay != 1) {
        return HIAI_OK;
    }

    cv::Mat dstResize;
    cv::resize(dstRGB888, dstResize, cv::Size(IMAGE_RESIZE_480, IMAGE_RESIZE_360), (0, 0), (0, 0), cv::INTER_LINEAR);
    frameReadyQueue[imageInput->info.channelId].push_back(dstResize);
    SendFrame();
    return HIAI_OK;
}

HIAI_StatusT StreamDataOutputEngine::SendFrame()
{
    int isReady = 1;
    for (int i = 0; i < g_cameraNum; i++) {
        if (frameReadyQueue[i].empty()) {
            isReady = 0;
            break;
        }
    }

    // all the channel is ready to send
    if (isReady) {
        cv::Mat combine, combineRow1, combineRow2, combineRow3;
        if (g_cameraNum <= 4) {
            // 1~4 cameras
            std::vector<cv::Mat> row1;
            for (int i = 0; i < g_cameraNum; i++) {
                row1.push_back(frameReadyQueue[i].front());
            }
            for (int i = 0; i < 4 - g_cameraNum; i++) {
                cv::Mat zeroImage(IMAGE_RESIZE_360, IMAGE_RESIZE_480, CV_8UC3, cv::Scalar(0, 0, 0));
                row1.push_back(zeroImage);
            }
            cv::hconcat(row1, combineRow1);
            combineRow2 = cv::Mat(IMAGE_RESIZE_360, IMAGE_WIDTH_1920, CV_8UC3, cv::Scalar(0, 0, 0));
            combineRow3 = cv::Mat(IMAGE_RESIZE_360, IMAGE_WIDTH_1920, CV_8UC3, cv::Scalar(0, 0, 0));
        } else if (g_cameraNum <= 8) {
            // 5~8 cameras
            std::vector<cv::Mat> row1 = {frameReadyQueue[0].front(),
                frameReadyQueue[1].front(),
                frameReadyQueue[2].front(),
                frameReadyQueue[3].front()};
            cv::hconcat(row1, combineRow1);

            std::vector<cv::Mat> row2;
            for (int i = 4; i < g_cameraNum; i++) {
                row2.push_back(frameReadyQueue[i].front());
            }
            for (int i = 0; i < 8 - g_cameraNum; i++) {
                cv::Mat zeroImage(IMAGE_RESIZE_360, IMAGE_RESIZE_480, CV_8UC3, cv::Scalar(0, 0, 0));
                row2.push_back(zeroImage);
            }
            cv::hconcat(row2, combineRow2);
            combineRow3 = cv::Mat(IMAGE_RESIZE_360, IMAGE_WIDTH_1920, CV_8UC3, cv::Scalar(0, 0, 0));
        } else if (g_cameraNum <= 12) {
            // 9-12 cameras
            std::vector<cv::Mat> row1 = {frameReadyQueue[0].front(),
                frameReadyQueue[1].front(),
                frameReadyQueue[2].front(),
                frameReadyQueue[3].front()};
            cv::hconcat(row1, combineRow1);

            std::vector<cv::Mat> row2 = {frameReadyQueue[4].front(),
                frameReadyQueue[5].front(),
                frameReadyQueue[6].front(),
                frameReadyQueue[7].front()};
            cv::hconcat(row2, combineRow2);

            std::vector<cv::Mat> row3;
            for (int i = 8; i < g_cameraNum; i++) {
                row3.push_back(frameReadyQueue[i].front());
            }
            for (int i = 0; i < 12 - g_cameraNum; i++) {
                cv::Mat zeroImage(IMAGE_RESIZE_360, IMAGE_RESIZE_480, CV_8UC3, cv::Scalar(0, 0, 0));
                row3.push_back(zeroImage);
            }

            cv::hconcat(row3, combineRow3);
        }
        std::vector<cv::Mat> col = {combineRow1, combineRow2, combineRow3};
        cv::vconcat(col, combine);

        if (combine.data != NULL) {
            BRGInterleave2YUV(combine.data, readyFramYuv, IMAGE_WIDTH_1920, IMAGE_HEIGHT_1080);
            FramePusher(readyFramYuv);
        }

        for (int i = 0; i < g_cameraNum; i++) {
            frameReadyQueue[i].pop_front();
        }
    }

    return HIAI_OK;
}

HIAI_IMPL_ENGINE_PROCESS("StreamDataOutputEngine", StreamDataOutputEngine, SDO_INPUT_SIZE)
{
    if (arg1 != nullptr) {
        std::shared_ptr<RawDataBufferHigh> dataInput = static_pointer_cast<RawDataBufferHigh>(arg1);
        int channelID = dataInput->channelID;
        frameDataMap[channelID][dataInput->frameId] = dataInput;
        if (structuredInfoMap[channelID].find(dataInput->frameId) != structuredInfoMap[channelID].end()) {
            structuredInfoMap[channelID][dataInput->frameId]->imgOrigin.buf.data = dataInput->data;
            structuredInfoMap[channelID][dataInput->frameId]->imgOrigin.buf.len_of_byte = dataInput->len_of_byte;
            PrepareToPush(structuredInfoMap[channelID][dataInput->frameId]);
        }

        // erase the unused structured info
        auto iter = structuredInfoMap[channelID].begin();
        while (iter != structuredInfoMap[channelID].end()) {
            if ((iter->first) < dataInput->frameId) {
                iter = structuredInfoMap[channelID].erase(iter);
            } else {
                iter++;
            }
        }
    }

    if (arg0 != nullptr) {
        std::shared_ptr<DeviceStreamData> dataInput = static_pointer_cast<DeviceStreamData>(arg0);

        int channelID = dataInput->info.channelId;
        // the iuput data is structured info
        if (structuredInfoMap[channelID].find(dataInput->info.frameId) != structuredInfoMap[channelID].end()) {
            structuredInfoMap[channelID][dataInput->info.frameId]->face.insert(
                structuredInfoMap[channelID][dataInput->info.frameId]->face.end(),
                dataInput->face.begin(),
                dataInput->face.end());
        } else {
            structuredInfoMap[channelID][dataInput->info.frameId] = dataInput;
        }

        if (frameDataMap[channelID].find(dataInput->info.frameId) != frameDataMap[channelID].end()) {
            dataInput->imgOrigin.buf.data = frameDataMap[channelID][dataInput->info.frameId]->data;
            dataInput->imgOrigin.buf.len_of_byte = frameDataMap[channelID][dataInput->info.frameId]->len_of_byte;
            PrepareToPush(dataInput);
        }

        // erase the unused frame data
        auto iter = frameDataMap[channelID].begin();
        while (iter != frameDataMap[channelID].end()) {
            if ((iter->first) < dataInput->info.frameId) {
                iter = frameDataMap[channelID].erase(iter);
            } else {
                iter++;
            }
        }
    }
    return HIAI_OK;
}