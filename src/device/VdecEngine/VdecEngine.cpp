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

#include "VdecEngine.h"
#include <hiaiengine/data_type.h>
#include <malloc.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <vector>
#include <string>
#include "hiaiengine/ai_model_manager.h"
#include "hiaiengine/api.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/log.h"

#define FACE_DETECT_RESISE_WIDTH 416
#define FACE_DETECT_RESISE_HEIGHT 416
#define VPC_WIDTH_ALIGN (16)
#define VPC_HEIGHT_ALIGN (2)
HIAI_REGISTER_DATA_TYPE("DeviceStreamData", DeviceStreamData);
HIAI_REGISTER_SERIALIZE_FUNC("RawDataBufferHigh", RawDataBufferHigh, GetTransSearPtr, GetTransDearPtr);
HIAI_REGISTER_SERIALIZE_FUNC("StreamRawData", StreamRawData, StreamRawDataSerialize, StreamRawDataDeserialize);

void VdecEngine::VdecResultCallback(FRAME* frame, void* hiaiData)
{
    /* judge parameter */
    if (hiaiData == NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " input parameter of VdecResultCallback error!");
        return;
    }

    /* Get decode video information */
    DecodeH26xInfo* decodeH26xInfo = (DecodeH26xInfo*)hiaiData;
    VdecEngine* enginePtr = (VdecEngine*)decodeH26xInfo->vdecEnginePtr;
    enginePtr->vdecCounter++;

    /*
     * convert hbfc to yuv420
     */
    FRAME_BUFFER frameBufTemp;
    frameBufTemp.info = decodeH26xInfo->info;
    const int SKIP_FRAME_INTERVAL = 2;
    if (frameBufTemp.info.frameId % SKIP_FRAME_INTERVAL) {
        return;
    }

    if (enginePtr->VdecImageResize(frame, frameBufTemp.cropImageList) != HIAI_OK) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " VdecImageResize of VdecResultCallback error!");
        return;
    }

    enginePtr->frameQueue.push(frameBufTemp);
    return;
}

void VdecEngine::VdecErrorCallback(VDECERR* vdecErr)
{
    HIAI_ENGINE_LOG(
        HIAI_IDE_ERROR, "VdecErrorCallback vdec error type=%d,error channel=%d", vdecErr->errType, vdecErr->channelId);
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "VdecErrorCallback vdec error！");
}

HIAI_StatusT VdecEngine::Init(const AIConfig& config, const std::vector<AIModelDescription>& model_desc)
{
    HIAI_StatusT ret = HIAI_OK;
    /* create the vdec object */
    if (piDvppApiVdec == NULL) {
        ret = CreateVdecApi(piDvppApiVdec, 0);
        if ((ret != 0) || (piDvppApiVdec == NULL)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "fail to intialize dvpp api!");
            return HIAI_ERROR;
        }
    }

    /* create the vpc object */
    if (piDvppApiVpc == NULL) {
        ret = CreateDvppApi(piDvppApiVpc);
        if ((ret != 0) || (piDvppApiVpc == NULL)) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "fail to intialize vpc api!");
            return HIAI_ERROR;
        }
    }

    // pre allocate memory for the detectImg
    ret = hiai::HIAIMemory::HIAI_DVPP_DMalloc(ALIGN_UP(FACE_DETECT_RESISE_WIDTH, VPC_WIDTH_ALIGN) *
                                                  ALIGN_UP(FACE_DETECT_RESISE_HEIGHT, VPC_HEIGHT_ALIGN) * 3 / 2,
        (void*&)output1Buffer);
    if (ret != HIAI_OK || output1Buffer == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " HIAI_DVPP_DMalloc of VdecImageResize() error!");
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

VdecEngine::~VdecEngine()
{
    /* Destroy vdec object */
    if (piDvppApiVdec != NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Destroy vdec api!");
        DestroyVdecApi(piDvppApiVdec, 0);
        piDvppApiVdec = NULL;
    }

    /* Destroy vpc object */
    if (piDvppApiVpc != NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Destroy vpc api!");
        DestroyDvppApi(piDvppApiVpc);
        piDvppApiVpc = NULL;
    }

    hiai::HIAIMemory::HIAI_DVPP_DFree(output1Buffer);
}

/**
 * @ingroup hiaiengine
 *
 **/
HIAI_IMPL_ENGINE_PROCESS("VdecEngine", VdecEngine, INPUT_SIZE)
{
    std::shared_ptr<StreamRawData> streamRawData = std::static_pointer_cast<StreamRawData>(arg0);

    if (streamRawData == nullptr) {
        return HIAI_ERROR;
    }

    if (DecodeH26xVideo(streamRawData) != HIAI_OK) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " SendData of VdecEngine error!");
        return HIAI_ERROR;
    }

    while (!frameQueue.empty()) {
        FRAME_BUFFER frameTemp = frameQueue.front();
        frameQueue.pop();

        if (frameTemp.info.isDisplayChannel) {
            std::shared_ptr<RawDataBufferHigh> outputStreamData = std::make_shared<RawDataBufferHigh>();
            outputStreamData->data = frameTemp.cropImageList[0].buf.data;
            outputStreamData->len_of_byte = frameTemp.cropImageList[0].buf.len_of_byte;
            outputStreamData->frameId = frameTemp.info.frameId;
            outputStreamData->channelID = frameTemp.info.channelId;

            if (hiai::Engine::SendData(1, "RawDataBufferHigh", std::static_pointer_cast<void>(outputStreamData)) !=
                HIAI_OK) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " SendData of outputStreamData error!");
                return HIAI_ERROR;
            }
        }

        /* construct data transmission struct */
        std::shared_ptr<DeviceStreamData> deviceStreamData = std::make_shared<DeviceStreamData>();
        deviceStreamData->info = frameTemp.info;
        deviceStreamData->imgOrigin = frameTemp.cropImageList[0];
        deviceStreamData->detectImg = frameTemp.cropImageList[1];

        if (hiai::Engine::SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(deviceStreamData)) !=
            HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " SendData of VdecEngine error!");
            return HIAI_ERROR;
        }
    }
    
    return HIAI_OK;
}

HIAI_StatusT VdecEngine::DecodeH26xVideo(std::shared_ptr<StreamRawData> streamRawData)
{
    /* define parameters for DVPP Vdec */
    vdec_in_msg vdecMsg;

    /* decode the H26x video */
    // set input fromat: h264, h265 warning, please define enum
    const int FORMAT_LEN = 4;
    if (streamRawData->info.format == 0) {
        int mcs = memcpy_s(vdecMsg.video_format, FORMAT_LEN, "h264", FORMAT_LEN);
		if (mcs != 0) {
			return HIAI_ERROR;
		}
    } else {
        int mcs = memcpy_s(vdecMsg.video_format, FORMAT_LEN, "h265", FORMAT_LEN);
		if (mcs != 0) {
			return HIAI_ERROR;
		}
    }
    // set output format: yuv420sp_UV(default)=nv12, yuv420sp_VU=nv21
 
    vdecMsg.in_buffer = (char*)streamRawData->buf.data.get();
    vdecMsg.in_buffer_size = streamRawData->buf.len_of_byte;

    std::shared_ptr<DecodeH26xInfo> decodeH26xInfo = make_shared<DecodeH26xInfo>();
    decodeH26xInfo->setFrameIndex(streamRawData->info.frameId);
    decodeH26xInfo->info = streamRawData->info;
    decodeH26xInfo->vdecEnginePtr = (void*)this;
    clock_gettime(CLOCK_REALTIME, &decodeH26xInfo->time);
    vdecMsg.hiai_data_sp = decodeH26xInfo;  // if use hiai_data_sp, set hiai_data as NULL
    vdecMsg.channelId = streamRawData->info.channelId;
    vdecMsg.hiai_data = NULL;  // if use hiai_data_sp, set hiai_data as NULL
    vdecMsg.call_back = VdecEngine::VdecResultCallback;
    vdecMsg.err_report = VdecEngine::VdecErrorCallback;

    dvppapi_ctl_msg dvppapiCtlMsg;
    dvppapiCtlMsg.in_size = sizeof(vdec_in_msg);
    dvppapiCtlMsg.in = (void*)&vdecMsg;

    if (VdecCtl(piDvppApiVdec, DVPP_CTL_VDEC_PROC, &dvppapiCtlMsg, 0) != 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " VdecCtl of DecodeH26xVideo() error!");
        vdecFailedCounter++;
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

HIAI_StatusT VdecEngine::VdecImageResize(FRAME* frame, std::vector<ImageInfo>& cropImageList)
{
    /*
     * vpc, change format and resize
     */
    /* Construct VPC input parameter */
    std::shared_ptr<VpcUserImageConfigure> imageConfigure(new VpcUserImageConfigure);
    imageConfigure->bareDataAddr = nullptr;
    imageConfigure->bareDataBufferSize = 0;
    // set input frame format
    imageConfigure->isCompressData = true;
    imageConfigure->widthStride = frame->width;
    imageConfigure->heightStride = frame->height;
    string imageFormat(frame->image_format);
    if (frame->bitdepth == 8) {
        if (imageFormat == "nv12") {
            imageConfigure->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV;
        } else {
            imageConfigure->inputFormat = INPUT_YUV420_SEMI_PLANNER_VU;
        }
    } else {
        if (imageFormat == "nv12") {
            imageConfigure->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV_10BIT;
        } else {
            imageConfigure->inputFormat = INPUT_YUV420_SEMI_PLANNER_VU_10BIT;
        }
    }
    // set hfbc input address
    VpcCompressDataConfigure* compressDataConfigure = &imageConfigure->compressDataConfigure;
    uintptr_t baseAddr = (uintptr_t)frame->buffer;
    compressDataConfigure->lumaHeadAddr = baseAddr + frame->offset_head_y;
    compressDataConfigure->chromaHeadAddr = baseAddr + frame->offset_head_c;
    compressDataConfigure->lumaPayloadAddr = baseAddr + frame->offset_payload_y;
    compressDataConfigure->chromaPayloadAddr = baseAddr + frame->offset_payload_c;
    compressDataConfigure->lumaHeadStride = frame->stride_head;
    compressDataConfigure->chromaHeadStride = frame->stride_head;
    compressDataConfigure->lumaPayloadStride = frame->stride_payload;
    compressDataConfigure->chromaPayloadStride = frame->stride_payload;

    imageConfigure->outputFormat = OUTPUT_YUV420SP_UV;
    imageConfigure->yuvSumEnable = false;
    imageConfigure->cmdListBufferAddr = nullptr;
    imageConfigure->cmdListBufferSize = 0;

    /* ouput 0: raw image, convert format from INPUT_YUV420_SEMI_PLANNER_VU to
     * OUTPUT_YUV420SP_UV */
    /* ouput 1: resized image for detect network input, convert format from
     * INPUT_YUV420_SEMI_PLANNER_VU to OUTPUT_YUV420SP_UV */
    std::shared_ptr<VpcUserRoiConfigure> roiConfigureOutput0(new VpcUserRoiConfigure);
    std::shared_ptr<VpcUserRoiConfigure> roiConfigureOutput1(new VpcUserRoiConfigure);
    imageConfigure->roiConfigure = roiConfigureOutput0.get();
    roiConfigureOutput0->next = roiConfigureOutput1.get();
    roiConfigureOutput1->next = nullptr;
    const int MOD_2 = 2;
    /* ouput 0: raw image*/
    VpcUserRoiInputConfigure* inputConfigure = &roiConfigureOutput0->inputConfigure;
    // set map area: the entire picture
    inputConfigure->cropArea.leftOffset = 0;                                                              // even
    inputConfigure->cropArea.rightOffset = (frame->width % MOD_2) ? frame->width : (frame->width - 1);    // odd
    inputConfigure->cropArea.upOffset = 0;                                                                // even
    inputConfigure->cropArea.downOffset = (frame->height % MOD_2) ? frame->height : (frame->height - 1);  // odd

    // Construct output buffer
    VpcUserRoiOutputConfigure* outputConfigure = &roiConfigureOutput0->outputConfigure;
    outputConfigure->widthStride = ALIGN_UP(frame->width, VPC_WIDTH_ALIGN);                              // align to 16
    outputConfigure->heightStride = ALIGN_UP(frame->height, VPC_HEIGHT_ALIGN);                           // align to 2
    outputConfigure->bufferSize = outputConfigure->widthStride * outputConfigure->heightStride * 3 / 2;  // yuv420sp_UV
    // Construct output buffer
    uint8_t* output0Buffer = nullptr;
    HIAI_StatusT ret = hiai::HIAIMemory::HIAI_DVPP_DMalloc(outputConfigure->bufferSize, (void*&)output0Buffer);
    if (ret != HIAI_OK || output0Buffer == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " HIAI_DVPP_DMalloc of VdecImageResize() error!");
        return HIAI_ERROR;
    }
    outputConfigure->addr = output0Buffer;  // align to 128

    // set crop area:
    outputConfigure->outputArea.leftOffset = 0;  // 这个偏移值需要16对齐
    outputConfigure->outputArea.rightOffset = (frame->width % MOD_2) ? frame->width : (frame->width - 1);
    outputConfigure->outputArea.upOffset = 0;
    outputConfigure->outputArea.downOffset = (frame->height % MOD_2) ? frame->height : (frame->height - 1);

    /* ouput 1: resized image for detect network input */
    inputConfigure = &roiConfigureOutput1->inputConfigure;
    // set map area: the entire picture
    inputConfigure->cropArea = roiConfigureOutput0->inputConfigure.cropArea;  // same as output 0

    // Construct output buffer
    outputConfigure = &roiConfigureOutput1->outputConfigure;
    outputConfigure->widthStride = ALIGN_UP(FACE_DETECT_RESISE_WIDTH, VPC_WIDTH_ALIGN);                  // align to 16
    outputConfigure->heightStride = ALIGN_UP(FACE_DETECT_RESISE_HEIGHT, VPC_HEIGHT_ALIGN);               // align to 2
    outputConfigure->bufferSize = outputConfigure->widthStride * outputConfigure->heightStride * 3 / 2;  // yuv420sp_UV
    outputConfigure->addr = output1Buffer;

    // set crop area:
    outputConfigure->outputArea.leftOffset = 0;  // 这个偏移值需要16对齐
    outputConfigure->outputArea.rightOffset =
        (FACE_DETECT_RESISE_WIDTH % MOD_2) ? FACE_DETECT_RESISE_WIDTH : (FACE_DETECT_RESISE_WIDTH - 1);
    outputConfigure->outputArea.upOffset = 0;
    outputConfigure->outputArea.downOffset =
        (FACE_DETECT_RESISE_HEIGHT % MOD_2) ? FACE_DETECT_RESISE_HEIGHT : (FACE_DETECT_RESISE_HEIGHT - 1);

    /* process of VPC */
    dvppapi_ctl_msg dvppApiCtlMsg;
    dvppApiCtlMsg.in = static_cast<void*>(imageConfigure.get());
    dvppApiCtlMsg.in_size = sizeof(VpcUserImageConfigure);

    if (DvppCtl(piDvppApiVpc, DVPP_CTL_VPC_PROC, &dvppApiCtlMsg) != 0) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " vpc of VdecImageResize() error!");
        hiai::HIAIMemory::HIAI_DVPP_DFree(roiConfigureOutput0->outputConfigure.addr);
        return HIAI_ERROR;
    }

    /* Get output image */
    int i = 0;
    while (imageConfigure->roiConfigure != nullptr) {
        ImageInfo img;
        img.format = OUTPUT_YUV420SP_UV;
        img.width = imageConfigure->roiConfigure->outputConfigure.outputArea.rightOffset;
        img.height = imageConfigure->roiConfigure->outputConfigure.outputArea.downOffset;
        img.widthAligned = imageConfigure->roiConfigure->outputConfigure.widthStride;
        img.heightAligned = imageConfigure->roiConfigure->outputConfigure.heightStride;
        if (i == 0) {
            img.buf.data = std::shared_ptr<uint8_t>(imageConfigure->roiConfigure->outputConfigure.addr,
                [](std::uint8_t* data) { hiai::HIAIMemory::HIAI_DVPP_DFree(data); });
        } else {
            std::shared_ptr<u_int8_t> detectImgBuffer(
                new uint8_t[imageConfigure->roiConfigure->outputConfigure.bufferSize]);
            ret = memcpy_s(detectImgBuffer.get(),
                imageConfigure->roiConfigure->outputConfigure.bufferSize,
                imageConfigure->roiConfigure->outputConfigure.addr,
                imageConfigure->roiConfigure->outputConfigure.bufferSize);
            if (ret != 0) {
                HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " The memcpy of result of vpc error!");
                return HIAI_ERROR;
            }
            img.buf.data = detectImgBuffer;
        }
        img.buf.len_of_byte = imageConfigure->roiConfigure->outputConfigure.bufferSize;

        /* update */
        cropImageList.push_back(img);
        imageConfigure->roiConfigure = imageConfigure->roiConfigure->next;
        i++;
    }

    return HIAI_OK;
}
