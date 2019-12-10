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

#include "FaceLandmarkEngine.h"
#include "dvpp/Vpc.h"
#include "engine_tools.h"
#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <hiaiengine/api.h>
#include <hiaiengine/data_type.h>
#include <iostream>
#include <math.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

#define FACE_LANDMARK_WIDTH_ALIGN  (16)
#define FACE_LANDMARK_HEIGHT_ALIGN (2)

HIAI_StatusT FaceLandmarkEngine::ParamInit(std::map<string, string> &keyValueConfig)
{
    if (CheckInputParam(keyValueConfig, kBatchSize, kChannel, kHeight, kWidth) != HIAI_OK) {
        return HIAI_ERROR;
    }

    // the value limit the face num in one frame
    if (CheckEmpty(keyValueConfig["max_face_num_per_frame"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    if (!(std::stringstream(keyValueConfig["max_face_num_per_frame"]) >> maxFaceNumPerFrame)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "max_face_num_per_frame is not digit,please check!");
        return HIAI_ERROR;
    }
    if (maxFaceNumPerFrame < 1) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "max_face_num_per_frame must be larger than 0");
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

HIAI_StatusT FaceLandmarkEngine::DimCheck(std::vector<hiai::TensorDimension> &inputTensorDims,
                                          std::vector<hiai::TensorDimension> &outputTensorDims)
{
    for (auto &dims : inputTensorDims) {
        LogDumpDims(dims);
    }
    for (auto &dims : outputTensorDims) {
        LogDumpDims(dims);
    }
    if ((inputTensorDims[0].n != kBatchSize) || (inputTensorDims[0].c != kChannel) ||
        (inputTensorDims[0].h != kHeight) || (inputTensorDims[0].w != kWidth)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                        "[FaceLandmarkEngine] input dims of the "
                        "om model are not identical to the ones "
                        "defined in graph config");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

HIAI_StatusT FaceLandmarkEngine::PreAllocate(std::vector<hiai::TensorDimension> &inputTensorDims,
                                             std::vector<hiai::TensorDimension> &outputTensorDims)
{
    // pre allocate inference input data buffer
    kInputSize = kBatchSize * kChannel * kAlignedHeight * kAlignedWidth / 2;  // data format:yuv420sp

    std::shared_ptr<uint8_t> inPtr((uint8_t *)HIAI_DVPP_DMalloc(kInputSize), HIAI_DVPP_DFree);
    inputDataBuffer = std::make_pair(inPtr, kInputSize);
    hiai::AITensorDescription tensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
    std::shared_ptr<hiai::IAITensor> inputTensor =
        hiai::AITensorFactory::GetInstance()->CreateTensor(tensorDesc, (void *)inPtr.get(), kInputSize);
    inputTensorVec.push_back(inputTensor);

    // pre allocate model inference output data buffer, different frames will
    // reuse this buffer, so memory copy is needed
    for (uint32_t index = 0; index < outputTensorDims.size(); index++) {
        hiai::AITensorDescription outputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] outputTensorDims[index].size %d",
                        outputTensorDims[index].size);
        uint8_t *buf = (uint8_t *)HIAI_DVPP_DMalloc(outputTensorDims[index].size);
        if (buf == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
                            "[FaceLandmarkEngine] HIAI_DVPP_DMalloc for model "
                            "inference failed.");
            return HIAI_ERROR;
        }
        outputDataBuffer.push_back(std::shared_ptr<uint8_t>(buf, HIAI_DVPP_DFree));
        std::shared_ptr<hiai::IAITensor> outputTensor =
            hiai::AITensorFactory::GetInstance()->CreateTensor(outputTensorDesc, buf, outputTensorDims[index].size);
        shared_ptr<hiai::AINeuralNetworkBuffer> nn_tensor =
            static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputTensor);
        nn_tensor->SetName(outputTensorDims[index].name);
        outputTensorVec.push_back(outputTensor);
    }
}

/**
 * @brief: init, inherited from hiaiengine lib
 */
HIAI_StatusT FaceLandmarkEngine::Init(const hiai::AIConfig &config,
                                      const std::vector<hiai::AIModelDescription> &model_desc)
{
    HIAI_ENGINE_LOG(this, HIAI_OK, "[FaceLandmarkEngine] start init");
    // AI model maneger
    modelManager = std::make_shared<hiai::AIModelManager>();
    hiai::AIModelDescription modelDesc;
    map<string, string> keyValueConfig = KvMap(config);
    if (CheckEmpty(keyValueConfig["model"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    std::string modelPath = keyValueConfig["model"];
    std::set<char> delims { '\\', '/' };
    std::vector<std::string> path = SplitPath(modelPath, delims);
    std::string modelName = path.back();
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] model path %s", modelPath.c_str());
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] model name %s", modelName.c_str());
    modelDesc.set_path(modelPath);
    modelDesc.set_name(modelName);
    // init ai model manager
    hiai::AIStatus ret = modelManager->Init(config, { modelDesc });
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] ai model manager init failed!");
        return HIAI_ERROR;
    }

    // parse other params
    ParamInit(keyValueConfig);

    // get the input and output tensor dim from the model
    kAlignedWidth = ALIGN_UP(kWidth, FACE_LANDMARK_WIDTH_ALIGN);
    kAlignedHeight = ALIGN_UP(kHeight, FACE_LANDMARK_HEIGHT_ALIGN);
    std::vector<hiai::TensorDimension> inputTensorDims;
    std::vector<hiai::TensorDimension> outputTensorDims;
    ret = modelManager->GetModelIOTensorDim(modelName, inputTensorDims, outputTensorDims);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] GetModelIOTensorDim failed!");
        return HIAI_ERROR;
    }

    // dump and check the dim info
    if (DimCheck(inputTensorDims, outputTensorDims) != HIAI_OK) {
        return HIAI_ERROR;
    }

    PreAllocate(inputTensorDims, outputTensorDims);

    /* create the vpc object */
    if (piDvppApiVpc == NULL) {
        ret = CreateDvppApi(piDvppApiVpc);
        if ((ret != HIAI_OK) && (piDvppApiVpc == NULL)) {
            HIAI_ENGINE_LOG(this, HIAI_ERROR, "[FaceLandmarkEngine] fail to intialize vpc api!");
            return HIAI_ERROR;
        }
    }

    // pre allocate vpc output data buffer, different frames will reuse this
    // buffer, so memory copy is needed
    for (int i = 0; i < maxFaceNumPerFrame; i++) {
        uint8_t *outputBuffer = nullptr;
        ret = hiai::HIAIMemory::HIAI_DVPP_DMalloc(kInputSize / kBatchSize, (void *&)outputBuffer);
        if (ret != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] HIAI_DVPP_DMalloc for vpc failed.");
            return HIAI_ERROR;
        }
        outputBufferShared.push_back(std::shared_ptr<uint8_t>(outputBuffer, [](std::uint8_t *data) { hiai::HIAIMemory::HIAI_DVPP_DFree(data); }));
    }
    HIAI_ENGINE_LOG(this, HIAI_OK, "[FaceLandmarkEngine] init success");
    return HIAI_OK;
}

/**
 * @brief destructor
 */
FaceLandmarkEngine::~FaceLandmarkEngine()
{
    /* Destroy vpc object */
    if (piDvppApiVpc != NULL) {
        HIAI_ENGINE_LOG(this, HIAI_OK, "[FaceLandmarkEngine] Destroy vpc api!");
        DestroyDvppApi(piDvppApiVpc);
        piDvppApiVpc = NULL;
    }
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("FaceLandmarkEngine", FaceLandmarkEngine, FACE_LANDMARK_INPUT_SIZE)
{
    if (arg0 == NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] invalid input");
        return HIAI_ERROR;
    }
    std::shared_ptr<DeviceStreamData> dataInput = std::static_pointer_cast<DeviceStreamData>(arg0);

    if (dataInput->detectResult.size() > 0) {
        if (FaceCropResize(dataInput->imgOrigin, dataInput->detectResult, dataInput->face, kWidth, kHeight) !=
            HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] Corp face Image error");
            return HIAI_ERROR;
        }
    }

    dataInput->imgOrigin.buf.data.reset();
    dataInput->imgOrigin.buf.len_of_byte = 0;

    // iterate the whole faces of one frame, waiting for enough number of faces to fill the batch
    // the last batch may not be filled fully, it is OK as the padding will be done before inference
    bool last_face = false;
    for (int i = 0; i < dataInput->face.size(); i++) {
        faceObjectQueue.push_back(&(dataInput->face[i]));
        last_face = (i == dataInput->face.size() - 1);
        // in register mode, we need not to fill the batch
        if ((faceObjectQueue.size() < kBatchSize) && (!last_face)) {
            continue;
        }
        if (Infer(faceObjectQueue) != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] infer error!");
            faceObjectQueue.clear();
            return HIAI_ERROR;
        }
        faceObjectQueue.clear();
    }

    // send data to Warpaffine
    if (SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(dataInput)) != HIAI_OK) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceLandmarkEngine] send data failed!");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

/**
 * @brief do model inference, support batchsize padding
 * @[in]: faceObjectQueue: vector of faceobject
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceLandmarkEngine::Infer(std::vector<FaceObject *> &faceObjectQueue)
{
    int ret = 0;
    // copy data to input buffer
    uint8_t *dataBufferPtr = inputDataBuffer.first.get();
    for (int i = 0; i < faceObjectQueue.size(); i++) {
        FaceObject *faceObjectPtr = faceObjectQueue[i];
        ret = memcpy_s(dataBufferPtr, kInputSize / kBatchSize, faceObjectPtr->imgCroped.buf.data.get(),
                       faceObjectPtr->imgCroped.buf.len_of_byte);
        if (ret != 0) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Data Buffer memory copy failed");
            return HIAI_ERROR;
        }
        dataBufferPtr += kInputSize / kBatchSize;
    }
    // pad to kBatchSize
    if (faceObjectQueue.size() < kBatchSize) {
        int padNum = kBatchSize - faceObjectQueue.size();
        ret = memset_s(dataBufferPtr, kInputSize / kBatchSize * padNum, static_cast<char>(0),
                       kInputSize / kBatchSize * padNum);
        if (ret != 0) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Data Buffer memory set failed");
            return HIAI_ERROR;
        }
    }

    // inference
    hiai::AIContext aiContext;
    ret = modelManager->Process(aiContext, inputTensorVec, outputTensorVec, 0);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "AI Model Manager Process failed");
        return HIAI_ERROR;
    }

    // post process
    shared_ptr<hiai::AINeuralNetworkBuffer> tensorResults =
        std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputTensorVec[0]);
    int featureVecterSize = tensorResults->GetSize() / kBatchSize;
    uint8_t *resPtr = (uint8_t *)tensorResults->GetBuffer();
    if (resPtr == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Get result of face landmark process failed");
        return HIAI_ERROR;
    }
    for (int32_t i = 0; i < faceObjectQueue.size(); i++) {
        std::shared_ptr<u_int8_t> outBuffer(new uint8_t[featureVecterSize]);
        ret = memcpy_s(outBuffer.get(), featureVecterSize, resPtr, featureVecterSize);
        if (ret != 0) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "outBuffer memory set failed");
            return HIAI_ERROR;
        }
        faceObjectQueue[i]->landmarks.data = outBuffer;
        faceObjectQueue[i]->landmarks.len_of_byte = featureVecterSize;
        resPtr += featureVecterSize;
    }
    return HIAI_OK;
}

/**
 * @brief: crop and resize faces from one frame
 * @[in]: imgOrigin: original frame data, detectResult: detecction result, face: one face object, width: width of target
 * face size, height: height of target face size
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceLandmarkEngine::FaceCropResize(ImageInfo &imgOrigin, std::vector<DetectInfo> &detectResult,
                                                std::vector<FaceObject> &face, int width, int height)
{
    /* Construct VPC input parameter */
    std::shared_ptr<VpcUserImageConfigure> imageConfigure(new VpcUserImageConfigure);
    imageConfigure->bareDataAddr = imgOrigin.buf.data.get();
    imageConfigure->bareDataBufferSize = imgOrigin.buf.len_of_byte;
    imageConfigure->isCompressData = false;
    imageConfigure->widthStride = imgOrigin.widthAligned;
    imageConfigure->heightStride = imgOrigin.heightAligned;
    imageConfigure->inputFormat = INPUT_YUV420_SEMI_PLANNER_UV;
    imageConfigure->outputFormat = OUTPUT_YUV420SP_UV;
    imageConfigure->yuvSumEnable = false;
    imageConfigure->cmdListBufferAddr = nullptr;
    imageConfigure->cmdListBufferSize = 0;
    VpcUserRoiConfigure *lastRoi;
    std::vector<VpcUserRoiConfigure> roiConfigures(detectResult.size());
    const int MOD_2 = 2;
    for (int32_t i = 0; i < detectResult.size(); i++) {
        VpcUserRoiConfigure *roiConfigure = &(roiConfigures[i]);
        roiConfigure->next = nullptr;
        // set map area: face
        VpcUserRoiInputConfigure *inputConfigure = &(roiConfigure->inputConfigure);
        inputConfigure->cropArea.leftOffset = detectResult[i].location.anchor_lt.x % MOD_2
                                              ? detectResult[i].location.anchor_lt.x - 1
                                              : detectResult[i].location.anchor_lt.x;
        inputConfigure->cropArea.rightOffset = detectResult[i].location.anchor_rb.x % MOD_2
                                               ? detectResult[i].location.anchor_rb.x
                                               : detectResult[i].location.anchor_rb.x - 1;
        inputConfigure->cropArea.upOffset = detectResult[i].location.anchor_lt.y % MOD_2
                                            ? detectResult[i].location.anchor_lt.y - 1
                                            : detectResult[i].location.anchor_lt.y;
        inputConfigure->cropArea.downOffset = detectResult[i].location.anchor_rb.y % MOD_2
                                              ? detectResult[i].location.anchor_rb.y
                                              : detectResult[i].location.anchor_rb.y - 1;

        // Construct output buffer
        VpcUserRoiOutputConfigure *outputConfigure = &(roiConfigure->outputConfigure);
        outputConfigure->widthStride = ALIGN_UP(width, FACE_LANDMARK_WIDTH_ALIGN);     // align to 16
        outputConfigure->heightStride = ALIGN_UP(height, FACE_LANDMARK_HEIGHT_ALIGN);  // align to 2
        outputConfigure->bufferSize =
            outputConfigure->widthStride * outputConfigure->heightStride * 3 / 2;  // yuv420sp_UV
        // we set the outputbuffer to the pre allocated buffer to avoid frequently dvpp allocating
        outputConfigure->addr = outputBufferShared[i].get();

        // set crop area:
        outputConfigure->outputArea.leftOffset = 0;
        outputConfigure->outputArea.rightOffset = (width % MOD_2) ? width : (width - 1);
        outputConfigure->outputArea.upOffset = 0;
        outputConfigure->outputArea.downOffset = (height % MOD_2) ? height : (height - 1);

        if (i == 0) {
            imageConfigure->roiConfigure = roiConfigure;
            lastRoi = roiConfigure;
        } else {
            lastRoi->next = roiConfigure;
            lastRoi = roiConfigure;
        }
    }

    /* process of VPC */
    dvppapi_ctl_msg dvppApiCtlMsg;
    dvppApiCtlMsg.in = static_cast<void *>(imageConfigure.get());
    dvppApiCtlMsg.in_size = sizeof(VpcUserImageConfigure);

    if (DvppCtl(piDvppApiVpc, DVPP_CTL_VPC_PROC, &dvppApiCtlMsg) != 0) {
        while (imageConfigure->roiConfigure != nullptr) {
            HIAI_DVPP_DFree(imageConfigure->roiConfigure->outputConfigure.addr);
            imageConfigure->roiConfigure = imageConfigure->roiConfigure->next;
        }
        return HIAI_ERROR;
    }

    /* Get output image */
    int i = 0;
    while (imageConfigure->roiConfigure != nullptr) {
        FaceObject faceObject;
        faceObject.info = detectResult[i];
        i++;
        faceObject.imgCroped.format = OUTPUT_YUV420SP_UV;
        faceObject.imgCroped.width = imageConfigure->roiConfigure->outputConfigure.outputArea.rightOffset;
        faceObject.imgCroped.height = imageConfigure->roiConfigure->outputConfigure.outputArea.downOffset;
        faceObject.imgCroped.widthAligned = imageConfigure->roiConfigure->outputConfigure.widthStride;
        faceObject.imgCroped.heightAligned = imageConfigure->roiConfigure->outputConfigure.heightStride;
        std::shared_ptr<u_int8_t> outBuffer(new uint8_t[imageConfigure->roiConfigure->outputConfigure.bufferSize]);
        int mcps = memcpy_s(outBuffer.get(), imageConfigure->roiConfigure->outputConfigure.bufferSize,
                            imageConfigure->roiConfigure->outputConfigure.addr,
                            imageConfigure->roiConfigure->outputConfigure.bufferSize);
        if (mcps != 0) {
            return HIAI_ERROR;
        }
        faceObject.imgCroped.buf.data = outBuffer;
        faceObject.imgCroped.buf.len_of_byte = imageConfigure->roiConfigure->outputConfigure.bufferSize;
        face.push_back(faceObject);
        imageConfigure->roiConfigure = imageConfigure->roiConfigure->next;
    }
    return HIAI_OK;
}
