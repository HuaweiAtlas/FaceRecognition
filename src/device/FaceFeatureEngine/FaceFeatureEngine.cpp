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

#include "FaceFeatureEngine.h"
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
#define FACE_FEATURE_WIDTH_ALIGN (16)
#define FACE_FEATURE_HEIGHT_ALIGN (2)
HIAI_REGISTER_SERIALIZE_FUNC("RawDataBufferHigh", RawDataBufferHigh, GetTransSearPtr, GetTransDearPtr);
HIAI_REGISTER_DATA_TYPE("DeviceStreamData", DeviceStreamData);

HIAI_StatusT FaceFeatureEngine::ParamInit(std::map<string, string>& keyValueConfig)
{
    if (CheckInputParam(keyValueConfig, kBatchSize, kChannel, kHeight, kWidth) != HIAI_OK) {
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

HIAI_StatusT FaceFeatureEngine::DimCheck(
    std::vector<hiai::TensorDimension>& inputTensorDims, std::vector<hiai::TensorDimension>& outputTensorDims)
{
    for (auto& dims : inputTensorDims) {
        LogDumpDims(dims);
    }
    for (auto& dims : outputTensorDims) {
        LogDumpDims(dims);
    }
    if ((inputTensorDims[0].n != kBatchSize) || (inputTensorDims[0].c != kChannel) ||
        (inputTensorDims[0].h != kHeight) || (inputTensorDims[0].w != kWidth)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
            "[FaceFeatureEngine] input dims of the "
            "om model are not identical to the ones "
            "defined in graph config");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

HIAI_StatusT FaceFeatureEngine::PreAllocate(
    std::vector<hiai::TensorDimension>& inputTensorDims, std::vector<hiai::TensorDimension>& outputTensorDims)
{
    // pre allocate inference input data buffer
    kInputSize = kBatchSize * kChannel * kAlignedHeight * kAlignedWidth;  // data format: rgb888

    std::shared_ptr<uint8_t> inPtr((uint8_t*)HIAI_DVPP_DMalloc(kInputSize), HIAI_DVPP_DFree);
    inputDataBuffer = std::make_pair(inPtr, kInputSize);
    hiai::AITensorDescription tensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
    std::shared_ptr<hiai::IAITensor> inputTensor =
        hiai::AITensorFactory::GetInstance()->CreateTensor(tensorDesc, (void*)inPtr.get(), kInputSize);
    inputTensorVec.push_back(inputTensor);

    // pre allocate model inference output data buffer
    for (uint32_t index = 0; index < outputTensorDims.size(); index++) {
        hiai::AITensorDescription outputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
        HIAI_ENGINE_LOG(
            HIAI_IDE_ERROR, "[FaceFeatureEngine] outputTensorDims[index].size %d", outputTensorDims[index].size);
        uint8_t* buf = (uint8_t*)HIAI_DVPP_DMalloc(outputTensorDims[index].size);
        if (buf == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] HIAI_DVPP_DMalloc failed.");
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
HIAI_StatusT FaceFeatureEngine::Init(
    const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    HIAI_ENGINE_LOG(this, HIAI_OK, "[FaceFeatureEngine] start Init");

    // AI model maneger
    modelManager = std::make_shared<hiai::AIModelManager>();
    hiai::AIModelDescription modelDesc;
    std::map<string, string> keyValueConfig = KvMap(config);
    if (CheckEmpty(keyValueConfig["model"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    std::string modelPath = keyValueConfig["model"];
    std::set<char> delims{'\\', '/'};
    std::vector<std::string> path = SplitPath(modelPath, delims);
    std::string modelName = path.back();
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] model path %s", modelPath.c_str());
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] model name %s", modelName.c_str());
    modelDesc.set_path(modelPath);
    modelDesc.set_name(modelName);

    // init ai model manager
    hiai::AIStatus ret = modelManager->Init(config, {modelDesc});
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] AI model manager init failed!");
        return HIAI_ERROR;
    }

    // parse other params
    ParamInit(keyValueConfig);

    // get the input and output tensor dim from the model
    kAlignedWidth = ALIGN_UP(kWidth, FACE_FEATURE_WIDTH_ALIGN);
    kAlignedHeight = ALIGN_UP(kHeight, FACE_FEATURE_HEIGHT_ALIGN);
    std::vector<hiai::TensorDimension> inputTensorDims;
    std::vector<hiai::TensorDimension> outputTensorDims;
    ret = modelManager->GetModelIOTensorDim(modelName, inputTensorDims, outputTensorDims);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] GetModelIOTensorDim failed!");
        return HIAI_ERROR;
    }

    // dump and check the dim info
    if (DimCheck(inputTensorDims, outputTensorDims) != HIAI_OK) {
        return HIAI_ERROR;
    }

    PreAllocate(inputTensorDims, outputTensorDims);

    HIAI_ENGINE_LOG(this, HIAI_OK, "[FaceFeatureEngine] init success");
    return HIAI_OK;
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("FaceFeatureEngine", FaceFeatureEngine, FACE_FEATURE_INPUT_SIZE)
{

    HIAI_StatusT ret = HIAI_OK;
    if (arg0 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] invalid input");
        return HIAI_ERROR;
    }

    std::shared_ptr<DeviceStreamData> dataInput = std::static_pointer_cast<DeviceStreamData>(arg0);
    // waiting for batch data
    bool last_face = false;
    for (int i = 0; i < dataInput->face.size(); i++) {
        faceObjectQueue.push_back(&(dataInput->face[i]));
        last_face = (i == dataInput->face.size() - 1);
        if ((faceObjectQueue.size() < kBatchSize) && (!last_face)) {
            continue;
        }
        ret = Infer(faceObjectQueue);
        if (ret != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceFeatureEngine] infer error!");
            faceObjectQueue.clear();
            return HIAI_ERROR;
        }
        faceObjectQueue.clear();
    }

    // free unused data before sending to host
    for (auto it = dataInput->face.begin(); it != dataInput->face.end(); it++) {
        (*it).imgAffine.buf.data.reset();
        (*it).imgAffine.buf.len_of_byte = 0;
        (*it).landmarks.len_of_byte = 0;
    }

    ret = SendData(dataInput->info.channelId, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
    if (ret != HIAI_OK) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "FaceFeatureEngine send output error!");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

/**
 * @brief do model inference, support batchsize padding
 * @[in]: faceObjectQueue: vector of faceobject
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceFeatureEngine::Infer(std::vector<FaceObject*>& faceObjectQueue)
{
    int ret = 0;
    // copy data to input buffer
    uint8_t* dataBufferPtr = inputDataBuffer.first.get();
    for (int i = 0; i < faceObjectQueue.size(); i++) {
        FaceObject* faceObjectPtr = faceObjectQueue[i];
        ret = memcpy_s(dataBufferPtr,
            kInputSize / kBatchSize,
            faceObjectPtr->imgAffine.buf.data.get(),
            faceObjectPtr->imgAffine.buf.len_of_byte);
        if (ret != 0) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "AI Model Manager memory copy failed");
            return HIAI_ERROR;
        }
        dataBufferPtr += kInputSize / kBatchSize;
    }
    // pad to kBatchSize
    if (faceObjectQueue.size() < kBatchSize) {
        int padNum = kBatchSize - faceObjectQueue.size();
        ret = memset_s(
            dataBufferPtr, kInputSize / kBatchSize * padNum, static_cast<char>(0), kInputSize / kBatchSize * padNum);
        if (ret != 0) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "AI Model Manager memory set failed");
            return HIAI_ERROR;
        }
    }

    hiai::AIContext aiContext;
    ret = modelManager->Process(aiContext, inputTensorVec, outputTensorVec, 0);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "AI Model Manager Process failed");
        return HIAI_ERROR;
    }

    shared_ptr<hiai::AINeuralNetworkBuffer> tensorResults =
        std::static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputTensorVec[0]);
    int featureVecterSize = tensorResults->GetSize() / kBatchSize;
    uint8_t* resPtr = (uint8_t*)tensorResults->GetBuffer();
    if (resPtr == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Get result of face feature process failed");
        return HIAI_ERROR;
    }
    for (int32_t i = 0; i < faceObjectQueue.size(); i++) {
        std::shared_ptr<uint8_t> outBuffer(new uint8_t[featureVecterSize]);
        ret = memcpy_s(outBuffer.get(), featureVecterSize, resPtr, featureVecterSize);
        if (ret != 0) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "AI Model Manager memory copy failed");
            return HIAI_ERROR;
        }
        faceObjectQueue[i]->featureVector.data = outBuffer;
        faceObjectQueue[i]->featureVector.len_of_byte = featureVecterSize;
        resPtr += featureVecterSize;
    }
    return HIAI_OK;
}
