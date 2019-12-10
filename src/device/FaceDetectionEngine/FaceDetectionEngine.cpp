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
#include "FaceDetectionEngine.h"
#include "engine_tools.h"
#include "hiaiengine/ai_memory.h"
#include "hiaiengine/c_graph.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/log.h"
#include <map>
#include <memory>
#include <set>
#include <string>
#include <sys/time.h>
#include <vector>

#define FACE_DETECT_WIDTH_ALIGN (16)
#define FACE_DETECT_HEIGHT_ALIGN (2)

HIAI_StatusT FaceDetectionEngine::ParamInit(std::map<string, string>& keyValueConfig)
{
    if (CheckInputParam(keyValueConfig, kBatchSize, kChannel, kHeight, kWidth) != HIAI_OK) {
        return HIAI_ERROR;
    }
    inputArgQueue.reserve(kBatchSize);

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

HIAI_StatusT FaceDetectionEngine::PreAllocate(
    std::vector<hiai::TensorDimension>& inputTensorDims, std::vector<hiai::TensorDimension>& outputTensorDims)
{
    // pre allocate inference input data buffer
    kInputSize = kBatchSize * kChannel * kAlignedHeight * kAlignedWidth / 2;  // data format is yuv420sp

    if (kInputSize != inputTensorDims[0].size) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR,
            "[FaceDetectionEngine] inputSize != "
            "inputTensorDims[0].size (%d vs. %d)",
            kInputSize,
            inputTensorDims[0].size);
        return HIAI_ERROR;
    }
    std::shared_ptr<uint8_t> inPtr((uint8_t*)HIAI_DVPP_DMalloc(kInputSize), HIAI_DVPP_DFree);
    inputDataBuffer = std::make_pair(inPtr, kInputSize);
    hiai::AITensorDescription tensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
    std::shared_ptr<hiai::IAITensor> inputTensor =
        hiai::AITensorFactory::GetInstance()->CreateTensor(tensorDesc, (void*)inPtr.get(), kInputSize);
    inputTensorVec.push_back(inputTensor);

    // pre allocate inference output data buffer
    for (uint32_t index = 0; index < outputTensorDims.size(); index++) {
        hiai::AITensorDescription outputTensorDesc = hiai::AINeuralNetworkBuffer::GetDescription();
        HIAI_ENGINE_LOG(
            HIAI_IDE_ERROR, "[FaceDetectionEngine] outputTensorDims[index].size %d", outputTensorDims[index].size);
        uint8_t* buf = (uint8_t*)HIAI_DVPP_DMalloc(outputTensorDims[index].size);
        if (buf == nullptr) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] HIAI_DVPP_DMalloc failed.");
            return HIAI_ERROR;
        }
        outputDataBuffer.push_back(std::shared_ptr<uint8_t>(buf, HIAI_DVPP_DFree));
        std::shared_ptr<hiai::IAITensor> outputTensor =
            hiai::AITensorFactory::GetInstance()->CreateTensor(outputTensorDesc, buf, outputTensorDims[index].size);
        shared_ptr<hiai::AINeuralNetworkBuffer> nnTensor =
            static_pointer_cast<hiai::AINeuralNetworkBuffer>(outputTensor);
        nnTensor->SetName(outputTensorDims[index].name);
        outputTensorVec.push_back(outputTensor);
    }
}

/**
 * @brief: dump and check the dim info
 * @in: inputTensorDims, input dims out of model
 *      outputTensorDims, output dims out of model
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceDetectionEngine::DimCheck(
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
            "[FaceDetectionEngine] input dims of the "
            "om model are not identical to the ones "
            "defined in graph config");
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

/**
 * @brief: init, inherited from hiaiengine lib
 */
HIAI_StatusT FaceDetectionEngine::Init(
    const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] start init!");
    // AI model maneger
    modelManager = std::make_shared<hiai::AIModelManager>();
    hiai::AIModelDescription modelDesc;
    std::map<string, string> keyValueConfig = KvMap(config);
    if (HIAI_OK != CheckEmpty(keyValueConfig["model"])) {
        return HIAI_ERROR;
    }
    std::string modelPath = keyValueConfig["model"];
    std::set<char> delims{'\\', '/'};
    std::vector<std::string> path = SplitPath(modelPath, delims);
    std::string modelName = path.back();
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] model path %s", modelPath.c_str());
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] model name %s", modelName.c_str());
    modelDesc.set_path(modelPath);
    modelDesc.set_name(modelName);
    // init ai model manager
    HIAI_StatusT ret = modelManager->Init(config, {modelDesc});
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] AI model manager init failed!");
        return HIAI_ERROR;
    }

    ParamInit(keyValueConfig);

    // get the input and output tensor dim from the model
    kAlignedWidth = ALIGN_UP(kWidth, FACE_DETECT_WIDTH_ALIGN);
    kAlignedHeight = ALIGN_UP(kHeight, FACE_DETECT_HEIGHT_ALIGN);
    std::vector<hiai::TensorDimension> inputTensorDims;
    std::vector<hiai::TensorDimension> outputTensorDims;
    ret = modelManager->GetModelIOTensorDim(modelName, inputTensorDims, outputTensorDims);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] GetModelIOTensorDim failed!");
        return HIAI_ERROR;
    }

    if (HIAI_OK != DimCheck(inputTensorDims, outputTensorDims)) {
        return HIAI_ERROR;
    }

    PreAllocate(inputTensorDims, outputTensorDims);

    // init the yolo poster process
    mYolo = new CYolo;
    int retYolo = mYolo->Init();
    if (retYolo == -1) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] YOLO init ERROR!");
        return HIAI_ERROR;
    }

    // omp optimize
    omp_set_num_threads(4);

    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] end init!");
    return HIAI_OK;
}

/**
 * @brief: destroy the yolo poster process
 */
FaceDetectionEngine::~FaceDetectionEngine()
{
    delete mYolo;
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("FaceDetectionEngine", FaceDetectionEngine, DT_INPUT_SIZE)
{
    if (arg0 == nullptr) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] invalid input");
        return HIAI_ERROR;
    }

    std::shared_ptr<DeviceStreamData> dataInput = std::static_pointer_cast<DeviceStreamData>(arg0);
    inputArgQueue.push_back(dataInput);

    // waiting for data to fill the batchsize
    // when registering, DO NOT FILL
    if (inputArgQueue.size() < kBatchSize) {
        return HIAI_OK;
    }
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    uint8_t* dataBufferPtr = inputDataBuffer.first.get();
    // the image resizing has been done by VPC of each channel, need to do memory copy to fill the batch
    // free the memory of detectImg, because it will never be used
    for (int i = 0; i < inputArgQueue.size(); i++) {
        dataInput = inputArgQueue[i];
        int mcps = memcpy_s(dataBufferPtr,
            kInputSize / kBatchSize,
            dataInput->detectImg.buf.data.get(),
            dataInput->detectImg.buf.len_of_byte);
        if (mcps != 0) {
            inputArgQueue.clear();
            return HIAI_ERROR;
        }
        dataBufferPtr += kInputSize / kBatchSize;
        dataInput->detectImg.buf.data.reset();
        dataInput->detectImg.buf.len_of_byte = 0;
    }

    // inference
    hiai::AIContext aiContext;
    HIAI_StatusT ret = modelManager->Process(aiContext, inputTensorVec, outputTensorVec, 0);
    if (ret != hiai::SUCCESS) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] AI Model Manager Process failed");
        inputArgQueue.clear();
        return HIAI_ERROR;
    }
    PostProcessDetectionYoloV3Tiny();
    inputArgQueue.clear();
    gettimeofday(&t2, NULL);
    const float TIME_UNIT = 1000000.0;
    timeComsumed = timeComsumed + t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) / TIME_UNIT;
    runCount++;
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, " [FaceDetectionEngine] average e2e time=%8f", timeComsumed / runCount);

    return HIAI_OK;
}

/**
 * @brief select top faces from the whole frame
 * @in: detectResult: the detect result from the detection model inference
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceDetectionEngine::FaceSelection(std::vector<DetectInfo>& detectResult)
{
    std::vector<DetectInfo> detectResultCopy(detectResult);
    std::vector<float> metric;
    for (auto i = detectResult.begin(); i != detectResult.end(); i++) {
        float area = ((*i).location.anchor_rb.x - (*i).location.anchor_lt.x) *
                     ((*i).location.anchor_rb.y - (*i).location.anchor_lt.y);
        metric.push_back(area * ((*i).confidence));
    }
    auto idx = SortIndexVector(metric);  // sort by descend order and return the index
    detectResult.clear();
    for (auto i = idx.begin(); (i != idx.begin() + maxFaceNumPerFrame) && (i != idx.end()); i++) {
        detectResult.push_back(detectResultCopy[*i]);
    }
    return HIAI_OK;
}

/**
 * @brief: senddata according to channel id and mode
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceDetectionEngine::SelectiveSendData(std::shared_ptr<DeviceStreamData> deviceStreamData)
{
    // send the data to SORT engine according to the  channel ID
    return SendData(
        deviceStreamData->info.channelId, "DeviceStreamData", std::static_pointer_cast<void>(deviceStreamData));
}

/**
 * @brief: yolo poster porcess
 * @return: HIAI_StatusT
 */
HIAI_StatusT FaceDetectionEngine::PostProcessDetectionYoloV3Tiny(void)
{
    uint32_t singleSize = 0;
    for (uint32_t j = 0; j < outputTensorVec.size(); j++) {
        std::shared_ptr<hiai::AISimpleTensor> resultTensor =
            std::static_pointer_cast<hiai::AISimpleTensor>(outputTensorVec[j]);
        singleSize += resultTensor->GetSize() / kBatchSize;
    }

    uint8_t* singleBufPtr = new uint8_t[singleSize];
    for (int32_t j = 0; j < inputArgQueue.size(); j++) {
        std::shared_ptr<DeviceStreamData> deviceStreamData = inputArgQueue[j];
        int32_t idx = 0;
        for (int32_t i = 0; i < outputTensorVec.size(); i++) {
            std::shared_ptr<hiai::AISimpleTensor> resultTensor =
                std::static_pointer_cast<hiai::AISimpleTensor>(outputTensorVec[i]);
            int32_t offset = j * resultTensor->GetSize() / kBatchSize;
            int mcps = memcpy_s(singleBufPtr + idx,
                resultTensor->GetSize() / kBatchSize,
                (uint8_t*)resultTensor->GetBuffer() + offset,
                resultTensor->GetSize() / kBatchSize);
            if (mcps != 0) {
                delete[] singleBufPtr;
                return HIAI_ERROR;
            }
            idx += resultTensor->GetSize() / kBatchSize;
        }

        int numDetected = mYolo->process((float*)singleBufPtr,
            singleSize,
            deviceStreamData->imgOrigin.width,
            deviceStreamData->imgOrigin.height,
            deviceStreamData->detectResult);

        if (numDetected > 0) {
            FaceSelection(deviceStreamData->detectResult);  // select the top k faces from one frame
        }

        if (SelectiveSendData(deviceStreamData) != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[FaceDetectionEngine] senddata error!");
            delete[] singleBufPtr;
            return HIAI_ERROR;
        }
    }
    delete[] singleBufPtr;

    return HIAI_OK;
}
