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

#include "MergeEngine.h"
#include <hiaiengine/data_type.h>
#include <hiaiengine/engine.h>
#include <sys/time.h>

/**
 * @brief: init, inherited from hiaiengine lib
 */
HIAI_REGISTER_DATA_TYPE("DeviceStreamData", DeviceStreamData);
HIAI_StatusT MergeEngine::Init(const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& model_desc)
{
    HIAI_ENGINE_LOG(this, HIAI_OK, "MergeEngine start init");
    if (keyFrame == nullptr) {
        keyFrame = std::make_shared<DeviceStreamData>();
    }
    HIAI_ENGINE_LOG(this, HIAI_OK, "MergeEngine init success");
    std::cout << "MergeEngine init success" << std::endl;
    return HIAI_OK;
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("MergeEngine", MergeEngine, MERGE_ENGINE_INPUT_SIZE)
{
    if (arg0 == NULL) {
        return HIAI_ERROR;
    }
    HIAI_StatusT ret = HIAI_OK;
    std::shared_ptr<DeviceStreamData> dataInput = std::static_pointer_cast<DeviceStreamData>(arg0);

    if (dataInput->isKeyFrame == 1) {
        keyFrame = dataInput;
        ret = SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
        if (ret != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MergeEngine] senddata error!");
            return HIAI_ERROR;
        }

        auto it = deviceStreamDataQueue.begin();
        while (it != deviceStreamDataQueue.end()) {
            if ((*it)->trkFrameID == keyFrame->info.frameId) {
                for (int i = 0; i < (*it)->detIdx.size(); i++) {
                    FaceObject faceObject;
                    faceObject.info = (*it)->detectResult[(*it)->detIdx[i]];
                    faceObject.featureVector.data = keyFrame->face[(*it)->trkIdx[i]].featureVector.data;
                    faceObject.featureVector.len_of_byte = keyFrame->face[(*it)->trkIdx[i]].featureVector.len_of_byte;
                    faceObject.imgCroped.buf.len_of_byte = 0;
                    faceObject.imgAffine.buf.len_of_byte = 0;
                    faceObject.landmarks.len_of_byte = 0;
                    (*it)->face.push_back(faceObject);
                }
                ret = SendData(0, "DeviceStreamData", std::static_pointer_cast<void>((*it)));
                if (HIAI_OK != ret) {
                    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MergeEngine] senddata error!");
                }
                it = deviceStreamDataQueue.erase(it);
            } else if ((*it)->trkFrameID < keyFrame->info.frameId) {
                it = deviceStreamDataQueue.erase(it);
            } else {
                it++;
            }
        }
    } else if ((dataInput->isKeyFrame == 0) && (dataInput->isTracked == 0)) {  
        ret = SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
        if (ret != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MergeEngine] senddata error!");
            return HIAI_ERROR;
        }
    } else if ((dataInput->trkFrameID == keyFrame->info.frameId) && (keyFrame->isKeyFrame == 1)) {
        for (int i = 0; i < dataInput->detIdx.size(); i++) {
            FaceObject faceObject;
            faceObject.info = dataInput->detectResult[dataInput->detIdx[i]];
            faceObject.featureVector.data = keyFrame->face[dataInput->trkIdx[i]].featureVector.data;
            faceObject.featureVector.len_of_byte = keyFrame->face[dataInput->trkIdx[i]].featureVector.len_of_byte;
            faceObject.imgCroped.buf.len_of_byte = 0;
            faceObject.imgAffine.buf.len_of_byte = 0;
            faceObject.landmarks.len_of_byte = 0;
            dataInput->face.push_back(faceObject);
        }
        ret = SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(dataInput));
        if (ret != HIAI_OK) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[MergeEngine] senddata error!");
            return HIAI_ERROR;
        }
    } else {
        deviceStreamDataQueue.push_back(dataInput);
    }

    return HIAI_OK;
}
