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

#include "DistributionEngine.h"
#include <hiaiengine/api.h>
#include <hiaiengine/data_type.h>

/**
 * @brief: init, inherited from hiaiengine lib
 */
HIAI_StatusT DistributionEngine::Init(
    const hiai::AIConfig& config, const std::vector<hiai::AIModelDescription>& modelDesc)
{
    HIAI_ENGINE_LOG(this, HIAI_OK, "[DistributionEngine] init success");
    return HIAI_OK;
}

/**
 * @ingroup hiaiengine
 * @brief HIAI_DEFINE_PROCESS : Realize the port input/output processing
 * @[in]: DT_INPUT_SIZE: num of input ports
 */
HIAI_IMPL_ENGINE_PROCESS("DistributionEngine", DistributionEngine, DISTRIBUTION_INPUT_SIZE)
{
    if (arg0 == NULL) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DistributionEngine] invalid input");
        return HIAI_ERROR;
    }
    std::shared_ptr<DeviceStreamData> detectInput = std::static_pointer_cast<DeviceStreamData>(arg0);

    // send data to FaceLarndmark engine according to the face num, this trick
    // may reduce inference time to some degree
    HIAI_StatusT ret = HIAI_OK;
    if (detectInput->detectResult.size() <= 1) {
        ret = SendData(0, "DeviceStreamData", std::static_pointer_cast<void>(detectInput));
        if (HIAI_OK != ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DistributionEngine] send data error, face num =1!");
            return HIAI_ERROR;
        }
    } else if (detectInput->detectResult.size() <= 4) {
        ret = SendData(1, "DeviceStreamData", std::static_pointer_cast<void>(detectInput));
        if (HIAI_OK != ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DistributionEngine] send data error, face num >1 and <=4!");
            return HIAI_ERROR;
        }
    } else {
        ret = SendData(2, "DeviceStreamData", std::static_pointer_cast<void>(detectInput));
        if (HIAI_OK != ret) {
            HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "[DistributionEngine] send data error, face num >4!");
            return HIAI_ERROR;
        }
    }

    return HIAI_OK;
}
