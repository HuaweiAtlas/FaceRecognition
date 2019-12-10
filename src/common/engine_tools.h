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
#ifndef ATLASFACEDEMO_ENGINE_TOOLS_H
#define ATLASFACEDEMO_ENGINE_TOOLS_H

#include "hiaiengine/ai_model_manager.h"
#include "hiaiengine/c_graph.h"
#include "hiaiengine/engine.h"
#include "hiaiengine/log.h"
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>
using hiai::AIModelDescription;
using hiai::AIModelManager;
using hiai::AINeuralNetworkBuffer;
using hiai::AITensorDescription;
using hiai::AITensorFactory;
using hiai::IAITensor;
using hiai::TensorDimension;
using std::map;
using std::string;

template <typename T>
inline std::vector<int> SortIndexVector(std::vector<T>& v)
{
    std::vector<int> idx(v.size());
    for (int i = 0; i < v.size(); i++) {
        idx[i] = i;
    }
    std::sort(idx.begin(), idx.end(), [&v](int i, int j) { return v[i] > v[j]; });
    return idx;
}

inline map<string, string> KvMap(const hiai::AIConfig& config)
{
    map<string, string> kv;
    for (int index = 0; index < config.items_size(); ++index) {
        const ::hiai::AIConfigItem& item = config.items(index);
        kv.insert(std::make_pair(item.name(), item.value()));
    }
    return std::move(kv);
}

inline HIAI_StatusT CheckEmpty(const string& value)
{
    if (value.empty()) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "Check empty value!");
        return HIAI_ERROR;
    }
    return HIAI_OK;
}

inline HIAI_StatusT CheckInputParam(std::map<string, string>& keyValueConfig, uint32_t& kBatchSize, uint32_t& kChannel,
                                    uint32_t& kHeight, uint32_t& kWidth)
{
    // batch size
    if (CheckEmpty(keyValueConfig["batch_size"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    if (!(std::stringstream(keyValueConfig["batch_size"]) >> kBatchSize)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "batch_size is not digit,please check!");
        return HIAI_ERROR;
    }

    // channel number
    if (CheckEmpty(keyValueConfig["input_channel"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    if (!(std::stringstream(keyValueConfig["input_channel"]) >> kChannel)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "input_channel is not digit,please check!");
        return HIAI_ERROR;
    }

    // height
    if (CheckEmpty(keyValueConfig["input_height"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    if (!(std::stringstream(keyValueConfig["input_height"]) >> kHeight)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "input_height is not digit,please check!");
        return HIAI_ERROR;
    }

    // width
    if (CheckEmpty(keyValueConfig["input_width"]) != HIAI_OK) {
        return HIAI_ERROR;
    }
    if (!(std::stringstream(keyValueConfig["input_width"]) >> kWidth)) {
        HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "input_width is not digit,please check!");
        return HIAI_ERROR;
    }

    return HIAI_OK;
}

static std::vector<std::string> SplitPath(const std::string& str, const std::set<char> delimiters)
{
    std::vector<std::string> result;
    char const* pch = str.c_str();
    char const* start = pch;
    for (; *pch; ++pch) {
        if (delimiters.find(*pch) != delimiters.end()) {
            if (start != pch) {
                std::string str(start, pch);
                result.push_back(str);
            } else {
                result.push_back("");
            }
            start = pch + 1;
        }
    }
    result.push_back(start);
    return result;
}

inline void LogDumpDims(const hiai::TensorDimension& dims)
{
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "tensor name %s", dims.name.c_str());
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "tensor dim %d", dims.n);
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "tensor dim %d", dims.c);
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "tensor dim %d", dims.h);
    HIAI_ENGINE_LOG(HIAI_IDE_ERROR, "tensor dim %d", dims.w);
}

#endif  // ATLASFACEDEMO_ENGINE_TOOLS_H
