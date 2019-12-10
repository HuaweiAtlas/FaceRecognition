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

#include "register.h"
#include <fstream>

float L2NormSum(const float* vectorBuf, int size)
{
    float a;
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        a = pow(vectorBuf[i], 2.0f);
        sum += a;
    }

    return sqrt(sum);
}

void L2Normalization(float* vectorBuf, int size)
{
    float a = L2NormSum(vectorBuf, size);
    a = (a > DBL_EPSILON) ? a : DBL_EPSILON;

    for (int i = 0; i < size; i++) {
        vectorBuf[i] = vectorBuf[i] / a;
    }
}

// the bigger the better
float CalSimilarity(const float* x, const float* y, int size)
{
    int i;
    float a = 0.0f;
    float b = 0.0f;
    float similarity = 0.0f;

    for (i = 0; i < size; i++) {
        similarity += x[i] * y[i];
    }
    return similarity;
}

int FeaturelibRead(std::map<int, std::shared_ptr<float>>& featureLib, std::string& featureLibPath, uint32_t featureLen,
    uint32_t featureNum)
{
    if (featureLen <= 0 || featureNum <= 0) {
        std::cout << "feature_len or feature_num is invalid, please check! " << std::endl;
        return -1;
    }

    std::ifstream featurelibFile(featureLibPath, std::ios::in);
    if (!featurelibFile) {
        std::cout << "read Featurelib failed" << std::endl;
        return -1;
    }

    std::streampos pos = featurelibFile.tellg();
    featurelibFile.seekg(0, std::ios::end);
    uint64_t totoalFeatureByteLen = featurelibFile.tellg();
    featurelibFile.seekg(pos);
    if (totoalFeatureByteLen != featureLen * featureNum * sizeof(float)) {
        std::cout << "the len of featurelib does not match the configured len" << std::endl;
        featurelibFile.close();
        return -1;
    }

    // read all the features in the bin file
    char* featureLibRaw = new char[featureNum * featureLen * sizeof(float)];
    if (featureLibRaw == nullptr) {
        std::cout << "new featureLibRaw failded!" << std::endl;
        featurelibFile.close();
        return -1;
    }
    featurelibFile.read(featureLibRaw, featureNum * featureLen * sizeof(float));
    std::cout << "read raw Featurelib from file SUCCESS" << std::endl;
    featurelibFile.close();

    for (int i = 0; i < featureNum; i++) {
        std::shared_ptr<float> feature(new float[featureLen]);
        int ret = memcpy_s(feature.get(),
            featureLen * sizeof(float),
            featureLibRaw + i * featureLen * sizeof(float),
            featureLen * sizeof(float));
        if (ret != 0) {
            delete[] featureLibRaw;
            featureLibRaw = nullptr;
            std::cout << "feature copy failed." << std::endl;
            return -1;
        }
        L2Normalization(feature.get(), featureLen);
        featureLib[i] = feature;
    }

    delete[] featureLibRaw;
    featureLibRaw = nullptr;
    std::cout << "Featurelib create SUCCESS" << std::endl;
    return 0;
}

int SearchFeatureLib(
    std::map<int, std::shared_ptr<float>>& featureLib, float* featureVector, int featureSize, float& similarity)
{
    L2Normalization(featureVector, featureSize);
    float maxSimilarity = 0.0f;
    int objectId = -1;
    auto iter = featureLib.begin();
    while (iter != featureLib.end()) {
        float simi = CalSimilarity(featureVector, iter->second.get(), featureSize);
        if (simi > maxSimilarity) {
            maxSimilarity = simi;
            objectId = iter->first;
        }
        iter++;
    }

    similarity = maxSimilarity;
    const float SIMILAR_THREHOLD = 0.8;
    if (maxSimilarity >= SIMILAR_THREHOLD) {
        return objectId;
    } else {
        return -1;
    }
}
