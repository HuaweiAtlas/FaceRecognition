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
#include "Hungarian.h"
#include <memory.h>
#define INF (0x3f3f3f3f)
#define VISITED (1)

void HungarianInit(HungarianHandle* handle, std::vector<std::vector<int>>& cost, int rows, int cols)
{
    int i, j, value;
    if (rows > cols) {
        handle->transpose = true;
        handle->cols = rows;
        handle->rows = cols;
        handle->resX = handle->yMatch;
        handle->resY = handle->xMatch;
    } else {
        handle->transpose = false;
        handle->rows = rows;
        handle->cols = cols;
        handle->resX = handle->xMatch;
        handle->resY = handle->yMatch;
    }

    for (i = 0; i < handle->rows; ++i) {
        handle->xValue[i] = 0;
        handle->xMatch[i] = -1;
        for (j = 0; j < handle->cols; ++j) {
            if (handle->transpose) {
                value = cost[j][i];
            } else {
                value = cost[i][j];
            }
            handle->adjMat[i][j] = value;
            if (handle->xValue[i] < value) {
                handle->xValue[i] = value;
            }
        }
    }

    for (i = 0; i < handle->cols; ++i) {
        handle->yValue[i] = 0;
        handle->yMatch[i] = -1;
    }
}

bool Match(HungarianHandle* handle, int id)
{
    int j, delta;
    handle->xVisit[id] = VISITED;
    for (j = 0; j < handle->cols; ++j) {
        if (handle->yVisit[j] != VISITED) {
            delta = handle->xValue[id] + handle->yValue[j] - handle->adjMat[id][j];
            if (delta == 0) {
                handle->yVisit[j] = VISITED;
                if (handle->yMatch[j] == -1 || Match(handle, handle->yMatch[j])) {
                    handle->yMatch[j] = id;
                    handle->xMatch[id] = j;
                    return true;
                }
            } else if (delta < handle->slack[j]) {
                handle->slack[j] = delta;
            }
        }
    }
    return false;
}

int HungarianSolve(HungarianHandle* handle, std::vector<std::vector<int>>& cost, int rows, int cols)
{
    HungarianInit(handle, cost, rows, cols);
    int ret = 0;
    int i, j, k, delta;
    for (i = 0; i < handle->rows; ++i) {
        while (true) {
            ret = memset_s(handle->xVisit, MAXOBJNUM * sizeof(int), static_cast<char>(0), handle->rows * sizeof(int));
            if (ret != 0) {
                HIAI_ENGINE_LOG(HIAI_ERROR, "[SORTEngine] hungarian xVisit memory set failed!");
                return -1;
            }
            ret = memset_s(handle->yVisit, MAXOBJNUM * sizeof(int), static_cast<char>(0), handle->cols * sizeof(int));
            if (ret != 0) {
                HIAI_ENGINE_LOG(HIAI_ERROR, "[SORTEngine] hungarian yVisit memory set failed!");
                return -1;
            }
            for (j = 0; j < handle->cols; ++j) {
                handle->slack[j] = INF;
            }
            if (Match(handle, i)) {
                break;
            }
            delta = INF;
            for (j = 0; j < handle->cols; ++j) {
                if (handle->yVisit[j] != VISITED && delta > handle->slack[j]) {
                    delta = handle->slack[j];
                }
            }
            if (delta == INF) {
                HIAI_ENGINE_LOG(HIAI_ERROR, "[Track_hungarian] Hungarian solve is invalid!");
                return -1;
            }
            for (j = 0; j < handle->rows; ++j) {
                if (handle->xVisit[j] == VISITED) {
                    handle->xValue[j] -= delta;
                }
            }
            for (j = 0; j < handle->cols; ++j) {
                if (handle->yVisit[j] == VISITED) {
                    handle->yValue[j] += delta;
                }
            }
        }
    }
    return handle->rows;
}