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
#ifndef ATLASFACEDEMO_HUNGARIAN_H
#define ATLASFACEDEMO_HUNGARIAN_H
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include <vector>
#define MAXOBJNUM (100)

typedef struct HungarianHandle {
    int rows;   // 行数;
    int cols;   // 列数;
    int* resX;  //输出的行匹配结果, 匹配成功，输出为ID，失败输出-1
    int* resY;  //输出的列匹配结果
    bool transpose;
    int adjMat[MAXOBJNUM][MAXOBJNUM];  // 处理后的邻接矩阵(row <= col);
    int xMatch[MAXOBJNUM];             // X部匹配结果;
    int yMatch[MAXOBJNUM];             // Y部匹配结果;
    int xValue[MAXOBJNUM];             // X部顶标;
    int yValue[MAXOBJNUM];             // Y部顶标;
    int slack[MAXOBJNUM];              // 松驰函数;
    int xVisit[MAXOBJNUM];
    int yVisit[MAXOBJNUM];
} HungarianHandle;

// 匈牙利算法求解;
int HungarianSolve(HungarianHandle* handle, std::vector<std::vector<int>>& cost, int rows, int cols);
#endif