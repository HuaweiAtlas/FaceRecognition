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
#ifndef ATLASFACEDEMO_YOLO_H
#define ATLASFACEDEMO_YOLO_H
#include <algorithm>
#include <cmath>
#include <iosfwd>
#include <iostream>
#include <memory>
#include <omp.h>
#include <string>
#include <utility>
#include <vector>
#include <cfloat>

using std::string;

#define MAX_OUTPUT_BOX_NUM 200
#define CLASS_NUM 1
#define POST_NMS_SIZE (0.45)
#define POST_THRESH_SIZE (0.25)

#define BATCH_SIZE 4
#define ANCHORS_DIM 3
// predict attr: location (x, y, w, h) & confidence
#define PREDICT_ATTR_DIM (5)
#define DET_FEATUREMAP_FIRST (52)
#define DET_FEATUREMAP_SECOND (26)
#define DET_FEATUREMAP_THIRED (13)

// yolov3-tiny
static float g_biases[12] = {21, 32, 30, 53, 50, 76, 75, 126, 131, 176, 203, 268};
static float g_mask[6] = {0, 1, 2, 3, 4, 5};

static inline float sigmoid(float x)
{
    return 1. / (1. + exp(-x));
}

typedef struct {
    float x, y, w, h;
} box_rect;

typedef struct Detection {
    box_rect bbox;
    float* prob;
    float objectness;
} Detection;

typedef struct {
    int class_id;
    float x;
    float y;
    float width;
    float height;
} DetectBox;

void activate_array(float* x, const int n)
{
    int i;
    for (i = 0; i < n; ++i) {
        x[i] = sigmoid(x[i]);
    }
}
void free_detections(Detection* dets, int n)
{
    int i;
    for (i = 0; i < n; ++i) {
        free(dets[i].prob);
    }
    free(dets);
}

static int entry_index(int w, int h, int c, int batch, int location, int entry)
{
    int n = location / (w * h);
    int loc = location % (w * h);

    return batch * w * h * c + n * w * h * (PREDICT_ATTR_DIM + CLASS_NUM) + entry * w * h + loc;
}
float overlap(float x1, float w1, float x2, float w2)
{
    float l1 = x1 - w1 / 2;
    float l2 = x2 - w2 / 2;
    float left = l1 > l2 ? l1 : l2;
    float r1 = x1 + w1 / 2;
    float r2 = x2 + w2 / 2;
    float right = r1 < r2 ? r1 : r2;

    return right - left;
}
float box_intersection(box_rect a, box_rect b)
{
    float w = overlap(a.x, a.w, b.x, b.w);
    float h = overlap(a.y, a.h, b.y, b.h);
    if (w < 0 || h < 0) {
        return 0;
    }

    float area = w * h;
    return area;
}

float box_union(box_rect a, box_rect b)
{
    float i = box_intersection(a, b);
    float u = a.w * a.h + b.w * b.h - i;
    u = (u > DBL_EPSILON) ? u : DBL_EPSILON;
    return u;
}
float box_iou(box_rect a, box_rect b)
{
    return box_intersection(a, b) / box_union(a, b);
}

int nms_comparator_v3(const void* pa, const void* pb)
{
    Detection a = *(Detection*)pa;
    Detection b = *(Detection*)pb;
    float diff = a.objectness - b.objectness;
    if (diff < 0) {
        return 1;
    } else if (diff > 0) {
        return -1;
    }

    return 0;
}
void do_nms_sort(Detection* dets, int total, int classes, float thresh)
{
    int i, j, k;
    k = total - 1;

    for (i = 0; i <= k; ++i) {
        if (dets[i].objectness == 0) {
            Detection swap = dets[i];
            dets[i] = dets[k];
            dets[k] = swap;
            --k;
            --i;
        }
    }
    total = k + 1;

    for (k = 0; k < classes; ++k) {
        qsort(dets, total, sizeof(Detection), nms_comparator_v3);
        for (i = 0; i < total; ++i) {
            if (dets[i].prob[k] == 0) {
                continue;
            }
            box_rect a = dets[i].bbox;
            for (j = i + 1; j < total; ++j) {
                box_rect b = dets[j].bbox;
                if (box_iou(a, b) > thresh) {
                    dets[j].prob[k] = 0;
                }
            }
        }
    }
}

box_rect get_yolo_box(float* x, float* biases, int n, int index, int i, int j, int lw, int lh, int w, int h, int stride)
{
    box_rect b;
    if (lw == 0 || lh == 0 || w == 0 || h == 0) {
        b.x = 0;
        b.y = 0;
        b.w = 0;
        b.h = 0;
        return b;
    }
    b.x = (i + x[index + 0 * stride]) / lw;
    b.y = (j + x[index + 1 * stride]) / lh;
    b.w = exp(x[index + 2 * stride]) * biases[2 * n] / w;
    b.h = exp(x[index + 3 * stride]) * biases[2 * n + 1] / h;

    return b;
}

void correct_yolo_boxes(Detection* dets, int n, int w, int h, int netw, int neth)
{
    int new_w = 0;
    int new_h = 0;

    if (((float)netw / w) < ((float)neth / h)) {
        new_w = netw;
        new_h = (h * netw) / w;
    } else {
        new_h = neth;
        new_w = (w * neth) / h;
    }

    for (int i = 0; i < n; ++i) {
        box_rect b = dets[i].bbox;
        b.x = (b.x - (netw - new_w) / 2. / netw) / ((float)new_w / netw);
        b.y = (b.y - (neth - new_h) / 2. / neth) / ((float)new_h / neth);
        b.w *= (float)netw / new_w;
        b.h *= (float)neth / new_h;
        dets[i].bbox = b;
    }
}
int get_yolo_detections(float* data_begin, int w, int h, int c, int img_w, int img_h, int netw, int neth, float thresh,
    Detection* dets, int q)
{
    int i, j, n;
    float* predictions = data_begin;
    int count = 0;
    for (i = 0; i < w * h; ++i) {
        int row = i / w;
        int col = i % w;
        for (n = 0; n < ANCHORS_DIM; ++n) {
            int obj_index = entry_index(w, h, c, 0, n * w * h + i, 4);
            float objectness = predictions[obj_index];
            if (objectness <= thresh) {
                continue;
            }

            int box_index = entry_index(w, h, c, 0, n * w * h + i, 0);
            dets[count].bbox = get_yolo_box(
                predictions, g_biases, g_mask[ANCHORS_DIM * (1 - q) + n], box_index, col, row, w, h, netw, neth, w * h);
            dets[count].objectness = objectness;

            for (j = 0; j < CLASS_NUM; ++j) {
                int class_index = entry_index(w, h, c, 0, n * w * h + i, 4 + 1 + j);
                float prob = objectness * predictions[class_index];
                dets[count].prob[j] = (prob > thresh) ? prob : 0;
            }
            ++count;
        }
    }

    return count;
}

void get_output_layer_info(int layer, int& w, int& h, int& c, int& offset)
{
    int i = layer;  // 第几层
    c = ANCHORS_DIM * (CLASS_NUM + PREDICT_ATTR_DIM);
    if (i == 0) {
        w = DET_FEATUREMAP_THIRED;
        h = DET_FEATUREMAP_THIRED;
        offset = 0;
    } else if (i == 1) {
        w = DET_FEATUREMAP_SECOND;
        h = DET_FEATUREMAP_SECOND;
        offset = DET_FEATUREMAP_THIRED * DET_FEATUREMAP_THIRED * c;
    } else if (i == 2) {
        w = DET_FEATUREMAP_FIRST;
        h = DET_FEATUREMAP_FIRST;
        offset = (DET_FEATUREMAP_THIRED * DET_FEATUREMAP_THIRED + DET_FEATUREMAP_SECOND * DET_FEATUREMAP_SECOND) * c;
    }
}

class CYolo {
public:
#define MAX_OBJECT 100

    CYolo()
    {
        dets = NULL;
        detectBox = NULL;
    }
    int Init()
    {
        dets = (Detection*)calloc(MAX_OBJECT, sizeof(Detection));
        if (dets == NULL) {
            return -1;
        }
        for (int i = 0; i < MAX_OBJECT; ++i) {
            dets[i].prob = (float*)calloc(CLASS_NUM, sizeof(float));
            if (dets[i].prob == NULL) {
                return -1;
            }
        }

        detectBox = new DetectBox[MAX_OBJECT];
        if (detectBox == NULL) {
            return -1;
        }
        reset_detect();
        return 0;
    }
    ~CYolo()
    {
        free_detections(dets, MAX_OBJECT);
        delete[] detectBox;
        dets = NULL;
        detectBox = NULL;
    }

    int boarder_check(int coord, int max_boarder)
    {
        coord = coord > 0 ? coord : 1;
        coord = coord < max_boarder ? coord : max_boarder - 1;
        return coord;
    }

    int process(float* netout, uint32_t size, int imw, int imh, std::vector<DetectInfo>& detectResult)
    {
        float thresh = POST_THRESH_SIZE;
        float nms = POST_NMS_SIZE;
        int box_nr = 0;
        int input_w = 416;
        int input_h = 416;
        int out_layers = 2;
        for (int i = 0; i < out_layers; i++) {
            int b = 0;
            int w, h, c = 0;
            int offset = 0;
            get_output_layer_info(i, w, h, c, offset);

            float* data_begin = netout + offset;
            for (int n = 0; n < ANCHORS_DIM; ++n) {
                int index = entry_index(w, h, c, b, n * w * h, 0);
                activate_array((float*)data_begin + index, 2 * w * h);

                index = entry_index(w, h, c, b, n * w * h, 4);
                activate_array((float*)data_begin + index, (1 + CLASS_NUM) * w * h);
            }

            for (int k = 0; k < w * h; ++k) {
                for (int n = 0; n < ANCHORS_DIM; ++n) {
                    int obj_index = entry_index(w, h, c, 0, n * w * h + k, 4);
                    if (data_begin[obj_index] > thresh) {
                        ++box_nr;
                    }
                }
            }
        }

        reset_detect();

        Detection* dets_p = dets;
        for (int i = 0; i < out_layers; i++) {
            int b = 0;
            int w, h, c = 0;
            int offset = 0;
            get_output_layer_info(i, w, h, c, offset);
            float* data_begin = netout + offset;

            int count = get_yolo_detections(data_begin, w, h, c, imw, imh, input_w, input_h, thresh, dets_p, i);
            dets_p += count;
        }

        if (nms) {
            do_nms_sort(dets, box_nr, CLASS_NUM, nms);
        }

        int j = 0;
        const int DET_PROB_THRESH = 0.25;
        for (int _v = 0; _v < box_nr; _v++) {
            for (int k = 0; k < CLASS_NUM; k++) {
                if (dets[_v].prob[k] < DET_PROB_THRESH) {
                    continue;
                }
                DetectInfo detectTemp = {1, dets[_v].prob[k]};
                detectTemp.location.anchor_lt.x = boarder_check((dets[_v].bbox.x - dets[_v].bbox.w / 2) * imw, imw);
                detectTemp.location.anchor_lt.y = boarder_check((dets[_v].bbox.y - dets[_v].bbox.h / 2) * imh, imh);
                detectTemp.location.anchor_rb.x = boarder_check((dets[_v].bbox.x + dets[_v].bbox.w / 2) * imw, imw);
                detectTemp.location.anchor_rb.y = boarder_check((dets[_v].bbox.y + dets[_v].bbox.h / 2) * imh, imh);
                detectResult.push_back(detectTemp);
                j++;
            }
        }
        return j;
    }

private:
    Detection* dets = NULL;
    DetectBox* detectBox = NULL;

    int reset_detect()
    {
        for (int i = 0; i < MAX_OBJECT; ++i) {
            int ms = memset_s(dets[i].prob, CLASS_NUM * sizeof(float), 0, CLASS_NUM * sizeof(float));
            if (ms != 0) {
                return -1;
            }
        }
        return 0;
    }
};

#endif