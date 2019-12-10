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

#undef Status
#include <assert.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>
#include <signal.h>

#include "hiaiengine/ai_memory.h"
#include "hiaiengine/api.h"
#include "hiaiengine/data_type.h"
#include "hiaiengine/graph.h"
#include "inc/common.h"

#include "CommandLine.h"
#include "OperateConfig.h"
#include "stream_data.h"

using namespace std;
#define GRAPH_ID 100
bool g_signalRecieved = false;
int g_cameraNum;
int g_isDisplay;
bool g_isResume = false;
HIAI_REGISTER_SERIALIZE_FUNC("RawDataBufferHigh", RawDataBufferHigh, GetTransSearPtr, GetTransDearPtr);
HIAI_REGISTER_SERIALIZE_FUNC("StreamRawData", StreamRawData, StreamRawDataSerialize, StreamRawDataDeserialize);
void SigHandler(int signo)
{
    if (signo == SIGINT) {
        g_signalRecieved = true;
    }
}

int ReadConfigFromFile(OperateConfig& configData, int& deviceId)
{
    string tmp;
    string strkey;

    tmp = configData.GetValue("used_cam_num");
    if (tmp == "") {
        printf("[Error]: invalid used_cam_num\n");
        return 1;
    }
    if (!(std::stringstream(tmp) >> g_cameraNum)) {
        std::cout << "used_cam_num is not digit,please check!" << std::endl;
        return 1;
    }

    if (!(g_cameraNum >= 1 && g_cameraNum <= g_NUM_CAMERAS)) {
        printf("[Error]: invalid used_cam_num\n");
        return 1;
    }
    for (int i = 0; i < g_cameraNum; i++) {
        strkey = "cam";
        const int len = 10;
        char stritoa[len];
        sprintf_s(stritoa, len, "%d", i);
        strkey.append(stritoa);
        tmp = configData.GetValue(strkey);
        if (tmp == "") {
            printf("[Error]: invalid %s\n", strkey.c_str());
            return 1;
        }
        g_CAMERA_SETTINGS[i] = tmp;
    }

    tmp = configData.GetValue("device_id");
    if (!(std::stringstream(tmp) >> deviceId)) {
        std::cout << "deviceId is not digit,please check!" << std::endl;
        return 1;
    }
    // maybe device_id is not in this range!
    if (!(deviceId >= 0 && deviceId < 64)) {
        printf("[Error]: invalid device_id\n");
        return 1;
    }
    return 0;
}

bool ParseAndCheckCommandLine(int argc, char* argv[])
{
    gflags::ParseCommandLineNonHelpFlags(&argc, &argv, true);
    if (FLAGS_h) {
        showUsage();
        return false;
    }
    if (FLAGS_disp >= 1) {
        g_isDisplay = 1;
    } else {
        g_isDisplay = 0;
    }
    return true;
}

int main(int argc, char** argv)
{
    if (!ParseAndCheckCommandLine(argc, argv)) {
        return 0;
    }

    int deviceId = 0;
    OperateConfig faceConfig(FLAGS_setup);
    if (faceConfig.InitConfigData() == -1) {
        return -1;
    }
    if (ReadConfigFromFile(faceConfig, deviceId) == 1) {
        return 1;
    }

    while (!g_signalRecieved) {
        g_isResume = false;
        std::cout << "Starting the demo ..." << std::endl;
        HIAI_StatusT status = HIAI_Init(deviceId);
        if (status != HIAI_OK) {
            std::cout << "init device FAILED!" << std::endl;
            exit(0);
        }

        status = hiai::Graph::CreateGraph(FLAGS_graph);
        if (status != HIAI_OK) {
            std::cout << "Create graph FAILED!" << std::endl;
            exit(0);
        }

        std::shared_ptr<hiai::Graph> graph = hiai::Graph::GetInstance(GRAPH_ID);
        if (graph == nullptr) {
            printf("Fail to get the graph-%u\n", GRAPH_ID);
            return -1;
        }

        if (signal(SIGINT, SigHandler) == SIG_ERR) {
            printf("\ncan't catch SIGINT\n");
        }

        for (int i = 0; i < g_cameraNum; i++) {
            hiai::EnginePortID engineInfo;
            engineInfo.graph_id = GRAPH_ID;
            engineInfo.engine_id = g_RTSPPULLER_ENGINE_ID[i];
            engineInfo.port_id = 0;
            std::shared_ptr<std::string> srcData(new std::string(g_CAMERA_SETTINGS[i]));
            graph->SendData(engineInfo, "string", std::static_pointer_cast<void>(srcData));
        }

        const uint32_t SIGNAL_CHECK_TIMESTEP = 10000;
        while ((!g_signalRecieved) && (!g_isResume)) {
                usleep(SIGNAL_CHECK_TIMESTEP);
        }
        hiai::Graph::DestroyGraph(GRAPH_ID);
        const uint32_t DESTORY_TIMESTEP = 5000000;
        usleep(DESTORY_TIMESTEP);
    }
    printf("face demo finished\n");
    return 0;
}