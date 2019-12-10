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

#include "OperateConfig.h"
#include <fstream>
#include <iostream>
using namespace std;

bool IsSpace(char c)
{
    if (c == ' ' || c == '\t') {
        return true;
    }
    return false;
}

void Trim(string& str)
{
    if (str.empty()) {
        return;
    }
    int i, startPos, endPos;
    for (i = 0; i < str.size(); ++i) {
        if (!IsSpace(str[i])) {
            break;
        }
    }
    if (i == str.size()) {
        str = "";
        return;
    }

    startPos = i;
    for (i = str.size() - 1; i >= 0; --i) {
        if (!IsSpace(str[i])) {
            break;
        }
    }
    endPos = i;
    str = str.substr(startPos, endPos - startPos + 1);
}

int OperateConfig::InitConfigData()
{
    std::ifstream inFile(setupFileName);
    if (!inFile.is_open()) {
        std::cout << "cannot read setup.config file!" << std::endl;
        return -1;
    }
    string line, newLine;
    int startPos, endPos, pos;
    while (getline(inFile, line)) {
        if (line.empty()) {
            continue;
        }
        startPos = 0;
        endPos = line.size() - 1;
        pos = line.find(COMMENT_CHARATER);
        if (pos != -1) {
            if (pos == 0) {
                continue;
            }
            endPos = pos - 1;
        }
        newLine = line.substr(startPos, (endPos - startPos) + 1);  // delete comment
        pos = newLine.find('=');
        if (pos == -1) {
            continue;
        }
        string na = newLine.substr(0, pos);
        Trim(na);
        string value = newLine.substr(pos + 1, endPos + 1 - (pos + 1));
        Trim(value);
        configData.insert(std::make_pair(na, value));
    }
    return 0;
}
string OperateConfig::GetValue(const string& name)
{
    if (configData.count(name) == 0) {
        return "";
    }
    return configData.find(name)->second;
}

OperateConfig::OperateConfig(const string& filename)
{
    setupFileName = filename;
}
