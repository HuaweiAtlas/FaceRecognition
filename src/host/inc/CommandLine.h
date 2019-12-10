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

#ifndef ATLASFACEDEMO_COMMAND_LINE_H
#define ATLASFACEDEMO_COMMAND_LINE_H
#include <gflags/gflags.h>
#include <string>

/// @brief Define flag for showing help message <br>
static const char help_message[] = "Print a usage message.";
DEFINE_bool(h, false, "Print a usage message.");
DEFINE_string(graph, "./data/graph.config", "the config file using for creating graph");
DEFINE_string(setup, "./data/setup.config", "the config file using for setting device id, camera number, etc.");
DEFINE_int32(disp, 1, "if need display or not, value which is equal or larger than 1 is True, otherwise False  ");

/**
 * @brief This function show a help message
 */
static void showUsage()
{
    std::cout << std::endl << "Usage: facedemo_main [Options...]" << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "    -h                             "
              << "show usage message." << std::endl;
    std::cout << "    -graph                         "
              << "Specify the graph.config file" << std::endl;
    std::cout << "    -setup                         "
              << "Specify the setup.config file" << std::endl;
    std::cout << "    -disp                         "
              << "if need display or not, value which is equal or larger than 1 is True, otherwise False" << std::endl;
}

#endif  // ATLASFACEDEMO_COMMAND_LINE_H
