# -*- python -*-

# Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

package(default_visibility = ["//visibility:public"])
load("@com_github_ARMmbed_mbed-os//:rules.bzl", "mbed_binary")

COPTS = [
    "-Werror",
    "-Wdouble-promotion",
    "-Wno-maybe-uninitialized",
]

cc_library(
    name = "git_info",
    hdrs = ["git_info.h"],
    srcs = ["git_info.cc"],
    linkstamp = "git_info_linkstamp.cc",
    deps = [
        "@com_github_mjbots_mjlib//mjlib/base:visitor",
    ],
    copts = COPTS,
)

mbed_binary(
    name = "power_dist",
    srcs = [
        "assert.cc",
        "fdcan.cc",
        "fdcan.h",
        "fdcan_micro_server.h",
        "firmware_info.cc",
        "firmware_info.h",
        "lm5066.cc",
        "lm5066.h",
        "millisecond_timer.h",
        "power_dist.cc",
        "power_dist_hw.h",
        "stm32g4_flash.h",
        "uuid.cc",
        "uuid.h",
    ],
    deps = [
        ":git_info",
        "@com_github_mjbots_mjlib//mjlib/base:assert",
        "@com_github_mjbots_mjlib//mjlib/base:tokenizer",
        "@com_github_mjbots_mjlib//mjlib/base:limit",
        "@com_github_mjbots_mjlib//mjlib/base:string_span",
        "@com_github_mjbots_mjlib//mjlib/micro:async_exclusive",
        "@com_github_mjbots_mjlib//mjlib/micro:async_stream",
        "@com_github_mjbots_mjlib//mjlib/micro:callback_table",
        "@com_github_mjbots_mjlib//mjlib/micro:command_manager",
        "@com_github_mjbots_mjlib//mjlib/micro:persistent_config",
        "@com_github_mjbots_mjlib//mjlib/micro:pool_ptr",
        "@com_github_mjbots_mjlib//mjlib/micro:telemetry_manager",
        "@com_github_mjbots_mjlib//mjlib/multiplex:micro_server",
        "@com_github_mjbots_mjlib//mjlib/multiplex:micro_stream_datagram",
    ],
    copts = COPTS,
)

OCD = (
    "openocd " +
    "-f interface/stlink.cfg " +
    "-f target/stm32g4x.cfg "
)

genrule(
    name = "flash",
    tags = ["manual"],
    srcs = ["power_dist.bin"],
    outs = ["power_dist.stamp"],
    cmd = (OCD + "-c init -c \"reset_config none separate; program $(location power_dist.bin) verify 0x8000000 reset exit 0x8000000\" && touch $@"),
)
