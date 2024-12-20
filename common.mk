################################################################################
# \file common.mk
# \version 1.0
#
# \brief
# Settings shared across all projects.
#
################################################################################
# \copyright
# Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
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
################################################################################

MTB_TYPE=PROJECT

DEFINES+=CY_USING_HAL

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make modlibs' from command line), which will also update Eclipse IDE launch
# configurations. If TARGET is manually edited, ensure TARGET_<BSP>.mtb with a
# valid URL exists in the application, run 'make getlibs' to fetch BSP contents
# and update or regenerate launch configurations for your IDE.
TARGET=CY8CEVAL-062S2

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox IDE
#
# See also: CY_COMPILER_GCC_ARM_DIR in common_app.mk
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
# 
# If CONFIG is manually edited, ensure to update or regenerate launch configurations 
# for your IDE.
CONFIG=Debug

COMPONENTS+=CMSIS_DSP

include ../common_app.mk

################################################################################
# Search Paths for SVC MW and dependancies
################################################################################
## TODO: To be removed when libs are released on github
SEARCH_audio-voice-utils=../shared/middleware/audio-voice-utils
SEARCH_buffer-pool-manager=../shared/middleware/buffer-pool-manager
SEARCH_mw-utilities=../shared/middleware/mw-utilities
SEARCH_speech-onset-detection=../shared/middleware/speech-onset-detection
SEARCH_staged-voice-control=../shared/middleware/staged-voice-control
SEARCH_ipc=../shared/ipc

SEARCH+=$(SEARCH_audio-voice-utils)
SEARCH+=$(SEARCH_buffer-pool-manager)
SEARCH+=$(SEARCH_mw-utilities)
SEARCH+=$(SEARCH_speech-onset-detection)
SEARCH+=$(SEARCH_staged-voice-control)
SEARCH+=$(SEARCH_ipc)
################################################################################
