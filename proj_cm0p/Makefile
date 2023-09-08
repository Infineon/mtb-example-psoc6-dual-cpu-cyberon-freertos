################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Makefile for configurating the Cortex-M0p core project
#
################################################################################
# \copyright
# Copyright 2023, Cypress Semiconductor Corporation (an Infineon company)
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

include ../common.mk

################################################################################
# Basic Configuration
################################################################################

# Name of application (used to derive name of final linked file).
# 
# If APPNAME is edited, ensure to update or regenerate launch 
# configurations for your IDE.
APPNAME=cm0p_app

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS+=FREERTOS PSOC6HAL

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=CM0P_SLEEP CM0P_SECURE CM0P_CRYPTO CM0P_BLESS

# Set this application to run at the CM0+
CORE=CM0P
CORE_NAME=CM0P_0

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=
# Add additional defines to the build process (without a leading -D).
DEFINES+=CY_CORTEX_M4_APPL_ADDR=0x100A8000 CY_IPC_DEFAULT_CFG_DISABLE=1 __M33__
DEFINES+=ENABLE_SVC_LP_MW 
DEFINES+=CY_ML_FLOATING_POINT_fltxflt_NN=1 COMPONENT_ML_FLOAT32=1
DEFINES+=EMBEDDED_DEV
DEFINES+=RUN_LPWWD_ONLY
DEFINES+=C_MODELS=1
DEFINES+=ENABLE_PSOC6_IPC_COMMUNICATION

# Enable SVC debug logs
# DEFINES += ENABLE_SVC_LP_LOGS         #Enable logs through cy_log
# DEFINES += ENABLE_SVC_LP_LOGS=2       #Enable logs through direct printf function

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CFLAGS=

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
LDFLAGS=

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT=

# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
POSTBUILD=

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field 
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by 
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level 
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../..

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Link to the folder containing source files shared by CM4 and CM0+ CPU.
CY_EXTAPP_PATH= 

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=


include $(CY_TOOLS_DIR)/make/start.mk
