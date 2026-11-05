# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Scaffolding placeholder. This configures, but builds nothing: every leaf
# CMakeLists.txt below is a placeholder waiting for its sources. See
# docs/directory-structure.md for what belongs where.

cmake_minimum_required(VERSION 3.16)

project(qrb_qcnodes LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(QRB_QCNODES_BUILD_TESTS "Build the unit tests and the sample application" OFF)

add_subdirectory(source)

if(QRB_QCNODES_BUILD_TESTS)
  add_subdirectory(tests)
endif()
