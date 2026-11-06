# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# This repository is built as a submodule of the QcNode project: the QcNode
# top-level CMakeLists.txt pulls this directory in with add_subdirectory(), and
# supplies project(), the toolchain and the C++ standard. There is deliberately
# no project() call here, which means
#
#   ${PROJECT_SOURCE_DIR}   the QcNode tree, for gtest and tests/utils
#   ${QRB_QCNODES_ROOT}     this repository, for its own headers and sources
#
# tests/ is added unconditionally. Each leaf test directory guards itself with
# if( NOT TARGET <node library> ) return() endif(), so a configuration that
# does not build a given node drops its tests instead of failing to link.
#
# Scaffolding placeholder: every leaf CMakeLists.txt below defines no targets
# yet. See docs/directory-structure.md for what belongs where.

cmake_minimum_required(VERSION 3.16)

set( QRB_QCNODES_ROOT ${CMAKE_CURRENT_SOURCE_DIR} )

add_subdirectory(source)
add_subdirectory(tests)
