# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Reference layout for a QCNode unit test. Copy this directory, rename Lidar to
# the node under test, and the rest follows.
#
# The guard is what lets every node test sit in the build unconditionally: when
# the node library has not been built -- a disabled feature, or a configure in
# which the QcNode targets are absent -- the directory drops out quietly
# instead of failing to link.
if( NOT TARGET QCNodeLidar )
    return()
endif()

set( HEADERS_DIR ${QRB_QCNODES_ROOT}/include            # this repository's public headers
                 ${QRB_QCNODES_ROOT}/source/Node/lidar  # the node's private headers, LidarImpl.hpp
                 ${PROJECT_SOURCE_DIR}/tests/utils      # QCNodeTestUtils: md5_utils.hpp, accuracy.hpp
)

add_executable( gtest_NodeLidar gtest_NodeLidar.cpp )
target_include_directories( gtest_NodeLidar PUBLIC ${HEADERS_DIR} )
target_link_libraries( gtest_NodeLidar gtest QCNodeLidar QCNodeTestUtils BufferManager )
install( TARGETS gtest_NodeLidar DESTINATION bin )

# gtest_main is deliberately not linked: gtest_NodeLidar.cpp carries its own
# main(), so the same sources can also be folded into the aggregated binary
# that compiles every node's cases into one executable. Uncomment to join it --
# the GTEST_QCNODE compile definition is what suppresses the local main().
#
# set( GTEST_QCNODE_SOURCES ${GTEST_QCNODE_SOURCES}
#      ${CMAKE_CURRENT_SOURCE_DIR}/gtest_NodeLidar.cpp
#      PARENT_SCOPE )
