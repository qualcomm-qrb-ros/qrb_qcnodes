# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Scaffolding placeholder: no sources yet. The target will look roughly like
#
#   add_library( QCNodeLidar SHARED
#     Lidar.cpp         # lifecycle entry points and factory function
#     LidarConfig.cpp   # QCNodeConfigIfs implementation
#     LidarImpl.cpp     # implementation class, private header LidarImpl.hpp
#     LidarMonitor.cpp  # QCNodeMonitoringIfs implementation
#   )
#   target_include_directories( QCNodeLidar PUBLIC ${QRB_QCNODES_ROOT}/include )
#
# The target name matters: tests/unit_test/Node/lidar/CMakeLists.txt guards on
# TARGET QCNodeLidar, so renaming it here silently drops the unit tests.
