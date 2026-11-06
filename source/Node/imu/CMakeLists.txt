# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Scaffolding placeholder: no sources yet. The target will look roughly like
#
#   add_library( QCNodeImu SHARED
#     Imu.cpp         # lifecycle entry points and factory function
#     ImuConfig.cpp   # QCNodeConfigIfs implementation
#     ImuImpl.cpp     # implementation class, private header ImuImpl.hpp
#     ImuMonitor.cpp  # QCNodeMonitoringIfs implementation
#   )
#   target_include_directories( QCNodeImu PUBLIC ${QRB_QCNODES_ROOT}/include )
#
# The target name matters: tests/unit_test/Node/imu/ will guard on
# TARGET QCNodeImu, so renaming it here silently drops the unit tests.
