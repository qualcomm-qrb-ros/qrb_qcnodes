# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Built as part of the qcnode build (see qcnode/source/CMakeLists.txt, or the qcnode_%.bbappend of
# meta-qcom-robotics-sdk, which adds this directory through its CMAKE_PROJECT_qcnode_INCLUDE hook).
set( QRB_QCNODES_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../../include )
set( QRB_CONTRACTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../qrb_contracts )

# The color and depth images and the IMU samples are qrb_contracts descriptors.
if( NOT TARGET qrb_contracts::qrb_contracts )
    add_subdirectory( ${QRB_CONTRACTS_DIR} qrb_contracts )
endif()

# librealsense2 (the librealsense2 recipe in Yocto, a system library of the device otherwise).
# A Yocto sysroot carries its CMake package. A toolchain sysroot that mirrors the device /usr, as
# the other QCNode vendor libraries are found (e.g. qcxclient for the Camera node), may only have
# the headers and the versioned library: then they are taken from ${CMAKE_SYSROOT}/usr.
find_package( realsense2 CONFIG QUIET )
if( TARGET realsense2::realsense2 )
    # libQCNode.so is defined in the qcnode top directory: make the imported target visible there.
    set_target_properties( realsense2::realsense2 PROPERTIES IMPORTED_GLOBAL TRUE )
    set( REALSENSE2_LIBRARY realsense2::realsense2 )
    message( STATUS "RealSense node: librealsense2 ${realsense2_VERSION} (CMake package)" )
else()
    set( REALSENSE2_LIB_FILE "" )
    if( EXISTS ${CMAKE_SYSROOT}/usr/lib/librealsense2.so )
        set( REALSENSE2_LIB_FILE ${CMAKE_SYSROOT}/usr/lib/librealsense2.so )
    else()
        # The device only installs the real file librealsense2.so.<version>, without the
        # librealsense2.so development symlink.
        file( GLOB REALSENSE2_VERSIONED_LIBS ${CMAKE_SYSROOT}/usr/lib/librealsense2.so.* )
        list( SORT REALSENSE2_VERSIONED_LIBS )
        list( LENGTH REALSENSE2_VERSIONED_LIBS REALSENSE2_NUM_LIBS )
        if( REALSENSE2_NUM_LIBS GREATER 0 )
            list( GET REALSENSE2_VERSIONED_LIBS -1 REALSENSE2_LIB_FILE )
        endif()
    endif()

    if( NOT EXISTS ${CMAKE_SYSROOT}/usr/include/librealsense2/rs.hpp OR NOT REALSENSE2_LIB_FILE )
        message( FATAL_ERROR "librealsense2 not found: no realsense2 CMake package, and not in the "
                             "toolchain sysroot ${CMAKE_SYSROOT} (usr/include/librealsense2/rs.hpp "
                             "and usr/lib/librealsense2.so*, as installed on the device). Add "
                             "librealsense2 to DEPENDS, or use the toolchain matching the device "
                             "image." )
    endif()
    set( REALSENSE2_LIBRARY ${REALSENSE2_LIB_FILE} )
    message( STATUS "RealSense node: librealsense2 ${REALSENSE2_LIB_FILE} (sysroot)" )
endif()

# If the sysroot is older than the device image, librealsense2 may need a newer glibc than the
# sysroot provides (e.g. GLIBC_2.43 vs 2.39). Executables linking libQCNode.so then fail with
# "undefined reference to ...@GLIBC_2.43" although they run fine on the device. Prefer the
# toolchain matching the device; this option is a workaround that skips that link-time check.
option( REALSENSE_ALLOW_SHLIB_UNDEFINED "Do not check the librealsense2 dependencies at link time" OFF )

set( QRB_REALSENSE_HEADERS ${QRB_QCNODES_INCLUDE}/QC/Node/RealSense.hpp )

add_library( QCNodeRealSense OBJECT
    RealSense.cpp
    RealSenseConfig.cpp
    RealSenseMonitoring.cpp
    RealSenseImpl.cpp
)

set_property( TARGET QCNodeRealSense PROPERTY LINKER_LANGUAGE CXX )
set_property( TARGET QCNodeRealSense PROPERTY POSITION_INDEPENDENT_CODE ON )

target_include_directories( QCNodeRealSense PUBLIC ${PROJECT_SOURCE_DIR}/include ${QRB_QCNODES_INCLUDE} )
# PUBLIC so that libQCNode.so, which links this OBJECT library, links librealsense2, and its users
# see the qrb_contracts descriptors the node delivers.
target_link_libraries( QCNodeRealSense PUBLIC QCNodeBase qrb_contracts::qrb_contracts
                       ${REALSENSE2_LIBRARY} pthread )
target_compile_options( QCNodeRealSense PRIVATE -Wall )
if( REALSENSE_ALLOW_SHLIB_UNDEFINED )
    message( WARNING "RealSense node: linking with --allow-shlib-undefined" )
    target_link_options( QCNodeRealSense INTERFACE -Wl,--allow-shlib-undefined )
endif()

install( FILES ${QRB_REALSENSE_HEADERS} DESTINATION include/QC/Node )

# Add the RealSense sample to QCNodeSample so that it can be run by
# `qcrun ./bin/QCNodeSampleApp -t RealSense`. QCNodeSample is defined later in the qcnode build
# (tests/sample), so defer the call. The path is expanded here because variables in a deferred call
# are evaluated at the time it runs.
get_filename_component( QRB_REALSENSE_SAMPLE ${CMAKE_CURRENT_SOURCE_DIR}/../../../tests/sample/SampleRealSense.cpp ABSOLUTE )
cmake_language( EVAL CODE "cmake_language( DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL target_sources QCNodeSample PRIVATE ${QRB_REALSENSE_SAMPLE} )" )
