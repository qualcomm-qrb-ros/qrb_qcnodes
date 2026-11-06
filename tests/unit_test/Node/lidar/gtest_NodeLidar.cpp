// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// Reference layout for a QCNode unit test. Copy this file and its
// CMakeLists.txt, rename Lidar to the node under test, drop the Sanity case
// and fill in the suites below.
//
// Cases are grouped by concern through the suite name rather than by fixture,
// matching the other node tests:
//
//   NodeLidar              behaviour of the node against real buffers
//   NodeLidarConfig        rejection of invalid configuration
//   NodeLidarStateMachine  lifecycle calls made out of order
//
// Document each case with a short Doxygen comment naming what it covers, so a
// reader can tell which branch is exercised without reading the body.

#include "gtest/gtest.h"

// The includes a node test normally needs. They stay commented out until the
// Lidar QCNode lands, because none of these headers exist yet:
//
// #include "QC/Node/Lidar.hpp"            // public header: config struct, factory
// #include "LidarImpl.hpp"                // private implementation class
// #include "QC/sample/BufferManager.hpp"  // buffer allocation for frame descriptors
// #include "md5_utils.hpp"                // golden output comparison, from tests/utils
// #include <string>
//
// using namespace QC::Node;
// using namespace QC::test::utils;
// using namespace QC::sample;

/**
 * @brief Proves the test binary compiles, links against gtest and runs.
 *
 * This is the only case here that builds today, because it touches nothing
 * belonging to the node. Delete it as soon as there is real coverage.
 */
TEST( NodeLidar, Sanity )
{
    EXPECT_TRUE( true );
}

// ---------------------------------------------------------------------------
// Configuration validation
//
// One case per field the node must reject, each asserting that Initialize()
// refuses the DataTree instead of returning QC_STATUS_OK. Assert with
// EXPECT_NE on the status rather than on a specific error code, so the case
// survives a change of error code.
// ---------------------------------------------------------------------------
//
// /**
//  * @brief Configuration carrying an empty node name
//  * @coverage LidarConfig.cpp, name validation
//  */
// TEST( NodeLidarConfig, EmptyName )
// {
//     QCNodeIfs *pLidar = new QC::Node::Lidar();
//     DataTree dt;
//     dt.Set<std::string>( "static.name", "" );
//     dt.Set<uint32_t>( "static.id", 0 );
//     dt.SetProcessorType( "static.processorType", QC_PROCESSOR_CPU );
//     QCNodeInit_t config = { dt.Dump() };
//
//     EXPECT_NE( QC_STATUS_OK, pLidar->Initialize( config ) );
//
//     delete pLidar;
// }

// ---------------------------------------------------------------------------
// Lifecycle ordering
//
// The Graph Framework drives Initialize() -> Start() ->
// ProcessFrameDescriptor() -> Stop() -> DeInitialize(). These cases assert the
// node rejects calls made out of that order, and need no buffers, which makes
// them the cheapest coverage to write first.
// ---------------------------------------------------------------------------
//
// /**
//  * @brief Start() before Initialize()
//  * @expected QC_STATUS_BAD_STATE
//  */
// TEST( NodeLidarStateMachine, StartInWrongState )
// {
//     QC::Node::Lidar lidar;
//
//     EXPECT_EQ( QC_STATUS_BAD_STATE, lidar.Start() );
// }

// ---------------------------------------------------------------------------
// Behaviour against buffers
//
// Allocate through BufferManager, push one frame descriptor, and compare the
// output against a golden file with md5_utils. Free every buffer on the way
// out, including on the failure paths, so a failing case does not leak into
// the ones that follow.
// ---------------------------------------------------------------------------
//
// /**
//  * @brief One scan in, one point cloud frame out
//  * @coverage LidarImpl.cpp, ProcessFrameDescriptor happy path
//  */
// TEST( NodeLidar, ProcessOneScan )
// {
//     BufferManager mgr;
//     // ... allocate input and output, Initialize, Start,
//     //     ProcessFrameDescriptor, Stop, DeInitialize, compare, Free
// }

// The aggregated binary compiles every node's cases into one executable and
// supplies its own main(), so this one is compiled out there.
#ifndef GTEST_QCNODE
#if __CTC__
extern "C" void ctc_append_all( void );
#endif
int main( int argc, char **argv )
{
    ::testing::InitGoogleTest( &argc, argv );
    int nVal = RUN_ALL_TESTS();
#if __CTC__
    ctc_append_all();
#endif
    return nVal;
}
#endif
