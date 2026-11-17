// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleDemo.cpp
 * @brief Sample for the Demo QCNode, runnable with QCNodeSampleApp (-t Demo).
 *
 * The sample creates the Demo node through QCNodeFactory, then every interval_ms milliseconds
 * from a worker thread it puts an OdometryDescriptor_t into the frame descriptor and calls
 * ProcessFrameDescriptor(), the node prints the frames, the position and the linear velocity
 * carried by the descriptor.
 *
 * The robot moves along the x axis of the "odom" frame at a constant speed.
 *
 * Attributes:
 *   interval_ms : (optional) the interval between two frames in milliseconds, default 1000
 *   velocity    : (optional) the linear velocity along x in m/s, default 1.0
 */

#include "QC/Node/Demo.hpp"
#include "QC/sample/SampleIF.hpp"
#include "QRB/Memory/Navigation/OdometryDescriptor.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace QC
{
namespace sample
{

using namespace QC::Node;

class SampleDemo : public SampleIF
{
public:
    SampleDemo() = default;
    ~SampleDemo() = default;

    QCStatus_e Init( std::string name, SampleConfig_t &config ) override;
    QCStatus_e Start() override;
    QCStatus_e Stop() override;
    QCStatus_e Deinit() override;

private:
    void ThreadMain();

private:
    std::unique_ptr<QCNodeIfs> m_node;
    std::thread m_thread;
    std::atomic<bool> m_stop{ false };
    uint32_t m_intervalMs = 1000;
    float m_velocity = 1.0f;
    uint32_t m_errorCount = 0;

    // The global buffer id of the node input in the frame descriptor.
    static constexpr uint32_t kInputBufferId = 0;
};

QCStatus_e SampleDemo::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_DEMO );

    if ( QC_STATUS_OK == ret )
    {
        m_intervalMs = Get( config, "interval_ms", (uint32_t) 1000 );
        m_velocity = Get( config, "velocity", 1.0f );
        ret = QCNodeFactory::CreateNode( QC_NODE_TYPE_DEMO, m_node );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to create the Demo node: %d", ret );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        DataTree dt;
        dt.Set<std::string>( "static.name", name );
        dt.Set<uint32_t>( "static.id", m_nodeId.id );

        QCNodeInit_t nodeInit;
        nodeInit.config = dt.Dump();
        ret = m_node->Initialize( nodeInit );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to initialize the Demo node: %d", ret );
        }
    }

    return ret;
}

QCStatus_e SampleDemo::Start()
{
    QCStatus_e ret = m_node->Start();

    if ( QC_STATUS_OK == ret )
    {
        m_stop = false;
        m_thread = std::thread( &SampleDemo::ThreadMain, this );
    }
    else
    {
        QC_ERROR( "failed to start the Demo node: %d", ret );
    }

    return ret;
}

void SampleDemo::ThreadMain()
{
    // The descriptor carries the odometry itself, it does not need a DMA buffer.
    QRB::Memory::OdometryDescriptor_t odom;
    (void) odom.frameId.Assign( "odom" );
    (void) odom.childFrameId.Assign( "base_link" );
    odom.twistWithCovariance.twist.linear.x = m_velocity;

    const double dt = (double) m_intervalMs / 1000.0;
    uint64_t timestampNs = 0;

    while ( false == m_stop )
    {
        NodeFrameDescriptor frameDesc( 1 );
        (void) frameDesc.SetBuffer( kInputBufferId, odom );

        QCStatus_e ret = m_node->ProcessFrameDescriptor( frameDesc );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "ProcessFrameDescriptor failed: %d", ret );
            m_errorCount++;
        }

        // Move the robot for the next frame.
        odom.poseWithCovariance.pose.position.x += m_velocity * dt;
        timestampNs += (uint64_t) m_intervalMs * 1000000ULL;
        odom.timestampNs = timestampNs;

        std::this_thread::sleep_for( std::chrono::milliseconds( m_intervalMs ) );
    }
}

QCStatus_e SampleDemo::Stop()
{
    m_stop = true;
    if ( m_thread.joinable() )
    {
        m_thread.join();
    }

    QCStatus_e ret = m_node->Stop();
    if ( QC_STATUS_OK == ret )
    {
        DemoMonitor_t monitor{};
        uint32_t size = sizeof( monitor );
        if ( QC_STATUS_OK == m_node->GetMonitoringIfs().Place( &monitor, size ) )
        {
            printf( "%s processed %u frames, %u errors\n", m_name.c_str(), monitor.frameCount,
                    m_errorCount );
        }
        if ( 0 != m_errorCount )
        {
            ret = QC_STATUS_FAIL;
        }
    }
    else
    {
        QC_ERROR( "failed to stop the Demo node: %d", ret );
    }

    return ret;
}

QCStatus_e SampleDemo::Deinit()
{
    QCStatus_e ret = m_node->DeInitialize();
    m_node.reset();
    return ret;
}

REGISTER_SAMPLE( Demo, SampleDemo );

}   // namespace sample
}   // namespace QC
