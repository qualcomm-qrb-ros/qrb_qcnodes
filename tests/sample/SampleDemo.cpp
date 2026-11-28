// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleDemo.cpp
 * @brief Sample for the Demo QCNode, runnable with QCNodeSampleApp (-t Demo).
 *
 * The sample creates the Demo node through QCNodeFactory, then every interval_ms milliseconds
 * from a worker thread it puts an OdometryDescriptor_t into the frame descriptor and calls
 * ProcessFrameDescriptor(), the node prints the frames, the position and the linear velocity
 * carried by the descriptor. The node reports the odometry back through the callback, which the
 * sample prints, counts and, with output_topic, publishes with a DataPublisher over the DataBroker
 * for another sample to consume, as a DemoFrameRef_t owning a copy of the OdometryDescriptor_t.
 *
 * The robot moves along the x axis of the "odom" frame at a constant speed.
 *
 * Attributes:
 *   interval_ms : (optional) the interval between two frames in milliseconds, default 1000
 *   velocity    : (optional) the linear velocity along x in m/s, default 1.0
 *   output_topic: (optional) the DataBroker topic to publish the odometry of the node on
 */

#include "QC/Node/Demo.hpp"
#include "QC/sample/DataBroker.hpp"
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

/**
 * @brief What the sample publishes over the DataBroker: one odometry. The descriptor holds its
 *        values in itself, so the message owns a copy and outlives the callback of the node.
 */
class DemoFrame
{
public:
    explicit DemoFrame( const QRB::Memory::OdometryDescriptor_t &odom ) : m_odom( odom ) {}
    QRB::Memory::OdometryDescriptor_t &Odom() { return m_odom; }

private:
    QRB::Memory::OdometryDescriptor_t m_odom;
};
typedef std::shared_ptr<DemoFrame> DemoFrameRef_t;

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
    void OnEvent( const QCNodeEventInfo_t &info );

private:
    std::unique_ptr<QCNodeIfs> m_node;
    std::thread m_thread;
    std::atomic<bool> m_stop{ false };
    uint32_t m_intervalMs = 1000;
    float m_velocity = 1.0f;
    uint32_t m_errorCount = 0;
    std::atomic<uint32_t> m_publishedCount{ 0 };
    std::string m_outputTopic;
    DataPublisher<DemoFrameRef_t> m_pub;

    // The global buffer id of the node input in the frame descriptor.
    static constexpr uint32_t kInputBufferId = 0;
    // The global buffer id of the odometry the node publishes in the frame descriptor.
    static constexpr uint32_t kOutputBufferId = 0;
};

QCStatus_e SampleDemo::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_DEMO );

    if ( QC_STATUS_OK == ret )
    {
        m_intervalMs = Get( config, "interval_ms", (uint32_t) 1000 );
        m_velocity = Get( config, "velocity", 1.0f );
        m_outputTopic = Get( config, "output_topic", "" );
        if ( false == m_outputTopic.empty() )
        {
            ret = m_pub.Init( name, m_outputTopic );
            if ( QC_STATUS_OK != ret )
            {
                QC_ERROR( "failed to init the publisher of topic %s: %d", m_outputTopic.c_str(),
                          ret );
            }
        }
    }

    if ( QC_STATUS_OK == ret )
    {
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
        nodeInit.callback = [this]( const QCNodeEventInfo_t &info ) { OnEvent( info ); };
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

void SampleDemo::OnEvent( const QCNodeEventInfo_t &info )
{
    const QRB::Memory::OdometryDescriptor_t *pOdom =
            dynamic_cast<const QRB::Memory::OdometryDescriptor_t *>(
                    &info.frameDesc.GetBuffer( kOutputBufferId ) );
    if ( ( QC_STATUS_OK == info.status ) && ( nullptr != pOdom ) )
    {
        const QRB::Memory::Point3d_t &position = pOdom->poseWithCovariance.pose.position;
        m_publishedCount++;
        printf( "%s published odom: %s -> %s, position (%.3f, %.3f, %.3f)\n", m_name.c_str(),
                pOdom->frameId.CStr(), pOdom->childFrameId.CStr(), position.x, position.y,
                position.z );

        if ( false == m_outputTopic.empty() )
        {
            DemoFrameRef_t frame = std::make_shared<DemoFrame>( *pOdom );
            (void) m_pub.Publish( frame );
        }
    }
    else
    {
        QC_ERROR( "bad event: status %d, odom %p", info.status, (const void *) pOdom );
        m_errorCount++;
    }
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
            printf( "%s processed %u frames, %u odometry received, %u errors\n", m_name.c_str(),
                    monitor.frameCount, m_publishedCount.load(), m_errorCount );
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
