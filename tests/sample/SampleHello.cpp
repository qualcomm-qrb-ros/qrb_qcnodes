// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleHello.cpp
 * @brief Sample for the Hello QCNode, runnable with QCNodeSampleApp (-t Hello).
 *
 * The sample creates the Hello node through QCNodeFactory, then calls
 * ProcessFrameDescriptor() every interval_ms milliseconds from a worker thread.
 *
 * Attributes:
 *   interval_ms : (optional) the interval between two frames in milliseconds, default 1000
 */

#include "QC/Node/Hello.hpp"
#include "QC/sample/SampleIF.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace QC
{
namespace sample
{

using namespace QC::Node;

class SampleHello : public SampleIF
{
public:
    SampleHello() = default;
    ~SampleHello() = default;

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
};

QCStatus_e SampleHello::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_HELLO );

    if ( QC_STATUS_OK == ret )
    {
        m_intervalMs = Get( config, "interval_ms", (uint32_t) 1000 );
        ret = QCNodeFactory::CreateNode( QC_NODE_TYPE_HELLO, m_node );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to create the Hello node: %d", ret );
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
            QC_ERROR( "failed to initialize the Hello node: %d", ret );
        }
    }

    return ret;
}

QCStatus_e SampleHello::Start()
{
    QCStatus_e ret = m_node->Start();

    if ( QC_STATUS_OK == ret )
    {
        m_stop = false;
        m_thread = std::thread( &SampleHello::ThreadMain, this );
    }
    else
    {
        QC_ERROR( "failed to start the Hello node: %d", ret );
    }

    return ret;
}

void SampleHello::ThreadMain()
{
    while ( false == m_stop )
    {
        NodeFrameDescriptor frameDesc( 1 );
        QCStatus_e ret = m_node->ProcessFrameDescriptor( frameDesc );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "ProcessFrameDescriptor failed: %d", ret );
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( m_intervalMs ) );
    }
}

QCStatus_e SampleHello::Stop()
{
    m_stop = true;
    if ( m_thread.joinable() )
    {
        m_thread.join();
    }

    QCStatus_e ret = m_node->Stop();
    if ( QC_STATUS_OK == ret )
    {
        HelloMonitor_t monitor{};
        uint32_t size = sizeof( monitor );
        if ( QC_STATUS_OK == m_node->GetMonitoringIfs().Place( &monitor, size ) )
        {
            printf( "%s processed %u frames\n", m_name.c_str(), monitor.frameCount );
        }
    }
    else
    {
        QC_ERROR( "failed to stop the Hello node: %d", ret );
    }

    return ret;
}

QCStatus_e SampleHello::Deinit()
{
    QCStatus_e ret = m_node->DeInitialize();
    m_node.reset();
    return ret;
}

REGISTER_SAMPLE( Hello, SampleHello );

}   // namespace sample
}   // namespace QC
