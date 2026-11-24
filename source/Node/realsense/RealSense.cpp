// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/RealSense.hpp"
#include "RealSenseImpl.hpp"

namespace QC
{
namespace Node
{

// Register the RealSense node with QCNodeFactory at static-initialization time.
const NodeRegistrar<RealSense> g_registerRealSense{ QC_NODE_TYPE_REALSENSE };

RealSense::RealSense()
    : m_configIfs( m_logger ),
      m_monitorIfs( m_logger ),
      m_pImpl( std::make_unique<RealSenseImpl>( m_nodeId, m_logger, m_monitorIfs ) )
{}

RealSense::~RealSense() = default;

QCStatus_e RealSense::Initialize( QCNodeInit_t &config )
{
    QCStatus_e status = QC_STATUS_OK;
    std::string errors;

    if ( QC_OBJECT_STATE_INITIAL != m_pImpl->GetState() )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        status = m_configIfs.VerifyAndSet( config.config, errors );
        if ( QC_STATUS_OK != status )
        {
            QC_ERROR( "Config error: %s", errors.c_str() );
        }
    }

    if ( QC_STATUS_OK == status )
    {
        status = NodeBase::Init( m_configIfs.Get().nodeId );
    }

    if ( QC_STATUS_OK == status )
    {
        status = m_pImpl->Initialize( m_configIfs.GetConfig(), config.callback, config.buffers );
        if ( QC_STATUS_OK != status )
        {
            (void) NodeBase::DeInitialize();
        }
    }

    return status;
}

QCStatus_e RealSense::Start()
{
    return m_pImpl->Start();
}

QCStatus_e RealSense::ProcessFrameDescriptor( QCFrameDescriptorNodeIfs &frameDesc )
{
    return m_pImpl->ReleaseFrame( frameDesc );
}

QCStatus_e RealSense::Stop()
{
    return m_pImpl->Stop();
}

QCStatus_e RealSense::DeInitialize()
{
    QCStatus_e status = m_pImpl->DeInitialize();

    if ( QC_STATUS_OK == status )
    {
        status = NodeBase::DeInitialize();
    }

    return status;
}

QCObjectState_e RealSense::GetState()
{
    return m_pImpl->GetState();
}

QCNodeConfigIfs &RealSense::GetConfigurationIfs()
{
    return m_configIfs;
}

QCNodeMonitoringIfs &RealSense::GetMonitoringIfs()
{
    return m_monitorIfs;
}

}   // namespace Node
}   // namespace QC
