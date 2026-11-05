// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/Hello.hpp"

namespace QC
{
namespace Node
{

// Register the Hello node with QCNodeFactory at static-initialization time.
const NodeRegistrar<Hello> g_registerHello{ QC_NODE_TYPE_HELLO };

Hello::Hello() : m_configIfs( m_logger ), m_monitorIfs( m_logger ), m_state( QC_OBJECT_STATE_INITIAL )
{}

QCStatus_e Hello::Initialize( QCNodeInit_t &config )
{
    QCStatus_e status = QC_STATUS_OK;
    std::string errors;

    if ( m_state != QC_OBJECT_STATE_INITIAL )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        status = m_configIfs.VerifyAndSet( config.config, errors );
        if ( status != QC_STATUS_OK )
        {
            QC_ERROR( "Config error: %s", errors.c_str() );
        }
    }

    if ( status == QC_STATUS_OK )
    {
        status = NodeBase::Init( m_configIfs.Get().nodeId );
    }

    if ( status == QC_STATUS_OK )
    {
        m_state = QC_OBJECT_STATE_READY;
        QC_INFO( "Hello node initialized" );
    }

    return status;
}

QCStatus_e Hello::Start()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_READY )
    {
        m_state = QC_OBJECT_STATE_RUNNING;
    }
    else
    {
        QC_ERROR( "Hello start failed due to wrong state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCStatus_e Hello::ProcessFrameDescriptor( QCFrameDescriptorNodeIfs & /*frameDesc*/ )
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_RUNNING )
    {
        m_monitorIfs.IncFrameCount();
        QC_INFO( "Hello world from node %s", m_nodeId.name.c_str() );
    }
    else
    {
        QC_ERROR( "Hello not in running state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCStatus_e Hello::Stop()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_RUNNING )
    {
        m_state = QC_OBJECT_STATE_READY;
    }
    else
    {
        QC_ERROR( "Hello stop failed due to wrong state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCStatus_e Hello::DeInitialize()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_READY )
    {
        status = NodeBase::DeInitialize();
        m_state = QC_OBJECT_STATE_INITIAL;
    }
    else
    {
        QC_ERROR( "Hello deinit failed due to wrong state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCObjectState_e Hello::GetState()
{
    return m_state;
}

QCNodeConfigIfs &Hello::GetConfigurationIfs()
{
    return m_configIfs;
}

QCNodeMonitoringIfs &Hello::GetMonitoringIfs()
{
    return m_monitorIfs;
}

}   // namespace Node
}   // namespace QC
