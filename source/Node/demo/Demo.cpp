// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/Demo.hpp"
#include "QRB/Memory/Navigation/OdometryDescriptor.hpp"

namespace QC
{
namespace Node
{

// Register the Demo node with QCNodeFactory at static-initialization time.
const NodeRegistrar<Demo> g_registerDemo{ QC_NODE_TYPE_DEMO };

Demo::Demo() : m_configIfs( m_logger ), m_monitorIfs( m_logger ), m_state( QC_OBJECT_STATE_INITIAL )
{}

QCStatus_e Demo::Initialize( QCNodeInit_t &config )
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
        QC_INFO( "Demo node initialized" );
    }

    return status;
}

QCStatus_e Demo::Start()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_READY )
    {
        m_state = QC_OBJECT_STATE_RUNNING;
    }
    else
    {
        QC_ERROR( "Demo start failed due to wrong state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCStatus_e Demo::ProcessFrameDescriptor( QCFrameDescriptorNodeIfs &frameDesc )
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state != QC_OBJECT_STATE_RUNNING )
    {
        QC_ERROR( "Demo not in running state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    if ( status == QC_STATUS_OK )
    {
        // GetBuffer() returns a dummy descriptor if the id is out of range, in which case the
        // dynamic_cast fails and the frame is rejected.
        const QRB::Memory::OdometryDescriptor_t *pInput =
                dynamic_cast<const QRB::Memory::OdometryDescriptor_t *>(
                        &frameDesc.GetBuffer( kInputBufferId ) );
        if ( pInput == nullptr )
        {
            QC_ERROR( "the input buffer %u is not an OdometryDescriptor", kInputBufferId );
            status = QC_STATUS_INVALID_BUF;
        }
        else
        {
            const QRB::Memory::Point3d_t &position = pInput->poseWithCovariance.pose.position;
            const QRB::Memory::Vector3d_t &linear = pInput->twistWithCovariance.twist.linear;
            m_monitorIfs.IncFrameCount();
            QC_INFO( "Demo: %s -> %s, position (%.3f, %.3f, %.3f), linear velocity "
                     "(%.3f, %.3f, %.3f)",
                     pInput->frameId.CStr(), pInput->childFrameId.CStr(), position.x, position.y,
                     position.z, linear.x, linear.y, linear.z );
            printf( "%s: %s -> %s, position (%.3f, %.3f, %.3f), linear velocity "
                    "(%.3f, %.3f, %.3f)\n",
                    m_nodeId.name.c_str(), pInput->frameId.CStr(), pInput->childFrameId.CStr(),
                    position.x, position.y, position.z, linear.x, linear.y, linear.z );
        }
    }

    return status;
}

QCStatus_e Demo::Stop()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_RUNNING )
    {
        m_state = QC_OBJECT_STATE_READY;
    }
    else
    {
        QC_ERROR( "Demo stop failed due to wrong state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCStatus_e Demo::DeInitialize()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_state == QC_OBJECT_STATE_READY )
    {
        status = NodeBase::DeInitialize();
        m_state = QC_OBJECT_STATE_INITIAL;
    }
    else
    {
        QC_ERROR( "Demo deinit failed due to wrong state: %d", m_state );
        status = QC_STATUS_BAD_STATE;
    }

    return status;
}

QCObjectState_e Demo::GetState()
{
    return m_state;
}

QCNodeConfigIfs &Demo::GetConfigurationIfs()
{
    return m_configIfs;
}

QCNodeMonitoringIfs &Demo::GetMonitoringIfs()
{
    return m_monitorIfs;
}

}   // namespace Node
}   // namespace QC
