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

QCStatus_e RealSense::GetDeviceInfo( RealSenseDeviceInfo_t &info )
{
    return m_pImpl->GetDeviceInfo( info );
}

QCStatus_e RealSense::GetCameraInfo( QCRealSenseStream_e stream, RealSenseCameraInfo_t &info )
{
    return m_pImpl->GetCameraInfo( stream, info );
}

QCStatus_e RealSense::GetExtrinsics( QCRealSenseStream_e from, QCRealSenseStream_e to,
                                     RealSenseExtrinsics_t &extrinsics )
{
    return m_pImpl->GetExtrinsics( from, to, extrinsics );
}

QCStatus_e RealSense::GetImuInfo( QCRealSenseStream_e stream, RealSenseImuInfo_t &info )
{
    return m_pImpl->GetImuInfo( stream, info );
}

QCStatus_e
RealSense::GetStaticTransforms( std::vector<QRB::Memory::TransformStamped_t> &transforms )
{
    return m_pImpl->GetStaticTransforms( transforms );
}

QCStatus_e RealSense::GetStaticTransforms( QRB::Memory::TFMessageDescriptor_t &tf )
{
    std::vector<QRB::Memory::TransformStamped_t> transforms;
    QCStatus_e status = m_pImpl->GetStaticTransforms( transforms );

    if ( QC_STATUS_OK == status )
    {
        const size_t capacity =
                ( ( nullptr == tf.pBuf ) || ( tf.offset > tf.size ) )
                        ? 0u
                        : ( tf.size - tf.offset ) / sizeof( QRB::Memory::TransformStamped_t );
        if ( capacity < transforms.size() )
        {
            QC_ERROR( "the TF buffer holds %u transforms, %u needed",
                      static_cast<uint32_t>( capacity ),
                      static_cast<uint32_t>( transforms.size() ) );
            status = QC_STATUS_INVALID_BUF;
        }
        else
        {
            QRB::Memory::TransformStamped_t *pOut =
                    static_cast<QRB::Memory::TransformStamped_t *>( tf.GetDataPtr() );
            for ( size_t i = 0; i < transforms.size(); i++ )
            {
                pOut[i] = transforms[i];
            }
            tf.transformCount = static_cast<uint32_t>( transforms.size() );
            tf.validSize = transforms.size() * sizeof( QRB::Memory::TransformStamped_t );
        }
    }

    return status;
}

QCStatus_e RealSense::EnumerateDevices( std::vector<RealSenseDeviceInfo_t> &devices )
{
    return RealSenseImpl::EnumerateDevices( devices );
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
