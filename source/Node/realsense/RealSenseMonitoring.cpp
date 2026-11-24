// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/RealSense.hpp"

#include <cstring>

namespace QC
{
namespace Node
{

RealSenseMonitoring::RealSenseMonitoring( Logger &logger ) : m_data(), m_logger( logger )
{
    Reset();
}

QCStatus_e RealSenseMonitoring::VerifyAndSet( const std::string config, std::string &errors )
{
    (void) config;
    (void) errors;
    return QC_STATUS_OK;
}

QCStatus_e RealSenseMonitoring::GetOptions( std::string &options )
{
    options = "";
    return QC_STATUS_OK;
}

void RealSenseMonitoring::OnColor( uint64_t timestampNs )
{
    m_colorFrameCount++;
    m_lastTimestampNs = timestampNs;
}

void RealSenseMonitoring::OnDepth( uint64_t timestampNs )
{
    m_depthFrameCount++;
    m_lastTimestampNs = timestampNs;
}

void RealSenseMonitoring::OnImu( uint64_t timestampNs )
{
    m_imuSampleCount++;
    m_lastTimestampNs = timestampNs;
}

void RealSenseMonitoring::Reset()
{
    m_colorFrameCount = 0;
    m_depthFrameCount = 0;
    m_imuSampleCount = 0;
    m_colorDropCount = 0;
    m_depthDropCount = 0;
    m_timeoutCount = 0;
    m_errorCount = 0;
    m_lastTimestampNs = 0;
    Snapshot();
}

void RealSenseMonitoring::Snapshot()
{
    m_data.colorFrameCount = m_colorFrameCount;
    m_data.depthFrameCount = m_depthFrameCount;
    m_data.imuSampleCount = m_imuSampleCount;
    m_data.colorDropCount = m_colorDropCount;
    m_data.depthDropCount = m_depthDropCount;
    m_data.timeoutCount = m_timeoutCount;
    m_data.errorCount = m_errorCount;
    m_data.lastTimestampNs = m_lastTimestampNs;
}

const QCNodeMonitoringBase_t &RealSenseMonitoring::Get()
{
    Snapshot();
    return m_data;
}

uint32_t RealSenseMonitoring::GetMaximalSize()
{
    return static_cast<uint32_t>( sizeof( RealSenseMonitor_t ) );
}

uint32_t RealSenseMonitoring::GetCurrentSize()
{
    return static_cast<uint32_t>( sizeof( RealSenseMonitor_t ) );
}

QCStatus_e RealSenseMonitoring::Place( void *ptr, uint32_t &size )
{
    QCStatus_e status = QC_STATUS_OK;

    if ( nullptr == ptr )
    {
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else if ( size < sizeof( RealSenseMonitor_t ) )
    {
        size = static_cast<uint32_t>( sizeof( RealSenseMonitor_t ) );
        status = QC_STATUS_OUT_OF_BOUND;
    }
    else
    {
        Snapshot();
        (void) memcpy( ptr, &m_data, sizeof( RealSenseMonitor_t ) );
        size = static_cast<uint32_t>( sizeof( RealSenseMonitor_t ) );
    }

    return status;
}

}   // namespace Node
}   // namespace QC
