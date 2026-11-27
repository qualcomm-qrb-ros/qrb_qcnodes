// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/RealSense.hpp"

#include <time.h>

#include <cstring>

namespace QC
{
namespace Node
{

namespace
{

constexpr uint64_t kNsPerSec = 1000000000u;

uint64_t SteadyNs()
{
    struct timespec ts = {};
    (void) clock_gettime( CLOCK_MONOTONIC, &ts );
    return static_cast<uint64_t>( ts.tv_sec ) * kNsPerSec + static_cast<uint64_t>( ts.tv_nsec );
}

bool ValidStream( QCRealSenseStream_e stream )
{
    return ( stream > QC_REALSENSE_STREAM_NONE ) && ( stream < QC_REALSENSE_STREAM_COUNT );
}

}   // namespace

RealSenseMonitoring::RealSenseMonitoring( Logger &logger ) : m_logger( logger )
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

void RealSenseMonitoring::OnFrame( QCRealSenseStream_e stream, uint64_t timestampNs,
                                   uint64_t frameNumber )
{
    if ( ValidStream( stream ) )
    {
        const uint64_t now = SteadyNs();
        std::lock_guard<std::mutex> lock( m_lock );

        // Frame counter continuity: a frame number that is not the next one is a gap. A smaller
        // or equal one (camera restart, repeated frame) restarts the continuity.
        if ( m_bHaveFrameNumber[stream] && ( frameNumber > m_live.lastFrameNumber[stream] + 1u ) )
        {
            const uint64_t missed = frameNumber - m_live.lastFrameNumber[stream] - 1u;
            m_live.gapCount[stream]++;
            m_live.missedFrames[stream] += missed;
            if ( missed > m_live.maxConsecutiveMissed[stream] )
            {
                m_live.maxConsecutiveMissed[stream] = missed;
            }
        }
        m_bHaveFrameNumber[stream] = true;
        m_live.lastFrameNumber[stream] = frameNumber;
        m_live.frameCount[stream]++;
        m_live.lastTimestampNs[stream] = timestampNs;

        // The measured rate, over windows of at least one second.
        if ( 0u == m_windowStartNs[stream] )
        {
            m_windowStartNs[stream] = now;
            m_windowFrames[stream] = 0u;
        }
        else
        {
            m_windowFrames[stream]++;
            const uint64_t elapsed = now - m_windowStartNs[stream];
            if ( elapsed >= kNsPerSec )
            {
                m_live.measuredFps[stream] = static_cast<double>( m_windowFrames[stream] ) *
                                             static_cast<double>( kNsPerSec ) /
                                             static_cast<double>( elapsed );
                m_windowStartNs[stream] = now;
                m_windowFrames[stream] = 0u;
            }
        }
    }
}

void RealSenseMonitoring::IncDrop( QCRealSenseStream_e stream )
{
    if ( ValidStream( stream ) )
    {
        std::lock_guard<std::mutex> lock( m_lock );
        m_live.dropCount[stream]++;
    }
}

void RealSenseMonitoring::IncTimeout()
{
    std::lock_guard<std::mutex> lock( m_lock );
    m_live.timeoutCount++;
}

void RealSenseMonitoring::IncError()
{
    std::lock_guard<std::mutex> lock( m_lock );
    m_live.errorCount++;
}

void RealSenseMonitoring::IncStartFail()
{
    std::lock_guard<std::mutex> lock( m_lock );
    m_live.startFailCount++;
}

void RealSenseMonitoring::SetConnected( bool bConnected )
{
    std::lock_guard<std::mutex> lock( m_lock );
    if ( m_live.bConnected && ( false == bConnected ) )
    {
        m_live.disconnectCount++;
    }
    m_live.bConnected = bConnected;
}

void RealSenseMonitoring::SetInterCamSyncMode( int32_t mode )
{
    std::lock_guard<std::mutex> lock( m_lock );
    m_live.interCamSyncMode = mode;
}

void RealSenseMonitoring::ResetContinuity()
{
    std::lock_guard<std::mutex> lock( m_lock );
    for ( int s = 0; s < QC_REALSENSE_STREAM_COUNT; s++ )
    {
        m_bHaveFrameNumber[s] = false;
        m_windowStartNs[s] = 0u;
        m_windowFrames[s] = 0u;
        m_live.measuredFps[s] = 0.0;
    }
}

void RealSenseMonitoring::Reset()
{
    std::lock_guard<std::mutex> lock( m_lock );
    for ( int s = 0; s < QC_REALSENSE_STREAM_COUNT; s++ )
    {
        m_live.frameCount[s] = 0u;
        m_live.dropCount[s] = 0u;
        m_live.gapCount[s] = 0u;
        m_live.missedFrames[s] = 0u;
        m_live.maxConsecutiveMissed[s] = 0u;
        m_live.lastTimestampNs[s] = 0u;
        m_live.lastFrameNumber[s] = 0u;
        m_live.measuredFps[s] = 0.0;
        m_windowStartNs[s] = 0u;
        m_windowFrames[s] = 0u;
        m_bHaveFrameNumber[s] = false;
    }
    m_live.timeoutCount = 0u;
    m_live.errorCount = 0u;
    m_live.startFailCount = 0u;
    m_live.disconnectCount = 0u;
    m_live.bConnected = false;
    m_live.interCamSyncMode = -1;
    m_data = m_live;
}

void RealSenseMonitoring::Snapshot()
{
    std::lock_guard<std::mutex> lock( m_lock );
    m_data = m_live;
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
