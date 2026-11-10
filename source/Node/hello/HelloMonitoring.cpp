// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/Hello.hpp"
#include <cstring>

namespace QC
{
namespace Node
{

HelloMonitoring::HelloMonitoring( Logger &logger ) : m_data{}, m_logger( logger ) {}

QCStatus_e HelloMonitoring::VerifyAndSet( const std::string /*config*/, std::string & /*errors*/ )
{
    return QC_STATUS_OK;
}

QCStatus_e HelloMonitoring::GetOptions( std::string &options )
{
    DataTree dt;
    dt.Set<std::string>( "frameCount", "Number of frames processed" );
    options = dt.Dump();
    return QC_STATUS_OK;
}

const QCNodeMonitoringBase_t &HelloMonitoring::Get()
{
    return m_data;
}

uint32_t HelloMonitoring::GetMaximalSize()
{
    return sizeof( HelloMonitor_t );
}

uint32_t HelloMonitoring::GetCurrentSize()
{
    return sizeof( HelloMonitor_t );
}

QCStatus_e HelloMonitoring::Place( void *ptr, uint32_t &size )
{
    QCStatus_e status = QC_STATUS_OK;

    if ( ptr == nullptr )
    {
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else if ( size < sizeof( HelloMonitor_t ) )
    {
        size = sizeof( HelloMonitor_t );
        status = QC_STATUS_OUT_OF_BOUND;
    }
    else
    {
        (void) memcpy( ptr, &m_data, sizeof( HelloMonitor_t ) );
        size = sizeof( HelloMonitor_t );
    }

    return status;
}

}   // namespace Node
}   // namespace QC
