// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/Demo.hpp"
#include <cstring>

namespace QC
{
namespace Node
{

DemoMonitoring::DemoMonitoring( Logger &logger ) : m_data{}, m_logger( logger ) {}

QCStatus_e DemoMonitoring::VerifyAndSet( const std::string /*config*/, std::string & /*errors*/ )
{
    return QC_STATUS_OK;
}

QCStatus_e DemoMonitoring::GetOptions( std::string &options )
{
    DataTree dt;
    dt.Set<std::string>( "frameCount", "Number of frames processed" );
    options = dt.Dump();
    return QC_STATUS_OK;
}

const QCNodeMonitoringBase_t &DemoMonitoring::Get()
{
    return m_data;
}

uint32_t DemoMonitoring::GetMaximalSize()
{
    return sizeof( DemoMonitor_t );
}

uint32_t DemoMonitoring::GetCurrentSize()
{
    return sizeof( DemoMonitor_t );
}

QCStatus_e DemoMonitoring::Place( void *ptr, uint32_t &size )
{
    QCStatus_e status = QC_STATUS_OK;

    if ( ptr == nullptr )
    {
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else if ( size < sizeof( DemoMonitor_t ) )
    {
        size = sizeof( DemoMonitor_t );
        status = QC_STATUS_OUT_OF_BOUND;
    }
    else
    {
        (void) memcpy( ptr, &m_data, sizeof( DemoMonitor_t ) );
        size = sizeof( DemoMonitor_t );
    }

    return status;
}

}   // namespace Node
}   // namespace QC
