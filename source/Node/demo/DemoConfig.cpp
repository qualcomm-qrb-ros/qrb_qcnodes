// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/Demo.hpp"

namespace QC
{
namespace Node
{

DemoConfig::DemoConfig( Logger &logger ) : NodeConfigBase( logger ) {}

QCStatus_e DemoConfig::VerifyAndSet( const std::string config, std::string &errors )
{
    QCStatus_e status = NodeConfigBase::VerifyAndSet( config, errors );
    if ( status == QC_STATUS_OK )
    {
        DataTree dt;
        status = m_dataTree.Get( "static", dt );
        if ( status == QC_STATUS_OK )
        {
            uint32_t id = dt.Get<uint32_t>( "id", UINT32_MAX );
            if ( id == UINT32_MAX )
            {
                errors += "the id is empty, ";
                status = QC_STATUS_BAD_ARGUMENTS;
            }
            else
            {
                m_config.nodeId.name = dt.Get<std::string>( "name", "" );
                m_config.nodeId.id = id;
                m_config.nodeId.type = QC_NODE_TYPE_DEMO;
            }
        }
        else
        {
            errors += "no static section, ";
        }
    }
    return status;
}

QCStatus_e DemoConfig::GetOptions( std::string &options )
{
    if ( m_options.empty() )
    {
        DataTree dt;
        dt.Set<std::string>( "description", "Demo node configuration options" );
        dt.Set<std::string>( "name", "Node name" );
        dt.Set<std::string>( "id", "Node id" );
        m_options = dt.Dump();
    }
    options = m_options;
    return QC_STATUS_OK;
}

const QCNodeConfigBase_t &DemoConfig::Get()
{
    return m_config;
}

}   // namespace Node
}   // namespace QC
