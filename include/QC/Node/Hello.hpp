// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file Hello.hpp
 * @brief The minimal example QCNode. It does no real work: it only counts the frames it receives.
 */

#ifndef QRB_QC_NODE_HELLO_HPP
#define QRB_QC_NODE_HELLO_HPP

#include "QC/Node/NodeBase.hpp"

namespace QC
{
namespace Node
{

/** @brief The node type used to register the Hello node with QCNodeFactory. */
#define QC_NODE_TYPE_HELLO QC_NODE_TYPE_CUSTOM_0

/** @brief The Hello node configuration. */
typedef struct HelloConfig_s : public QCNodeConfigBase_t
{
} HelloConfig_t;

/** @brief The Hello node monitoring data. */
typedef struct HelloMonitor_s : public QCNodeMonitoringBase_t
{
    uint32_t frameCount; /**< Number of frames processed */
} HelloMonitor_t;

class HelloConfig final : public NodeConfigBase
{
public:
    explicit HelloConfig( Logger &logger );
    ~HelloConfig() = default;

    QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    QCStatus_e GetOptions( std::string &options ) override;
    const QCNodeConfigBase_t &Get() override;

private:
    HelloConfig_t m_config;
    std::string m_options;
};

class HelloMonitoring final : public QCNodeMonitoringIfs
{
public:
    explicit HelloMonitoring( Logger &logger );
    ~HelloMonitoring() = default;

    QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    QCStatus_e GetOptions( std::string &options ) override;
    const QCNodeMonitoringBase_t &Get() override;
    uint32_t GetMaximalSize() override;
    uint32_t GetCurrentSize() override;
    QCStatus_e Place( void *ptr, uint32_t &size ) override;

    void IncFrameCount() { m_data.frameCount++; }

private:
    HelloMonitor_t m_data;
    Logger &m_logger;
};

class Hello final : public NodeBase
{
public:
    Hello();
    ~Hello() = default;

    Hello( const Hello & ) = delete;
    Hello &operator=( const Hello & ) = delete;
    Hello( Hello && ) = delete;
    Hello &operator=( Hello && ) = delete;

    QCStatus_e Initialize( QCNodeInit_t &config ) override;
    QCNodeConfigIfs &GetConfigurationIfs() override;
    QCNodeMonitoringIfs &GetMonitoringIfs() override;
    QCStatus_e Start() override;
    QCStatus_e ProcessFrameDescriptor( QCFrameDescriptorNodeIfs &frameDesc ) override;
    QCStatus_e Stop() override;
    QCStatus_e DeInitialize() override;
    QCObjectState_e GetState() override;

private:
    HelloConfig m_configIfs;
    HelloMonitoring m_monitorIfs;
    QCObjectState_e m_state;
};

}   // namespace Node
}   // namespace QC

#endif   // QRB_QC_NODE_HELLO_HPP
