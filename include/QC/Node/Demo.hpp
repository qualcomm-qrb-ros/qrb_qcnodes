// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file Demo.hpp
 * @brief The minimal example QCNode.
 *
 * Demo reads one QRB::Memory::OdometryDescriptor_t from the buffer 0 of the frame descriptor and
 * prints the frames, the position and the linear velocity it carries.
 *
 * Example static configuration:
 * @code
 * { "static": { "name": "demo0", "id": 0 } }
 * @endcode
 */

#ifndef QRB_QC_NODE_DEMO_HPP
#define QRB_QC_NODE_DEMO_HPP

#include "QC/Node/NodeBase.hpp"

namespace QC
{
namespace Node
{

/** @brief The node type used to register the Demo node with QCNodeFactory. */
#define QC_NODE_TYPE_DEMO QC_NODE_TYPE_CUSTOM_0

/** @brief The Demo node configuration. */
typedef struct DemoConfig_s : public QCNodeConfigBase_t
{
} DemoConfig_t;

/** @brief The Demo node monitoring data. */
typedef struct DemoMonitor_s : public QCNodeMonitoringBase_t
{
    uint32_t frameCount; /**< Number of frames processed */
} DemoMonitor_t;

class DemoConfig final : public NodeConfigBase
{
public:
    explicit DemoConfig( Logger &logger );
    ~DemoConfig() = default;

    QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    QCStatus_e GetOptions( std::string &options ) override;
    const QCNodeConfigBase_t &Get() override;

private:
    DemoConfig_t m_config;
    std::string m_options;
};

class DemoMonitoring final : public QCNodeMonitoringIfs
{
public:
    explicit DemoMonitoring( Logger &logger );
    ~DemoMonitoring() = default;

    QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    QCStatus_e GetOptions( std::string &options ) override;
    const QCNodeMonitoringBase_t &Get() override;
    uint32_t GetMaximalSize() override;
    uint32_t GetCurrentSize() override;
    QCStatus_e Place( void *ptr, uint32_t &size ) override;

    void IncFrameCount() { m_data.frameCount++; }
    uint32_t FrameCount() const { return m_data.frameCount; }

private:
    DemoMonitor_t m_data;
    Logger &m_logger;
};

class Demo final : public NodeBase
{
public:
    Demo();
    ~Demo() = default;

    Demo( const Demo & ) = delete;
    Demo &operator=( const Demo & ) = delete;
    Demo( Demo && ) = delete;
    Demo &operator=( Demo && ) = delete;

    QCStatus_e Initialize( QCNodeInit_t &config ) override;
    QCNodeConfigIfs &GetConfigurationIfs() override;
    QCNodeMonitoringIfs &GetMonitoringIfs() override;
    QCStatus_e Start() override;
    QCStatus_e ProcessFrameDescriptor( QCFrameDescriptorNodeIfs &frameDesc ) override;
    QCStatus_e Stop() override;
    QCStatus_e DeInitialize() override;
    QCObjectState_e GetState() override;

private:
    /** @brief The global buffer id of the input OdometryDescriptor_t in the frame descriptor. */
    static constexpr uint32_t kInputBufferId = 0;

    DemoConfig m_configIfs;
    DemoMonitoring m_monitorIfs;
    QCObjectState_e m_state;
};

}   // namespace Node
}   // namespace QC

#endif   // QRB_QC_NODE_DEMO_HPP
