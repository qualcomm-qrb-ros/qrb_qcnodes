// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/RealSense.hpp"

#include <cmath>

namespace QC
{
namespace Node
{

RealSenseConfig::RealSenseConfig( Logger &logger ) : NodeConfigBase( logger ) {}

QCStatus_e RealSenseConfig::VerifyAndSet( const std::string config, std::string &errors )
{
    QCStatus_e status = NodeConfigBase::VerifyAndSet( config, errors );
    DataTree dt;

    if ( QC_STATUS_OK == status )
    {
        status = m_dataTree.Get( "static", dt );
        if ( QC_STATUS_OK != status )
        {
            errors += "no static section, ";
        }
    }

    if ( QC_STATUS_OK == status )
    {
        RealSenseConfig_t defaults;
        RealSenseConfig_t cfg;
        const uint32_t id = dt.Get<uint32_t>( "id", UINT32_MAX );
        const std::string cameraName = dt.Get<std::string>( "cameraName", "camera" );
        const std::string clock = dt.Get<std::string>( "clock", "realtime" );
        const std::string uniteImu = dt.Get<std::string>( "uniteImuMethod", "copy" );

        cfg.serial = dt.Get<std::string>( "serial", "" );
        cfg.enableColor = dt.Get<bool>( "enableColor", defaults.enableColor );
        cfg.colorWidth = dt.Get<uint32_t>( "colorWidth", defaults.colorWidth );
        cfg.colorHeight = dt.Get<uint32_t>( "colorHeight", defaults.colorHeight );
        cfg.colorFps = dt.Get<uint32_t>( "colorFps", defaults.colorFps );
        cfg.enableDepth = dt.Get<bool>( "enableDepth", defaults.enableDepth );
        cfg.depthWidth = dt.Get<uint32_t>( "depthWidth", defaults.depthWidth );
        cfg.depthHeight = dt.Get<uint32_t>( "depthHeight", defaults.depthHeight );
        cfg.depthFps = dt.Get<uint32_t>( "depthFps", defaults.depthFps );
        cfg.alignDepth = dt.Get<bool>( "alignDepth", defaults.alignDepth );
        cfg.enableImu = dt.Get<bool>( "enableImu", defaults.enableImu );
        cfg.accelFps = dt.Get<uint32_t>( "accelFps", defaults.accelFps );
        cfg.gyroFps = dt.Get<uint32_t>( "gyroFps", defaults.gyroFps );
        cfg.linearAccelCov = dt.Get<double>( "linearAccelCov", defaults.linearAccelCov );
        cfg.angularVelocityCov =
                dt.Get<double>( "angularVelocityCov", defaults.angularVelocityCov );
        cfg.timeoutMs = dt.Get<uint32_t>( "timeoutMs", defaults.timeoutMs );

        if ( UINT32_MAX == id )
        {
            errors += "the id is empty, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( ( false == cfg.enableColor ) && ( false == cfg.enableDepth ) &&
             ( false == cfg.enableImu ) )
        {
            errors += "no stream is enabled, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.enableColor &&
             ( ( 0 == cfg.colorWidth ) || ( 0 == cfg.colorHeight ) || ( 0 == cfg.colorFps ) ) )
        {
            errors += "invalid color resolution or fps, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.enableDepth &&
             ( ( 0 == cfg.depthWidth ) || ( 0 == cfg.depthHeight ) || ( 0 == cfg.depthFps ) ) )
        {
            errors += "invalid depth resolution or fps, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.alignDepth && ( ( false == cfg.enableColor ) || ( false == cfg.enableDepth ) ) )
        {
            errors += "alignDepth needs both the color and the depth stream, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( ( false == std::isfinite( cfg.linearAccelCov ) ) || ( cfg.linearAccelCov < 0.0 ) ||
             ( false == std::isfinite( cfg.angularVelocityCov ) ) ||
             ( cfg.angularVelocityCov < 0.0 ) )
        {
            errors += "IMU covariances must be finite and not negative, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( "realtime" == clock )
        {
            cfg.clock = QC_REALSENSE_CLOCK_REALTIME;
        }
        else if ( "monotonic" == clock )
        {
            cfg.clock = QC_REALSENSE_CLOCK_MONOTONIC;
        }
        else
        {
            errors += "clock must be realtime or monotonic, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( "copy" == uniteImu )
        {
            cfg.uniteImuMethod = QC_REALSENSE_UNITE_IMU_COPY;
        }
        else if ( "linear_interpolation" == uniteImu )
        {
            cfg.uniteImuMethod = QC_REALSENSE_UNITE_IMU_LINEAR_INTERPOLATION;
        }
        else
        {
            errors += "uniteImuMethod must be copy or linear_interpolation, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( 0 == cfg.timeoutMs )
        {
            errors += "invalid timeoutMs, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }

        if ( QC_STATUS_OK == status )
        {
            // The realsense2_camera frame names. An aligned depth image is in the color frame.
            cfg.colorFrameId = dt.Get<std::string>( "colorFrameId", "" );
            if ( cfg.colorFrameId.empty() )
            {
                cfg.colorFrameId = cameraName + "_color_optical_frame";
            }
            cfg.depthFrameId = dt.Get<std::string>( "depthFrameId", "" );
            if ( cfg.depthFrameId.empty() )
            {
                cfg.depthFrameId =
                        cfg.alignDepth ? cfg.colorFrameId : cameraName + "_depth_optical_frame";
            }
            cfg.imuFrameId = dt.Get<std::string>( "imuFrameId", "" );
            if ( cfg.imuFrameId.empty() )
            {
                cfg.imuFrameId = cameraName + "_imu_optical_frame";
            }

            cfg.nodeId.name = dt.Get<std::string>( "name", "" );
            cfg.nodeId.id = static_cast<uint8_t>( id );
            cfg.nodeId.type = QC_NODE_TYPE_REALSENSE;
            m_config = cfg;
        }
    }

    return status;
}

QCStatus_e RealSenseConfig::GetOptions( std::string &options )
{
    if ( m_options.empty() )
    {
        DataTree dt;
        dt.Set<std::string>( "description", "RealSense camera node configuration options" );
        dt.Set<std::string>( "static.name", "Node name" );
        dt.Set<std::string>( "static.id", "Node id" );
        dt.Set<std::string>( "static.serial",
                             "Camera serial number, empty means the first camera found" );
        dt.Set<std::string>( "static.cameraName",
                             "Prefix of the default frame ids, default camera" );
        dt.Set<std::string>( "static.enableColor", "Deliver the rgb8 color stream, default true" );
        dt.Set<std::string>( "static.colorWidth", "Color width, default 640" );
        dt.Set<std::string>( "static.colorHeight", "Color height, default 480" );
        dt.Set<std::string>( "static.colorFps", "Color frame rate, default 30" );
        dt.Set<std::string>( "static.enableDepth", "Deliver the 16UC1 depth stream, default true" );
        dt.Set<std::string>( "static.depthWidth", "Depth width, default 640" );
        dt.Set<std::string>( "static.depthHeight", "Depth height, default 480" );
        dt.Set<std::string>( "static.depthFps", "Depth frame rate, default 30" );
        dt.Set<std::string>( "static.alignDepth", "Align depth to color, default false" );
        dt.Set<std::string>( "static.enableImu", "Deliver the IMU samples, default false" );
        dt.Set<std::string>( "static.accelFps", "Accel rate, default 0 (camera default)" );
        dt.Set<std::string>( "static.gyroFps", "Gyro rate, default 0 (camera default)" );
        dt.Set<std::string>( "static.uniteImuMethod",
                             "copy (default) or linear_interpolation of the accel samples" );
        dt.Set<std::string>( "static.linearAccelCov", "Linear acceleration covariance, 0.01" );
        dt.Set<std::string>( "static.angularVelocityCov", "Angular velocity covariance, 0.01" );
        dt.Set<std::string>( "static.clock", "Timestamp clock, realtime (default) or monotonic" );
        dt.Set<std::string>( "static.colorFrameId", "Default <cameraName>_color_optical_frame" );
        dt.Set<std::string>( "static.depthFrameId", "Default <cameraName>_depth_optical_frame" );
        dt.Set<std::string>( "static.imuFrameId", "Default <cameraName>_imu_optical_frame" );
        dt.Set<std::string>( "static.timeoutMs", "Time without any sample, default 1000" );
        m_options = dt.Dump();
    }
    options = m_options;
    return QC_STATUS_OK;
}

const QCNodeConfigBase_t &RealSenseConfig::Get()
{
    return m_config;
}

}   // namespace Node
}   // namespace QC
