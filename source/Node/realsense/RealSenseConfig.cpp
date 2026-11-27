// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "QC/Node/RealSense.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>

namespace QC
{
namespace Node
{

namespace
{

std::string Trim( const std::string &text )
{
    const size_t first = text.find_first_not_of( " \t" );
    const size_t last = text.find_last_not_of( " \t" );
    return ( std::string::npos == first ) ? std::string() : text.substr( first, last - first + 1 );
}

bool IsColorEncoding( const std::string &encoding )
{
    return ( QC_REALSENSE_ENCODING_RGB8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_BGR8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_RGBA8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_BGRA8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_YUYV == encoding );
}

bool IsTextureEncoding( const std::string &encoding )
{
    return ( QC_REALSENSE_ENCODING_RGB8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_BGR8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_RGBA8 == encoding ) ||
           ( QC_REALSENSE_ENCODING_BGRA8 == encoding );
}

}   // namespace

RealSenseConfig::RealSenseConfig( Logger &logger ) : NodeConfigBase( logger ) {}

bool RealSenseConfig::ParseSensorOptions( const std::string &text,
                                          std::vector<RealSenseOption_t> &options,
                                          std::string &errors )
{
    bool bOk = true;
    size_t start = 0;

    options.clear();
    while ( start <= text.size() )
    {
        size_t end = text.find_first_of( ",;", start );
        if ( std::string::npos == end )
        {
            end = text.size();
        }
        const std::string item = Trim( text.substr( start, end - start ) );
        start = end + 1;
        if ( item.empty() )
        {
            continue;
        }

        const size_t dot = item.find( '.' );
        const size_t eq = item.find( '=' );
        RealSenseOption_t option;
        if ( ( std::string::npos == dot ) || ( std::string::npos == eq ) || ( dot > eq ) )
        {
            errors += "sensor option '" + item + "' is not <module>.<option>=<value>, ";
            bOk = false;
            continue;
        }
        option.module = Trim( item.substr( 0, dot ) );
        option.option = Trim( item.substr( dot + 1, eq - dot - 1 ) );
        const std::string value = Trim( item.substr( eq + 1 ) );
        char *pEnd = nullptr;
        errno = 0;
        const float number = std::strtof( value.c_str(), &pEnd );
        if ( option.module.empty() || option.option.empty() || value.empty() ||
             ( nullptr == pEnd ) || ( '\0' != *pEnd ) || ( 0 != errno ) ||
             ( false == std::isfinite( number ) ) )
        {
            errors += "sensor option '" + item + "' is not <module>.<option>=<number>, ";
            bOk = false;
            continue;
        }
        option.value = number;
        options.push_back( option );
    }

    return bOk;
}

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
        const std::string clock = dt.Get<std::string>( "clock", "realtime" );
        const std::string uniteImu = dt.Get<std::string>( "uniteImuMethod", "copy" );
        const std::string texture = dt.Get<std::string>( "pointcloudTexture", "color" );
        const bool bImu = dt.Get<bool>( "enableImu", false );

        cfg.serial = dt.Get<std::string>( "serial", "" );
        cfg.cameraName = dt.Get<std::string>( "cameraName", defaults.cameraName );
        cfg.baseFrameId = dt.Get<std::string>( "baseFrameId", "" );
        cfg.enableColor = dt.Get<bool>( "enableColor", defaults.enableColor );
        cfg.colorWidth = dt.Get<uint32_t>( "colorWidth", defaults.colorWidth );
        cfg.colorHeight = dt.Get<uint32_t>( "colorHeight", defaults.colorHeight );
        cfg.colorFps = dt.Get<uint32_t>( "colorFps", defaults.colorFps );
        cfg.colorEncoding = dt.Get<std::string>( "colorEncoding", defaults.colorEncoding );
        cfg.enableDepth = dt.Get<bool>( "enableDepth", defaults.enableDepth );
        cfg.depthWidth = dt.Get<uint32_t>( "depthWidth", defaults.depthWidth );
        cfg.depthHeight = dt.Get<uint32_t>( "depthHeight", defaults.depthHeight );
        cfg.depthFps = dt.Get<uint32_t>( "depthFps", defaults.depthFps );
        cfg.enableInfra1 = dt.Get<bool>( "enableInfra1", defaults.enableInfra1 );
        cfg.enableInfra2 = dt.Get<bool>( "enableInfra2", defaults.enableInfra2 );
        cfg.infraWidth = dt.Get<uint32_t>( "infraWidth", defaults.infraWidth );
        cfg.infraHeight = dt.Get<uint32_t>( "infraHeight", defaults.infraHeight );
        cfg.infraFps = dt.Get<uint32_t>( "infraFps", defaults.infraFps );
        cfg.infraEncoding = dt.Get<std::string>( "infraEncoding", defaults.infraEncoding );
        cfg.alignDepth = dt.Get<bool>( "alignDepth", defaults.alignDepth );
        cfg.enablePointcloud = dt.Get<bool>( "enablePointcloud", defaults.enablePointcloud );
        cfg.orderedPointcloud = dt.Get<bool>( "orderedPointcloud", defaults.orderedPointcloud );
        cfg.allowNoTexturePoints =
                dt.Get<bool>( "allowNoTexturePoints", defaults.allowNoTexturePoints );
        cfg.enableSync = dt.Get<bool>( "enableSync", defaults.enableSync );
        cfg.enableAccel = dt.Get<bool>( "enableAccel", bImu );
        cfg.enableGyro = dt.Get<bool>( "enableGyro", bImu );
        cfg.accelFps = dt.Get<uint32_t>( "accelFps", defaults.accelFps );
        cfg.gyroFps = dt.Get<uint32_t>( "gyroFps", defaults.gyroFps );
        cfg.rawImu = dt.Get<bool>( "rawImu", defaults.rawImu );
        cfg.linearAccelCov = dt.Get<double>( "linearAccelCov", defaults.linearAccelCov );
        cfg.angularVelocityCov =
                dt.Get<double>( "angularVelocityCov", defaults.angularVelocityCov );
        cfg.interCamSyncMode = dt.Get<int>( "interCamSyncMode", defaults.interCamSyncMode );
        cfg.syncGroup = dt.Get<std::string>( "syncGroup", defaults.syncGroup );
        cfg.timeoutMs = dt.Get<uint32_t>( "timeoutMs", defaults.timeoutMs );

        if ( UINT32_MAX == id )
        {
            errors += "the id is empty, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.cameraName.empty() )
        {
            errors += "cameraName is empty, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( ( false == cfg.enableColor ) && ( false == cfg.enableDepth ) &&
             ( false == cfg.enableInfra1 ) && ( false == cfg.enableInfra2 ) &&
             ( false == cfg.enableAccel ) && ( false == cfg.enableGyro ) )
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
        if ( false == IsColorEncoding( cfg.colorEncoding ) )
        {
            errors += "colorEncoding must be rgb8, bgr8, rgba8, bgra8 or yuv422_yuy2, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.enableDepth &&
             ( ( 0 == cfg.depthWidth ) || ( 0 == cfg.depthHeight ) || ( 0 == cfg.depthFps ) ) )
        {
            errors += "invalid depth resolution or fps, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( ( cfg.enableInfra1 || cfg.enableInfra2 ) &&
             ( ( 0 == cfg.infraWidth ) || ( 0 == cfg.infraHeight ) || ( 0 == cfg.infraFps ) ) )
        {
            errors += "invalid infra resolution or fps, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( ( QC_REALSENSE_ENCODING_MONO8 != cfg.infraEncoding ) &&
             ( QC_REALSENSE_ENCODING_MONO16 != cfg.infraEncoding ) )
        {
            errors += "infraEncoding must be mono8 or mono16, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.alignDepth && ( ( false == cfg.enableColor ) || ( false == cfg.enableDepth ) ) )
        {
            errors += "alignDepth needs both the color and the depth stream, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( "color" == texture )
        {
            cfg.pointcloudTexture = true;
        }
        else if ( "none" == texture )
        {
            cfg.pointcloudTexture = false;
        }
        else
        {
            errors += "pointcloudTexture must be color or none, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.enablePointcloud && ( false == cfg.enableDepth ) )
        {
            errors += "enablePointcloud needs the depth stream, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.enablePointcloud && cfg.pointcloudTexture &&
             ( ( false == cfg.enableColor ) ||
               ( false == IsTextureEncoding( cfg.colorEncoding ) ) ) )
        {
            errors += "a color pointcloudTexture needs the color stream in rgb8, bgr8, rgba8 or "
                      "bgra8 (or set pointcloudTexture none), ";
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
        if ( "none" == uniteImu )
        {
            cfg.uniteImuMethod = QC_REALSENSE_UNITE_IMU_NONE;
        }
        else if ( "copy" == uniteImu )
        {
            cfg.uniteImuMethod = QC_REALSENSE_UNITE_IMU_COPY;
        }
        else if ( "linear_interpolation" == uniteImu )
        {
            cfg.uniteImuMethod = QC_REALSENSE_UNITE_IMU_LINEAR_INTERPOLATION;
        }
        else
        {
            errors += "uniteImuMethod must be none, copy or linear_interpolation, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( cfg.interCamSyncMode < -1 )
        {
            errors += "interCamSyncMode must be -1 (unchanged) or a camera sync mode, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( 0 == cfg.timeoutMs )
        {
            errors += "invalid timeoutMs, ";
            status = QC_STATUS_BAD_ARGUMENTS;
        }

        std::vector<RealSenseOption_t> sensorOptions;
        if ( false == ParseSensorOptions( dt.Get<std::string>( "sensorOptions", "" ), sensorOptions,
                                          errors ) )
        {
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        // depth_module.inter_cam_sync_mode is the hardware sync role: kept as interCamSyncMode so
        // that the role checks of the sync group apply to it too.
        for ( auto it = sensorOptions.begin(); it != sensorOptions.end(); )
        {
            if ( ( "depth_module" == it->module ) && ( "inter_cam_sync_mode" == it->option ) )
            {
                const int mode = static_cast<int>( std::lround( it->value ) );
                if ( ( -1 != cfg.interCamSyncMode ) && ( mode != cfg.interCamSyncMode ) )
                {
                    errors += "interCamSyncMode and depth_module.inter_cam_sync_mode differ, ";
                    status = QC_STATUS_BAD_ARGUMENTS;
                }
                cfg.interCamSyncMode = mode;
                it = sensorOptions.erase( it );
            }
            else
            {
                ++it;
            }
        }
        cfg.sensorOptions = sensorOptions;

        if ( QC_STATUS_OK == status )
        {
            if ( cfg.baseFrameId.empty() )
            {
                cfg.baseFrameId = cfg.cameraName + "_link";
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
                             "Logical camera name and frame id prefix, default camera" );
        dt.Set<std::string>( "static.baseFrameId", "Default <cameraName>_link" );
        dt.Set<std::string>( "static.enableColor", "Deliver the color stream, default true" );
        dt.Set<std::string>( "static.colorWidth", "Color width, default 640" );
        dt.Set<std::string>( "static.colorHeight", "Color height, default 480" );
        dt.Set<std::string>( "static.colorFps", "Color frame rate, default 30" );
        dt.Set<std::string>( "static.colorEncoding",
                             "rgb8 (default), bgr8, rgba8, bgra8 or yuv422_yuy2" );
        dt.Set<std::string>( "static.enableDepth", "Deliver the 16UC1 depth stream, default true" );
        dt.Set<std::string>( "static.depthWidth", "Depth width, default 640" );
        dt.Set<std::string>( "static.depthHeight", "Depth height, default 480" );
        dt.Set<std::string>( "static.depthFps", "Depth frame rate, default 30" );
        dt.Set<std::string>( "static.enableInfra1", "Deliver the left IR stream, default false" );
        dt.Set<std::string>( "static.enableInfra2", "Deliver the right IR stream, default false" );
        dt.Set<std::string>( "static.infraWidth", "IR width, default 640" );
        dt.Set<std::string>( "static.infraHeight", "IR height, default 480" );
        dt.Set<std::string>( "static.infraFps", "IR frame rate, default 30" );
        dt.Set<std::string>( "static.infraEncoding", "mono8 (default) or mono16" );
        dt.Set<std::string>( "static.alignDepth",
                             "Also deliver the depth aligned to color, default false" );
        dt.Set<std::string>( "static.enablePointcloud", "Deliver the point cloud, default false" );
        dt.Set<std::string>( "static.pointcloudTexture", "color (default) or none" );
        dt.Set<std::string>( "static.orderedPointcloud",
                             "Keep the depth image layout, default false" );
        dt.Set<std::string>( "static.allowNoTexturePoints",
                             "Keep the points outside the color image, default false" );
        dt.Set<std::string>( "static.enableSync", "Synchronized framesets, default false" );
        dt.Set<std::string>( "static.enableAccel", "Enable the accelerometer, default false" );
        dt.Set<std::string>( "static.enableGyro", "Enable the gyroscope, default false" );
        dt.Set<std::string>( "static.enableImu", "Shorthand for enableAccel and enableGyro" );
        dt.Set<std::string>( "static.accelFps", "Accel rate, default 0 (camera default)" );
        dt.Set<std::string>( "static.gyroFps", "Gyro rate, default 0 (camera default)" );
        dt.Set<std::string>( "static.uniteImuMethod",
                             "none, copy (default) or linear_interpolation" );
        dt.Set<std::string>( "static.rawImu",
                             "Also deliver accel and gyro with a united IMU, default false" );
        dt.Set<std::string>( "static.linearAccelCov", "Linear acceleration covariance, 0.01" );
        dt.Set<std::string>( "static.angularVelocityCov", "Angular velocity covariance, 0.01" );
        dt.Set<std::string>( "static.interCamSyncMode",
                             "-1 unchanged (default), 0 default, 1 master, 2 slave, ..." );
        dt.Set<std::string>( "static.syncGroup",
                             "Hardware sync group, one master per group, default default" );
        dt.Set<std::string>( "static.sensorOptions",
                             "<module>.<option>=<value>,... e.g. depth_module.exposure=8500" );
        dt.Set<std::string>( "static.clock", "Timestamp clock, realtime (default) or monotonic" );
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
