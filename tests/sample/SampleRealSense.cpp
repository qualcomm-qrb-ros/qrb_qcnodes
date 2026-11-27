// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleRealSense.cpp
 * @brief Sample for the RealSense QCNode, runnable with QCNodeSampleApp (-t RealSense).
 *
 * The sample allocates buffer_num DMA buffers for each enabled image stream and the pointcloud,
 * gives them to the RealSense node and starts the camera. For every sample delivered through the
 * node callback it:
 *  - prints it, if echo is true (like `ros2 topic echo`; one IMU sample out of 100);
 *  - measures the rate of its stream, if hz is true (like `ros2 topic hz`): once a second, and
 *    again at Stop, each enabled stream prints its configured profile, its average rate, the
 *    min/max/std dev of the intervals between the samples as they arrive, and the window, e.g.
 *      rs0: color 640x480    average rate: 30.002  min: 0.032s max: 0.035s std dev: 0.00045s
 *      window: 299   (on one line);
 *  - writes one line per sample to frame_log, if set, for the frame and synchronization checks
 *    (tests/tools/realsense_sync_report.py);
 *  - publishes it on the DataBroker topic of its stream (<stream>_topic), if set, as a
 *    std::shared_ptr<QCBufferDescriptorBase_t> holding the descriptor, for other samples to
 *    consume, and the images as DataFrames_t on <stream>_frames_topic for the QCNode samples
 *    that take images (Recorder, ...). A buffer goes back to the node when the last subscriber
 *    drops it;
 *  - otherwise gives the buffer back to the node at once.
 * With info true it prints, after Initialize, the camera, the CameraInfo of each enabled image
 * stream, the extrinsics and the static transforms (the realsense2_camera latched topics).
 *
 * Attributes (realsense2_camera parameter in brackets):
 *   serial       : the camera serial number, default "" (first camera found) [serial_no]
 *   camera_name  : the logical camera name and frame id prefix, default camera [camera_name]
 *   list_devices : print the connected cameras at Init, default false
 *   info         : print the camera, CameraInfo, extrinsics and TF at Init, default false
 *   color        : enable the color stream, default true [enable_color]
 *   color_width, color_height, color_fps : default 640, 480, 30 [rgb_camera.color_profile]
 *   color_format : rgb8 (default), bgr8, rgba8, bgra8, yuv422_yuy2 [rgb_camera.color_format]
 *   depth        : enable the depth stream, default true [enable_depth]
 *   depth_width, depth_height, depth_fps : default 640, 480, 30 [depth_module.depth_profile]
 *   infra1, infra2 : enable the left / right IR stream, default false [enable_infra1/2]
 *   infra_width, infra_height, infra_fps : default 640, 480, 30 [depth_module.infra_profile]
 *   infra_format : mono8 (default) or mono16 [depth_module.infra1_format]
 *   align_depth  : also deliver the depth aligned to color, default false [align_depth.enable]
 *   pointcloud   : deliver the point cloud, default false [pointcloud.enable]
 *   pointcloud_texture : color (default) or none [pointcloud.stream_filter]
 *   ordered_pc   : organized point cloud, default false [pointcloud.ordered_pc]
 *   allow_no_texture_points : default false [pointcloud.allow_no_texture_points]
 *   enable_sync  : synchronized framesets, default false [enable_sync]
 *   imu          : enable accel and gyro, default false
 *   accel, gyro  : enable the accel / gyro, default the imu value [enable_accel, enable_gyro]
 *   accel_fps, gyro_fps : default 0 (camera default) [accel_fps, gyro_fps]
 *   unite_imu_method : none, copy (default) or linear_interpolation [unite_imu_method 0/1/2]
 *   raw_imu      : with copy or linear_interpolation, also the accel and gyro, default false
 *   inter_cam_sync_mode : -1 unchanged (default), 0 default, 1 master, 2 slave
 *                  [depth_module.inter_cam_sync_mode]
 *   sync_group   : the hardware sync group, one master per group, default "default"
 *   sensor_options : <module>.<option>=<value>,... e.g.
 *                  depth_module.exposure=8500,rgb_camera.enable_auto_exposure=0 [<module>.<option>]
 *   clock        : realtime (default) or monotonic
 *   buffer_num   : buffers per image stream and for the pointcloud, default 4
 *   timeout_ms   : time without any sample reported as a timeout, default 1000
 *   hz           : print the rate of each stream once a second, default false
 *   echo         : print the samples, default true, or false with hz
 *   <stream>_topic : DataBroker topic of the descriptors of a stream, for color, depth,
 *                  aligned_depth, infra1, infra2, pointcloud, imu, accel and gyro, default ""
 *   <stream>_frames_topic : DataBroker topic of DataFrames_t, for color, depth, aligned_depth,
 *                  infra1 and infra2: rgb8 / bgr8 color as a QC ImageDescriptor_t RGB888 /
 *                  BGR888 sharing the buffer, the others as a BufferDescriptor_t; frameId the
 *                  librealsense frame number, default "" (no publish)
 *   imu_file     : write every IMU sample to this CSV file, default "" (none):
 *                  index,frame_number,timestamp_ns,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,stream
 *   frame_log    : write every sample to this CSV file, default "" (none):
 *                  camera_name,serial,stream,frame_number,timestamp_ns,sensor_timestamp_ms,
 *                  timestamp_domain,metadata_sensor_timestamp_us,host_receive_ns
 *   log_level    : the node log level, default ERROR
 *
 * Examples:
 *   # print color and depth frames
 *   ./bin/qcrun ./bin/QCNodeSampleApp -n rs0 -t RealSense -T 10
 *
 *   # the camera calibration and TF, as the realsense2_camera latched topics
 *   ./bin/qcrun ./bin/QCNodeSampleApp -n rs0 -t RealSense -T 1 -k info -v true -k infra1 -v true \
 *     -k infra2 -v true -k imu -v true
 *
 *   # color, aligned depth, IR, pointcloud and IMU with their rates
 *   ./bin/qcrun ./bin/QCNodeSampleApp -n rs0 -t RealSense -k align_depth -v true \
 *     -k infra1 -v true -k infra2 -v true -k pointcloud -v true -k imu -v true \
 *     -k gyro_fps -v 200 -k accel_fps -v 200 -k unite_imu_method -v linear_interpolation -k hz -v
 * true
 *
 *   # two hardware-synchronized D455, master and slave, logged for the sync report
 *   ./bin/qcrun ./bin/QCNodeSampleApp -T 60 \
 *     -n cam1 -t RealSense -k serial -v <SN1> -k camera_name -v cam1 -k inter_cam_sync_mode -v 1 \
 *       -k depth -v false -k infra1 -v true -k infra2 -v true -k hz -v true -k frame_log -v
 * /tmp/cam1.csv \
 *     -n cam2 -t RealSense -k serial -v <SN2> -k camera_name -v cam2 -k inter_cam_sync_mode -v 2 \
 *       -k depth -v false -k infra1 -v true -k infra2 -v true -k hz -v true -k frame_log -v
 * /tmp/cam2.csv
 */

#include "QC/Node/RealSense.hpp"
#include "QC/sample/BufferManager.hpp"
#include "QC/sample/SampleIF.hpp"
#include "QRB/Memory/Sensor/ImuDescriptor.hpp"
#include "QRB/Memory/Sensor/PointCloudDescriptor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace QC
{
namespace sample
{

using namespace QC::Node;
using QRB::Memory::ImuDescriptor_t;
using QRB::Memory::PointCloudDescriptor_t;

/**
 * @brief What the sample publishes over the DataBroker: the descriptor, kept alive with its buffer
 * for as long as a reference is.
 */
typedef std::shared_ptr<QCBufferDescriptorBase_t> DescriptorRef_t;

class SampleRealSense : public SampleIF
{
public:
    SampleRealSense() = default;
    ~SampleRealSense() = default;

    QCStatus_e Init( std::string name, SampleConfig_t &config ) override;
    QCStatus_e Start() override;
    QCStatus_e Stop() override;
    QCStatus_e Deinit() override;

private:
    /**
     * @brief Gives buffers back to the node. Shared with the published descriptors, which a
     * subscriber may drop after the node is gone: the release is then skipped.
     */
    struct Releaser
    {
        std::mutex lock;
        QCNodeIfs *pNode = nullptr;
        std::atomic<uint32_t> errors{ 0 };

        void Release( QCBufferDescriptorBase_t &bufDesc );
    };

    /**
     * @brief Gives one delivered buffer back to the node when destroyed. Shared by everything
     * published for the sample, so the buffer goes back once, when the last of them is dropped.
     */
    struct ReleaseToken
    {
        ReleaseToken( std::shared_ptr<Releaser> releaserIn, const BufferDescriptor_t &buffer )
            : releaser( std::move( releaserIn ) ),
              descriptor( buffer )
        {}
        ~ReleaseToken() { releaser->Release( descriptor ); }

        std::shared_ptr<Releaser> releaser;
        BufferDescriptor_t descriptor; /* the node matches the buffer by address */
    };

    /** @brief The rate of one stream, from the arrival times of its samples, as ros2 topic hz. */
    struct RateStats
    {
        std::string label; /* the configured profile, e.g. "color 640x480" */
        bool bHaveLast = false;
        std::chrono::steady_clock::time_point last;
        uint64_t intervals = 0;
        double sum = 0.0; /* seconds */
        double sumSq = 0.0;
        double min = 0.0;
        double max = 0.0;

        void Reset();
        void Add( std::chrono::steady_clock::time_point now );
        void Print( const std::string &sampleName ) const;
    };

    /** @brief What the sample does with one stream. */
    struct StreamCtx
    {
        bool bEnabled = false;
        RateStats rate;
        DataPublisher<DescriptorRef_t> pub;
        bool bPub = false;
        DataPublisher<DataFrames_t> framesPub;
        bool bFramesPub = false;
    };

    QCStatus_e AllocateImages( QCRealSenseStream_e stream, const char *encoding, uint32_t width,
                               uint32_t height );
    QCStatus_e AllocateClouds( uint32_t width, uint32_t height, bool bTexture );
    void PrintDevices();
    void PrintInfo();
    void PublishFrames( DataPublisher<DataFrames_t> &pub, const RealSenseImageDescriptor_t &image,
                        const std::shared_ptr<ReleaseToken> &token );
    void OnEvent( const QCNodeEventInfo_t &eventInfo );
    void OnImage( const RealSenseImageDescriptor_t &image );
    void OnCloud( const PointCloudDescriptor_t &cloud );
    void OnImu( const ImuDescriptor_t &imu, QCRealSenseStream_e stream );
    void LogFrame( const RealSenseFrameInfo_t &info );
    void EchoImage( const RealSenseImageDescriptor_t &image );
    void EchoCloud( const PointCloudDescriptor_t &cloud );
    void EchoImu( const ImuDescriptor_t &imu, QCRealSenseStream_e stream );
    void OnRate( QCRealSenseStream_e stream );
    void PrintRates();
    void FreeBuffers();

private:
    std::unique_ptr<QCNodeIfs> m_node;
    std::shared_ptr<Releaser> m_releaser;
    BufferManager *m_pBufMgr = nullptr;
    /* Sized once in Init() and never resized after the node got references to the elements. */
    std::vector<RealSenseImageDescriptor_t> m_images;
    std::vector<PointCloudDescriptor_t> m_clouds;

    bool m_bEcho = true;
    bool m_bHz = false;
    bool m_bInfo = false;
    uint32_t m_bufferNum = 4;
    /* Updated from the node callback, which the node serializes, and printed from it. */
    StreamCtx m_streams[QC_REALSENSE_STREAM_COUNT];
    std::chrono::steady_clock::time_point m_lastRatePrint;
    uint64_t m_imuCount = 0;

    /* Written from the node callback, which the node serializes. */
    FILE *m_pImuFile = nullptr;
    FILE *m_pFrameLog = nullptr;

    static constexpr uint64_t kImuEchoEvery = 100;
};

void SampleRealSense::Releaser::Release( QCBufferDescriptorBase_t &bufDesc )
{
    std::lock_guard<std::mutex> guard( lock );
    if ( nullptr != pNode )
    {
        NodeFrameDescriptorFlat<1> frameDesc;
        (void) frameDesc.SetBuffer( 0, bufDesc );
        if ( QC_STATUS_OK != pNode->ProcessFrameDescriptor( frameDesc ) )
        {
            errors++;
        }
    }
}

QCStatus_e SampleRealSense::Init( std::string name, SampleConfig_t &config )
{
    QCStatus_e ret = SampleIF::Init( name, QC_NODE_TYPE_REALSENSE );
    DataTree dt;
    bool bAlign = false;
    bool bPointcloud = false;
    bool bTexture = true;
    uint32_t colorWidth = 640;
    uint32_t colorHeight = 480;
    uint32_t depthWidth = 640;
    uint32_t depthHeight = 480;
    uint32_t infraWidth = 640;
    uint32_t infraHeight = 480;
    std::string colorFormat;
    std::string infraFormat;

    if ( QC_STATUS_OK == ret )
    {
        const bool bImu = Get( config, "imu", false );
        const bool bAccel = Get( config, "accel", bImu );
        const bool bGyro = Get( config, "gyro", bImu );
        const std::string unite = Get( config, "unite_imu_method", "copy" );
        const bool bRawImu = Get( config, "raw_imu", false );
        const bool bUnited = bAccel && bGyro && ( "none" != unite );
        const uint32_t colorFps = Get( config, "color_fps", (uint32_t) 30 );
        const uint32_t depthFps = Get( config, "depth_fps", (uint32_t) 30 );
        const uint32_t infraFps = Get( config, "infra_fps", (uint32_t) 30 );
        const uint32_t gyroFps = Get( config, "gyro_fps", (uint32_t) 0 );
        const uint32_t accelFps = Get( config, "accel_fps", (uint32_t) 0 );

        m_bHz = Get( config, "hz", false );
        m_bEcho = Get( config, "echo", !m_bHz );
        m_bInfo = Get( config, "info", false );
        m_bufferNum = Get( config, "buffer_num", (uint32_t) 4 );
        bAlign = Get( config, "align_depth", false );
        bPointcloud = Get( config, "pointcloud", false );
        bTexture = ( "none" != Get( config, "pointcloud_texture", "color" ) );
        colorWidth = Get( config, "color_width", (uint32_t) 640 );
        colorHeight = Get( config, "color_height", (uint32_t) 480 );
        depthWidth = Get( config, "depth_width", (uint32_t) 640 );
        depthHeight = Get( config, "depth_height", (uint32_t) 480 );
        infraWidth = Get( config, "infra_width", (uint32_t) 640 );
        infraHeight = Get( config, "infra_height", (uint32_t) 480 );
        colorFormat = Get( config, "color_format", QC_REALSENSE_ENCODING_RGB8 );
        infraFormat = Get( config, "infra_format", QC_REALSENSE_ENCODING_MONO8 );

        StreamCtx *s = m_streams;
        s[QC_REALSENSE_STREAM_COLOR].bEnabled = Get( config, "color", true );
        s[QC_REALSENSE_STREAM_DEPTH].bEnabled = Get( config, "depth", true );
        s[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR].bEnabled = bAlign;
        s[QC_REALSENSE_STREAM_INFRA1].bEnabled = Get( config, "infra1", false );
        s[QC_REALSENSE_STREAM_INFRA2].bEnabled = Get( config, "infra2", false );
        s[QC_REALSENSE_STREAM_POINTCLOUD].bEnabled = bPointcloud;
        s[QC_REALSENSE_STREAM_ACCEL].bEnabled = bAccel && ( ( false == bUnited ) || bRawImu );
        s[QC_REALSENSE_STREAM_GYRO].bEnabled = bGyro && ( ( false == bUnited ) || bRawImu );
        s[QC_REALSENSE_STREAM_IMU].bEnabled = bUnited;

        // The configured profiles printed with the measured rates.
        auto res = []( uint32_t w, uint32_t h ) {
            return std::to_string( w ) + "x" + std::to_string( h );
        };
        auto hz = []( const char *label, uint32_t fps ) {
            return ( 0 == fps ) ? std::string( label )
                                : std::string( label ) + " " + std::to_string( fps ) + "Hz";
        };
        s[QC_REALSENSE_STREAM_COLOR].rate.label = "color " + res( colorWidth, colorHeight );
        s[QC_REALSENSE_STREAM_DEPTH].rate.label = "depth " + res( depthWidth, depthHeight );
        s[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR].rate.label =
                "aligned " + res( colorWidth, colorHeight );
        s[QC_REALSENSE_STREAM_INFRA1].rate.label = "infra1 " + res( infraWidth, infraHeight );
        s[QC_REALSENSE_STREAM_INFRA2].rate.label = "infra2 " + res( infraWidth, infraHeight );
        s[QC_REALSENSE_STREAM_POINTCLOUD].rate.label = "pointcloud";
        s[QC_REALSENSE_STREAM_ACCEL].rate.label = hz( "accel", accelFps );
        s[QC_REALSENSE_STREAM_GYRO].rate.label = hz( "gyro", gyroFps );
        s[QC_REALSENSE_STREAM_IMU].rate.label = hz( "imu", gyroFps );

        dt.Set<std::string>( "static.name", name );
        dt.Set<uint32_t>( "static.id", m_nodeId.id );
        dt.Set<std::string>( "static.logLevel", Get( config, "log_level", "ERROR" ) );
        dt.Set<std::string>( "static.serial", Get( config, "serial", "" ) );
        dt.Set<std::string>( "static.cameraName", Get( config, "camera_name", "camera" ) );
        dt.Set<bool>( "static.enableColor", s[QC_REALSENSE_STREAM_COLOR].bEnabled );
        dt.Set<uint32_t>( "static.colorWidth", colorWidth );
        dt.Set<uint32_t>( "static.colorHeight", colorHeight );
        dt.Set<uint32_t>( "static.colorFps", colorFps );
        dt.Set<std::string>( "static.colorEncoding", colorFormat );
        dt.Set<bool>( "static.enableDepth", s[QC_REALSENSE_STREAM_DEPTH].bEnabled );
        dt.Set<uint32_t>( "static.depthWidth", depthWidth );
        dt.Set<uint32_t>( "static.depthHeight", depthHeight );
        dt.Set<uint32_t>( "static.depthFps", depthFps );
        dt.Set<bool>( "static.enableInfra1", s[QC_REALSENSE_STREAM_INFRA1].bEnabled );
        dt.Set<bool>( "static.enableInfra2", s[QC_REALSENSE_STREAM_INFRA2].bEnabled );
        dt.Set<uint32_t>( "static.infraWidth", infraWidth );
        dt.Set<uint32_t>( "static.infraHeight", infraHeight );
        dt.Set<uint32_t>( "static.infraFps", infraFps );
        dt.Set<std::string>( "static.infraEncoding", infraFormat );
        dt.Set<bool>( "static.alignDepth", bAlign );
        dt.Set<bool>( "static.enablePointcloud", bPointcloud );
        dt.Set<std::string>( "static.pointcloudTexture", bTexture ? "color" : "none" );
        dt.Set<bool>( "static.orderedPointcloud", Get( config, "ordered_pc", false ) );
        dt.Set<bool>( "static.allowNoTexturePoints",
                      Get( config, "allow_no_texture_points", false ) );
        dt.Set<bool>( "static.enableSync", Get( config, "enable_sync", false ) );
        dt.Set<bool>( "static.enableAccel", bAccel );
        dt.Set<bool>( "static.enableGyro", bGyro );
        dt.Set<uint32_t>( "static.accelFps", accelFps );
        dt.Set<uint32_t>( "static.gyroFps", gyroFps );
        dt.Set<std::string>( "static.uniteImuMethod", unite );
        dt.Set<bool>( "static.rawImu", bRawImu );
        dt.Set<int32_t>( "static.interCamSyncMode",
                         Get( config, "inter_cam_sync_mode", (int32_t) -1 ) );
        dt.Set<std::string>( "static.syncGroup", Get( config, "sync_group", "default" ) );
        dt.Set<std::string>( "static.sensorOptions", Get( config, "sensor_options", "" ) );
        dt.Set<std::string>( "static.clock", Get( config, "clock", "realtime" ) );
        dt.Set<uint32_t>( "static.timeoutMs", Get( config, "timeout_ms", (uint32_t) 1000 ) );

        if ( ( 0 == m_bufferNum ) || ( m_bufferNum > QC_REALSENSE_MAX_BUFFERS ) )
        {
            QC_ERROR( "buffer_num must be 1 to %u", QC_REALSENSE_MAX_BUFFERS );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }
        if ( Get( config, "list_devices", false ) )
        {
            PrintDevices();
        }

        // The DataBroker topics: <stream>_topic and, for the images, <stream>_frames_topic.
        static const struct
        {
            QCRealSenseStream_e stream;
            const char *key;
            bool bFrames;
        } kTopics[] = { { QC_REALSENSE_STREAM_COLOR, "color", true },
                        { QC_REALSENSE_STREAM_DEPTH, "depth", true },
                        { QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR, "aligned_depth", true },
                        { QC_REALSENSE_STREAM_INFRA1, "infra1", true },
                        { QC_REALSENSE_STREAM_INFRA2, "infra2", true },
                        { QC_REALSENSE_STREAM_POINTCLOUD, "pointcloud", false },
                        { QC_REALSENSE_STREAM_IMU, "imu", false },
                        { QC_REALSENSE_STREAM_ACCEL, "accel", false },
                        { QC_REALSENSE_STREAM_GYRO, "gyro", false } };
        for ( const auto &t : kTopics )
        {
            StreamCtx &ctx = m_streams[t.stream];
            const std::string topic = Get( config, std::string( t.key ) + "_topic", "" );
            if ( ( QC_STATUS_OK == ret ) && ( false == topic.empty() ) )
            {
                ret = ctx.pub.Init( name, topic );
                ctx.bPub = ( QC_STATUS_OK == ret );
            }
            const std::string framesTopic =
                    t.bFrames ? Get( config, std::string( t.key ) + "_frames_topic", "" ) : "";
            if ( ( QC_STATUS_OK == ret ) && ( false == framesTopic.empty() ) )
            {
                ret = ctx.framesPub.Init( name, framesTopic );
                ctx.bFramesPub = ( QC_STATUS_OK == ret );
            }
        }

        const std::string imuFile = Get( config, "imu_file", "" );
        if ( ( QC_STATUS_OK == ret ) && ( false == imuFile.empty() ) )
        {
            m_pImuFile = fopen( imuFile.c_str(), "w" );
            if ( nullptr == m_pImuFile )
            {
                QC_ERROR( "failed to create imu_file %s", imuFile.c_str() );
                ret = QC_STATUS_FAIL;
            }
            else
            {
                fprintf( m_pImuFile, "index,frame_number,timestamp_ns,gyro_x,gyro_y,gyro_z,"
                                     "accel_x,accel_y,accel_z,stream\n" );
            }
        }
        const std::string frameLog = Get( config, "frame_log", "" );
        if ( ( QC_STATUS_OK == ret ) && ( false == frameLog.empty() ) )
        {
            m_pFrameLog = fopen( frameLog.c_str(), "w" );
            if ( nullptr == m_pFrameLog )
            {
                QC_ERROR( "failed to create frame_log %s", frameLog.c_str() );
                ret = QC_STATUS_FAIL;
            }
            else
            {
                fprintf( m_pFrameLog, "camera_name,serial,stream,frame_number,timestamp_ns,"
                                      "sensor_timestamp_ms,timestamp_domain,"
                                      "metadata_sensor_timestamp_us,host_receive_ns\n" );
            }
        }
    }

    /* The DMA buffers the node copies the samples into. The vectors are sized once here and
     * never resized, as the node keeps references to their elements. */
    if ( QC_STATUS_OK == ret )
    {
        m_pBufMgr = BufferManager::Get( m_nodeId );
        if ( nullptr == m_pBufMgr )
        {
            QC_ERROR( "failed to get the buffer manager" );
            ret = QC_STATUS_NOMEM;
        }
    }
    if ( QC_STATUS_OK == ret )
    {
        m_images.clear();
        m_images.reserve( 5 * m_bufferNum );
        m_clouds.clear();
        m_clouds.reserve( m_bufferNum );
        if ( m_streams[QC_REALSENSE_STREAM_COLOR].bEnabled )
        {
            ret = AllocateImages( QC_REALSENSE_STREAM_COLOR, colorFormat.c_str(), colorWidth,
                                  colorHeight );
        }
    }
    if ( ( QC_STATUS_OK == ret ) && m_streams[QC_REALSENSE_STREAM_DEPTH].bEnabled )
    {
        ret = AllocateImages( QC_REALSENSE_STREAM_DEPTH, QC_REALSENSE_ENCODING_16UC1, depthWidth,
                              depthHeight );
    }
    if ( ( QC_STATUS_OK == ret ) && bAlign )
    {
        // An aligned depth image has the color resolution.
        ret = AllocateImages( QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR,
                              QC_REALSENSE_ENCODING_16UC1, colorWidth, colorHeight );
    }
    for ( QCRealSenseStream_e ir : { QC_REALSENSE_STREAM_INFRA1, QC_REALSENSE_STREAM_INFRA2 } )
    {
        if ( ( QC_STATUS_OK == ret ) && m_streams[ir].bEnabled )
        {
            ret = AllocateImages( ir, infraFormat.c_str(), infraWidth, infraHeight );
        }
    }
    if ( ( QC_STATUS_OK == ret ) && bPointcloud )
    {
        ret = AllocateClouds( depthWidth, depthHeight, bTexture );
    }

    if ( QC_STATUS_OK == ret )
    {
        ret = QCNodeFactory::CreateNode( QC_NODE_TYPE_REALSENSE, m_node );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to create the RealSense node: %d", ret );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        QCNodeInit_t nodeInit;
        nodeInit.config = dt.Dump();
        nodeInit.callback = std::bind( &SampleRealSense::OnEvent, this, std::placeholders::_1 );
        for ( RealSenseImageDescriptor_t &image : m_images )
        {
            nodeInit.buffers.push_back( image );
        }
        for ( PointCloudDescriptor_t &cloud : m_clouds )
        {
            nodeInit.buffers.push_back( cloud );
        }

        ret = m_node->Initialize( nodeInit );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to initialize the RealSense node: %d", ret );
        }
    }

    if ( QC_STATUS_OK == ret )
    {
        m_releaser = std::make_shared<Releaser>();
        m_releaser->pNode = m_node.get();
        if ( m_bInfo )
        {
            PrintInfo();
        }
    }
    else
    {
        m_node.reset();
        FreeBuffers();
        for ( FILE **ppFile : { &m_pImuFile, &m_pFrameLog } )
        {
            if ( nullptr != *ppFile )
            {
                (void) fclose( *ppFile );
                *ppFile = nullptr;
            }
        }
    }

    return ret;
}

QCStatus_e SampleRealSense::AllocateImages( QCRealSenseStream_e stream, const char *encoding,
                                            uint32_t width, uint32_t height )
{
    QCStatus_e ret = QC_STATUS_OK;
    const uint32_t step = width * RealSenseEncodingBytesPerPixel( encoding );

    if ( 0 == step )
    {
        QC_ERROR( "%s: unknown encoding %s", RealSenseStreamName( stream ), encoding );
        ret = QC_STATUS_BAD_ARGUMENTS;
    }
    for ( uint32_t i = 0; ( i < m_bufferNum ) && ( QC_STATUS_OK == ret ); i++ )
    {
        m_images.emplace_back();
        RealSenseImageDescriptor_t &image = m_images.back();
        ret = m_pBufMgr->Allocate(
                BufferProps_t( static_cast<size_t>( step ) * height, QC_MEMORY_ALLOCATOR_DMA ),
                image );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to allocate %s buffer %u: %d", RealSenseStreamName( stream ), i,
                      ret );
            m_images.pop_back();
        }
        else
        {
            image.offset = 0;
            image.validSize = 0;
            image.stream = stream;
            image.width = width;
            image.height = height;
            image.step = step;
            (void) image.encoding.Assign( encoding );
        }
    }

    return ret;
}

QCStatus_e SampleRealSense::AllocateClouds( uint32_t width, uint32_t height, bool bTexture )
{
    QCStatus_e ret = QC_STATUS_OK;
    // The largest cloud: an ordered one, one point per depth pixel.
    const size_t size = static_cast<size_t>( width ) * height * RealSensePointStep( bTexture );

    for ( uint32_t i = 0; ( i < m_bufferNum ) && ( QC_STATUS_OK == ret ); i++ )
    {
        m_clouds.emplace_back();
        PointCloudDescriptor_t &cloud = m_clouds.back();
        ret = m_pBufMgr->Allocate( BufferProps_t( size, QC_MEMORY_ALLOCATOR_DMA ), cloud );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to allocate pointcloud buffer %u: %d", i, ret );
            m_clouds.pop_back();
        }
        else
        {
            cloud.offset = 0;
            cloud.validSize = 0;
        }
    }

    return ret;
}

void SampleRealSense::PrintDevices()
{
    std::vector<RealSenseDeviceInfo_t> devices;
    if ( QC_STATUS_OK == RealSense::EnumerateDevices( devices ) )
    {
        printf( "%s: %u RealSense camera(s)\n", m_name.c_str(),
                static_cast<uint32_t>( devices.size() ) );
        for ( const RealSenseDeviceInfo_t &d : devices )
        {
            printf( "%s:   %s serial %s firmware %s usb %s port %s sync mode %d\n", m_name.c_str(),
                    d.name.c_str(), d.serial.c_str(), d.firmware.c_str(), d.usbType.c_str(),
                    d.physicalPort.c_str(), d.interCamSyncMode );
        }
    }
    else
    {
        printf( "%s: failed to enumerate the RealSense cameras\n", m_name.c_str() );
    }
}

void SampleRealSense::PrintInfo()
{
    RealSense *pRs = dynamic_cast<RealSense *>( m_node.get() );
    RealSenseDeviceInfo_t dev;
    const char *n = m_name.c_str();

    if ( ( nullptr == pRs ) || ( QC_STATUS_OK != pRs->GetDeviceInfo( dev ) ) )
    {
        return;
    }
    std::string modules;
    for ( const std::string &m : dev.sensors )
    {
        modules += ( modules.empty() ? "" : ", " ) + m;
    }
    printf( "%s: camera '%s' = %s serial %s firmware %s product %s usb %s port %s\n", n,
            dev.cameraName.c_str(), dev.name.c_str(), dev.serial.c_str(), dev.firmware.c_str(),
            dev.productId.c_str(), dev.usbType.c_str(), dev.physicalPort.c_str() );
    printf( "%s: modules %s, depth unit %f m, sync group %s mode %d\n", n, modules.c_str(),
            static_cast<double>( dev.depthScale ), dev.syncGroup.c_str(), dev.interCamSyncMode );

    for ( int s = QC_REALSENSE_STREAM_COLOR; s < QC_REALSENSE_STREAM_COUNT; s++ )
    {
        const QCRealSenseStream_e stream = static_cast<QCRealSenseStream_e>( s );
        const bool bImu = ( QC_REALSENSE_STREAM_ACCEL == stream ) ||
                          ( QC_REALSENSE_STREAM_GYRO == stream ) ||
                          ( QC_REALSENSE_STREAM_IMU == stream );
        RealSenseCameraInfo_t ci;
        if ( m_streams[s].bEnabled && ( false == bImu ) &&
             ( QC_STATUS_OK == pRs->GetCameraInfo( stream, ci ) ) )
        {
            printf( "%s: camera_info %s %ux%u frame %s %s K [%.3f %.3f %.3f; %.3f %.3f %.3f] "
                    "D [%.5f %.5f %.5f %.5f %.5f] P[3] %.4f\n",
                    n, RealSenseStreamName( stream ), ci.width, ci.height, ci.frameId.CStr(),
                    ci.distortionModel.CStr(), ci.K[0], ci.K[1], ci.K[2], ci.K[3], ci.K[4], ci.K[5],
                    ci.D[0], ci.D[1], ci.D[2], ci.D[3], ci.D[4], ci.P[3] );
        }
    }

    // realsense2_camera extrinsics/depth_to_<stream>.
    for ( QCRealSenseStream_e to :
          { QC_REALSENSE_STREAM_COLOR, QC_REALSENSE_STREAM_INFRA1, QC_REALSENSE_STREAM_INFRA2,
            QC_REALSENSE_STREAM_ACCEL, QC_REALSENSE_STREAM_GYRO } )
    {
        RealSenseExtrinsics_t ex;
        if ( QC_STATUS_OK == pRs->GetExtrinsics( QC_REALSENSE_STREAM_DEPTH, to, ex ) )
        {
            printf( "%s: extrinsics depth_to_%s translation (%.5f, %.5f, %.5f) rotation "
                    "[%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f]\n",
                    n, RealSenseStreamName( to ), static_cast<double>( ex.translation[0] ),
                    static_cast<double>( ex.translation[1] ),
                    static_cast<double>( ex.translation[2] ), static_cast<double>( ex.rotation[0] ),
                    static_cast<double>( ex.rotation[1] ), static_cast<double>( ex.rotation[2] ),
                    static_cast<double>( ex.rotation[3] ), static_cast<double>( ex.rotation[4] ),
                    static_cast<double>( ex.rotation[5] ), static_cast<double>( ex.rotation[6] ),
                    static_cast<double>( ex.rotation[7] ), static_cast<double>( ex.rotation[8] ) );
        }
    }

    for ( QCRealSenseStream_e imu : { QC_REALSENSE_STREAM_ACCEL, QC_REALSENSE_STREAM_GYRO } )
    {
        RealSenseImuInfo_t info;
        if ( ( m_streams[imu].bEnabled || m_streams[QC_REALSENSE_STREAM_IMU].bEnabled ) &&
             ( QC_STATUS_OK == pRs->GetImuInfo( imu, info ) ) )
        {
            printf( "%s: imu_info %s %s noise (%.3g, %.3g, %.3g) bias (%.3g, %.3g, %.3g)\n", n,
                    RealSenseStreamName( imu ), info.bValid ? "calibrated" : "not calibrated",
                    info.noiseVariances[0], info.noiseVariances[1], info.noiseVariances[2],
                    info.biasVariances[0], info.biasVariances[1], info.biasVariances[2] );
        }
    }

    std::vector<QRB::Memory::TransformStamped_t> tfs;
    if ( QC_STATUS_OK == pRs->GetStaticTransforms( tfs ) )
    {
        for ( const QRB::Memory::TransformStamped_t &tf : tfs )
        {
            const auto &t = tf.transform.translation;
            const auto &q = tf.transform.rotation;
            printf( "%s: tf_static %s -> %s translation (%.5f, %.5f, %.5f) rotation (%.5f, %.5f, "
                    "%.5f, %.5f)\n",
                    n, tf.frameId.CStr(), tf.childFrameId.CStr(), t.x, t.y, t.z, q.x, q.y, q.z,
                    q.w );
        }
    }
    fflush( stdout );
}

QCStatus_e SampleRealSense::Start()
{
    m_imuCount = 0;
    for ( StreamCtx &ctx : m_streams )
    {
        ctx.rate.Reset();
    }
    m_lastRatePrint = std::chrono::steady_clock::now();
    QCStatus_e ret = m_node->Start();

    if ( QC_STATUS_OK != ret )
    {
        QC_ERROR( "failed to start the RealSense node: %d", ret );
    }

    return ret;
}

void SampleRealSense::OnEvent( const QCNodeEventInfo_t &eventInfo )
{
    QCBufferDescriptorBase_t &bufDesc = eventInfo.frameDesc.GetBuffer( 0 );
    const auto *pInfo =
            dynamic_cast<const RealSenseFrameInfo_t *>( &eventInfo.frameDesc.GetBuffer( 1 ) );
    const auto *pImage = dynamic_cast<const RealSenseImageDescriptor_t *>( &bufDesc );
    const auto *pCloud = dynamic_cast<const PointCloudDescriptor_t *>( &bufDesc );
    const auto *pImu = dynamic_cast<const ImuDescriptor_t *>( &bufDesc );

    if ( QC_STATUS_OK != eventInfo.status )
    {
        QC_ERROR( "RealSense event error: %d", eventInfo.status );
        printf( "%s: RealSense error event %d (camera disconnected?)\n", m_name.c_str(),
                eventInfo.status );
        return;
    }
    if ( nullptr != pInfo )
    {
        LogFrame( *pInfo );
    }
    if ( nullptr != pImage )
    {
        OnImage( *pImage );
    }
    else if ( nullptr != pCloud )
    {
        OnCloud( *pCloud );
    }
    else if ( ( nullptr != pImu ) && ( nullptr != pInfo ) )
    {
        OnImu( *pImu, pInfo->stream );
    }
    else
    {
        QC_ERROR( "unexpected RealSense event buffer" );
    }
}

void SampleRealSense::LogFrame( const RealSenseFrameInfo_t &info )
{
    if ( nullptr != m_pFrameLog )
    {
        fprintf( m_pFrameLog, "%s,%s,%s,%" PRIu64 ",%" PRIu64 ",%.6f,%d,%s,%" PRIu64 "\n",
                 info.cameraName.CStr(), info.cameraSerial.CStr(),
                 RealSenseStreamName( info.stream ), info.frameNumber, info.timestampNs,
                 info.sensorTimestampMs, static_cast<int>( info.timestampDomain ),
                 info.bHaveMetadataSensorTimestamp
                         ? std::to_string( info.metadataSensorTimestampUs ).c_str()
                         : "",
                 info.hostReceiveNs );
    }
}

void SampleRealSense::OnImage( const RealSenseImageDescriptor_t &image )
{
    StreamCtx &ctx = m_streams[image.stream];

    if ( m_bHz )
    {
        OnRate( image.stream );
    }
    if ( m_bEcho )
    {
        EchoImage( image );
    }

    if ( ctx.bPub || ctx.bFramesPub )
    {
        /* Everything published points to the same buffer, which goes back to the node when the
         * last subscriber drops what it holds. */
        auto token = std::make_shared<ReleaseToken>(
                m_releaser, static_cast<const BufferDescriptor_t &>( image ) );
        if ( ctx.bPub )
        {
            // The deleter holds the token: the image goes back when the descriptor is dropped.
            DescriptorRef_t descriptor( new RealSenseImageDescriptor_t( image ),
                                        [token]( QCBufferDescriptorBase_t *pDesc ) {
                                            (void) token;
                                            delete pDesc;
                                        } );
            (void) ctx.pub.Publish( descriptor );
        }
        if ( ctx.bFramesPub )
        {
            PublishFrames( ctx.framesPub, image, token );
        }
    }
    else
    {
        RealSenseImageDescriptor_t copy( image );
        m_releaser->Release( copy );
    }
}

void SampleRealSense::OnCloud( const PointCloudDescriptor_t &cloud )
{
    StreamCtx &ctx = m_streams[QC_REALSENSE_STREAM_POINTCLOUD];

    if ( m_bHz )
    {
        OnRate( QC_REALSENSE_STREAM_POINTCLOUD );
    }
    if ( m_bEcho )
    {
        EchoCloud( cloud );
    }

    if ( ctx.bPub )
    {
        auto token = std::make_shared<ReleaseToken>(
                m_releaser, static_cast<const BufferDescriptor_t &>( cloud ) );
        DescriptorRef_t descriptor( new PointCloudDescriptor_t( cloud ),
                                    [token]( QCBufferDescriptorBase_t *pDesc ) {
                                        (void) token;
                                        delete pDesc;
                                    } );
        (void) ctx.pub.Publish( descriptor );
    }
    else
    {
        PointCloudDescriptor_t copy( cloud );
        m_releaser->Release( copy );
    }
}

void SampleRealSense::PublishFrames( DataPublisher<DataFrames_t> &pub,
                                     const RealSenseImageDescriptor_t &image,
                                     const std::shared_ptr<ReleaseToken> &token )
{
    SharedBuffer_t *pSharedBuffer = new SharedBuffer_t;
    const bool bRgb = ( image.encoding == QC_REALSENSE_ENCODING_RGB8 );
    const bool bBgr = ( image.encoding == QC_REALSENSE_ENCODING_BGR8 );

    if ( bRgb || bBgr )
    {
        // The color image as the QCNode RGB888 / BGR888 image the image samples take, on the
        // same buffer.
        ImageDescriptor_t qcImage;
        qcImage = static_cast<const BufferDescriptor_t &>( image );
        qcImage.format = bRgb ? QC_IMAGE_FORMAT_RGB888 : QC_IMAGE_FORMAT_BGR888;
        qcImage.batchSize = 1;
        qcImage.width = image.width;
        qcImage.height = image.height;
        qcImage.numPlanes = 1;
        qcImage.stride[0] = image.step;
        qcImage.actualHeight[0] = image.height;
        qcImage.planeBufSize[0] = static_cast<uint32_t>( image.GetImageSize() );
        pSharedBuffer->SetBuffer( qcImage );
    }
    else
    {
        // QCNode has no 16-bit or mono image format: the pixels as a plain buffer.
        BufferDescriptor_t buffer;
        buffer = static_cast<const BufferDescriptor_t &>( image );
        pSharedBuffer->SetBuffer( buffer );
    }

    // The deleter holds the token: the image goes back when the frame is dropped.
    std::shared_ptr<SharedBuffer_t> sharedBuffer( pSharedBuffer, [token]( SharedBuffer_t *pBuf ) {
        (void) token;
        delete pBuf;
    } );
    DataFrame_t frame;
    DataFrames_t frames;
    frame.buffer = sharedBuffer;
    frame.frameId = image.frameNumber;
    frame.timestamp = image.timestampNs;
    frames.Add( frame );
    (void) pub.Publish( frames );
}

void SampleRealSense::OnImu( const ImuDescriptor_t &imu, QCRealSenseStream_e stream )
{
    m_imuCount++;
    if ( m_bHz )
    {
        OnRate( stream );
    }
    if ( m_bEcho && ( 1 == ( m_imuCount % kImuEchoEvery ) ) )
    {
        EchoImu( imu, stream );
    }

    if ( nullptr != m_pImuFile )
    {
        fprintf( m_pImuFile,
                 "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
                 m_imuCount, imu.id, imu.timestampNs, imu.angular_velocity.x,
                 imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x,
                 imu.linear_acceleration.y, imu.linear_acceleration.z,
                 RealSenseStreamName( stream ) );
    }

    StreamCtx &ctx = m_streams[stream];
    if ( ctx.bPub )
    {
        // The delivered sample is valid during the callback only: publish a copy.
        DescriptorRef_t descriptor = std::make_shared<ImuDescriptor_t>( imu );
        (void) ctx.pub.Publish( descriptor );
    }
}

void SampleRealSense::EchoImage( const RealSenseImageDescriptor_t &image )
{
    /* Print the center pixel as a quick sanity check of the image content. */
    const uint32_t bpp = RealSenseEncodingBytesPerPixel( image.encoding.CStr() );
    const uint8_t *pCenter =
            image.GetRowPtr( image.height / 2 ) + static_cast<size_t>( image.width / 2 ) * bpp;
    const uint64_t nsPerSec = 1000000000u;
    char value[64];

    if ( 2u == bpp )
    {
        uint16_t pixel = 0;
        (void) memcpy( &pixel, pCenter, sizeof( pixel ) );
        (void) snprintf( value, sizeof( value ), "%u", static_cast<uint32_t>( pixel ) );
    }
    else if ( 1u == bpp )
    {
        (void) snprintf( value, sizeof( value ), "%u", static_cast<uint32_t>( pCenter[0] ) );
    }
    else
    {
        (void) snprintf( value, sizeof( value ), "(%u, %u, %u)",
                         static_cast<uint32_t>( pCenter[0] ), static_cast<uint32_t>( pCenter[1] ),
                         static_cast<uint32_t>( pCenter[2] ) );
    }
    printf( "%s: %s stamp %" PRIu64 ".%09" PRIu64 " frame %" PRIu64 " %s %ux%u %s step %u, "
            "center %s\n",
            m_name.c_str(), RealSenseStreamName( image.stream ), image.timestampNs / nsPerSec,
            image.timestampNs % nsPerSec, image.frameNumber, image.frameId.CStr(), image.width,
            image.height, image.encoding.CStr(), image.step, value );
}

void SampleRealSense::EchoCloud( const PointCloudDescriptor_t &cloud )
{
    const uint64_t nsPerSec = 1000000000u;
    const uint32_t n = cloud.GetNumPoints();
    float xyz[3] = { 0.0f, 0.0f, 0.0f };

    if ( n > 0u )
    {
        // The middle point, for an ordered cloud the center pixel.
        const uint8_t *pPoint =
                cloud.GetPointPtr( ( cloud.height / 2u ) * cloud.width + cloud.width / 2u );
        (void) memcpy( xyz, pPoint, sizeof( xyz ) );
    }
    printf( "%s: pointcloud stamp %" PRIu64 ".%09" PRIu64 " %s %ux%u points %u step %u%s, middle "
            "point (%.3f, %.3f, %.3f)\n",
            m_name.c_str(), cloud.timestampNs / nsPerSec, cloud.timestampNs % nsPerSec,
            cloud.frameId.CStr(), cloud.width, cloud.height, n, cloud.pointStep,
            ( cloud.numFields > 3u ) ? " xyzrgb" : " xyz", static_cast<double>( xyz[0] ),
            static_cast<double>( xyz[1] ), static_cast<double>( xyz[2] ) );
}

void SampleRealSense::EchoImu( const ImuDescriptor_t &imu, QCRealSenseStream_e stream )
{
    const uint64_t nsPerSec = 1000000000u;
    printf( "%s: %s stamp %" PRIu64 ".%09" PRIu64 " %s angular_velocity (%.4f, %.4f, %.4f) "
            "linear_acceleration (%.4f, %.4f, %.4f), sample %" PRIu64 "\n",
            m_name.c_str(), RealSenseStreamName( stream ), imu.timestampNs / nsPerSec,
            imu.timestampNs % nsPerSec, imu.frameId.CStr(), imu.angular_velocity.x,
            imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x,
            imu.linear_acceleration.y, imu.linear_acceleration.z, m_imuCount );
}

void SampleRealSense::RateStats::Reset()
{
    bHaveLast = false;
    intervals = 0;
    sum = 0.0;
    sumSq = 0.0;
    min = 0.0;
    max = 0.0;
}

void SampleRealSense::RateStats::Add( std::chrono::steady_clock::time_point now )
{
    if ( bHaveLast )
    {
        const double dt = std::chrono::duration<double>( now - last ).count();
        min = ( 0 == intervals ) ? dt : std::min( min, dt );
        max = ( 0 == intervals ) ? dt : std::max( max, dt );
        sum += dt;
        sumSq += dt * dt;
        intervals++;
    }
    last = now;
    bHaveLast = true;
}

void SampleRealSense::RateStats::Print( const std::string &sampleName ) const
{
    if ( intervals > 0 )
    {
        const double mean = sum / static_cast<double>( intervals );
        const double variance =
                std::max( 0.0, sumSq / static_cast<double>( intervals ) - mean * mean );
        printf( "%s: %-16s average rate: %.3f\tmin: %.3fs max: %.3fs std dev: %.5fs "
                "window: %" PRIu64 "\n",
                sampleName.c_str(), label.c_str(), ( mean > 0.0 ) ? 1.0 / mean : 0.0, min, max,
                std::sqrt( variance ), intervals );
    }
}

void SampleRealSense::OnRate( QCRealSenseStream_e stream )
{
    const auto now = std::chrono::steady_clock::now();
    m_streams[stream].rate.Add( now );
    if ( ( now - m_lastRatePrint ) >= std::chrono::seconds( 1 ) )
    {
        m_lastRatePrint = now;
        PrintRates();
    }
}

void SampleRealSense::PrintRates()
{
    for ( const StreamCtx &ctx : m_streams )
    {
        if ( ctx.bEnabled )
        {
            ctx.rate.Print( m_name );
        }
    }
    fflush( stdout );
}

QCStatus_e SampleRealSense::Stop()
{
    QCStatus_e ret = m_node->Stop();

    // Drop the descriptors still queued for the subscribers, giving their buffers back.
    for ( StreamCtx &ctx : m_streams )
    {
        ctx.pub.Clear();
        ctx.framesPub.Clear();
    }
    // The node is stopped: no callback writes any more.
    for ( FILE *pFile : { m_pImuFile, m_pFrameLog } )
    {
        if ( nullptr != pFile )
        {
            fflush( pFile );
        }
    }

    if ( m_bHz )
    {
        // The node is stopped: no callback updates the rates any more.
        PrintRates();
    }

    if ( QC_STATUS_OK == ret )
    {
        RealSenseMonitor_t monitor{};
        uint32_t size = sizeof( monitor );
        if ( QC_STATUS_OK == m_node->GetMonitoringIfs().Place( &monitor, size ) )
        {
            for ( int s = QC_REALSENSE_STREAM_COLOR; s < QC_REALSENSE_STREAM_COUNT; s++ )
            {
                if ( m_streams[s].bEnabled )
                {
                    printf( "%s: %s %" PRIu64 " (dropped %" PRIu64 ", gaps %" PRIu64
                            ", missed %" PRIu64 ", max consecutive missed %" PRIu64 ", fps %.2f)\n",
                            m_name.c_str(),
                            RealSenseStreamName( static_cast<QCRealSenseStream_e>( s ) ),
                            monitor.frameCount[s], monitor.dropCount[s], monitor.gapCount[s],
                            monitor.missedFrames[s], monitor.maxConsecutiveMissed[s],
                            monitor.measuredFps[s] );
                }
            }
            printf( "%s: connected %s, sync mode %d, timeouts %" PRIu64 ", errors %" PRIu64
                    ", disconnects %" PRIu64 ", release errors %u\n",
                    m_name.c_str(), monitor.bConnected ? "yes" : "no", monitor.interCamSyncMode,
                    monitor.timeoutCount, monitor.errorCount, monitor.disconnectCount,
                    m_releaser->errors.load() );
        }
    }
    else
    {
        QC_ERROR( "failed to stop the RealSense node: %d", ret );
    }

    return ret;
}

QCStatus_e SampleRealSense::Deinit()
{
    // A descriptor dropped from now on no longer gives its buffer back.
    {
        std::lock_guard<std::mutex> guard( m_releaser->lock );
        m_releaser->pNode = nullptr;
    }

    QCStatus_e ret = m_node->DeInitialize();
    if ( QC_STATUS_OK != ret )
    {
        QC_ERROR( "failed to deinitialize the RealSense node: %d", ret );
    }

    m_node.reset();
    FreeBuffers();
    for ( FILE **ppFile : { &m_pImuFile, &m_pFrameLog } )
    {
        if ( nullptr != *ppFile )
        {
            (void) fclose( *ppFile );
            *ppFile = nullptr;
        }
    }

    return ret;
}

void SampleRealSense::FreeBuffers()
{
    if ( nullptr != m_pBufMgr )
    {
        for ( RealSenseImageDescriptor_t &image : m_images )
        {
            if ( nullptr != image.pBuf )
            {
                (void) m_pBufMgr->Free( image );
            }
        }
        for ( PointCloudDescriptor_t &cloud : m_clouds )
        {
            if ( nullptr != cloud.pBuf )
            {
                (void) m_pBufMgr->Free( cloud );
            }
        }
        BufferManager::Put( m_pBufMgr );
        m_pBufMgr = nullptr;
    }
    m_images.clear();
    m_clouds.clear();
}

REGISTER_SAMPLE( RealSense, SampleRealSense );

}   // namespace sample
}   // namespace QC
