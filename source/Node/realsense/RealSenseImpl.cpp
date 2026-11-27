// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "RealSenseImpl.hpp"

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <time.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <iterator>
#include <set>
#include <utility>
#include <vector>

namespace QC
{
namespace Node
{

namespace
{

using QRB::Memory::ImuDescriptor_t;
using QRB::Memory::PointCloudDescriptor_t;
using QRB::Memory::Quaternion_t;
using QRB::Memory::TransformStamped_t;

constexpr int64_t kNsPerMs = 1000000;
constexpr int64_t kNsPerSec = 1000000000;

/* linear_interpolation: gyro samples kept at most while waiting for an accel sample, so that a
 * stalled accel stream does not grow the queue without bound. */
constexpr size_t kMaxPendingGyros = 64;

/* The streams the node opens on the camera, the others derive from them. */
constexpr QCRealSenseStream_e kPhysicalStreams[] = {
        QC_REALSENSE_STREAM_COLOR,  QC_REALSENSE_STREAM_DEPTH, QC_REALSENSE_STREAM_INFRA1,
        QC_REALSENSE_STREAM_INFRA2, QC_REALSENSE_STREAM_ACCEL, QC_REALSENSE_STREAM_GYRO };

/* The RealSense node whose application callback runs on this thread, to reject Stop() from it:
 * librealsense waits in sensor::stop() for its callbacks to return. */
thread_local const void *tl_pCallbackOwner = nullptr;

/* The hardware sync roles of the nodes of this process, per sync group. */
struct SyncMember
{
    const void *pOwner;
    std::string serial;
    std::string cameraName;
    int mode;
};
std::mutex g_syncLock;
std::map<std::string, std::vector<SyncMember>> g_syncGroups;

int64_t ClockNs( clockid_t clock )
{
    struct timespec ts = {};
    (void) clock_gettime( clock, &ts );
    return static_cast<int64_t>( ts.tv_sec ) * kNsPerSec + static_cast<int64_t>( ts.tv_nsec );
}

/** @brief The DMA-BUF fd of a buffer, or -1 if it has none (e.g. heap memory). */
int DmaBufFd( const QC::Memory::BufferDescriptor_t &buffer )
{
    int fd = -1;
    if ( ( buffer.allocatorType >= QC::Memory::QC_MEMORY_ALLOCATOR_DMA ) &&
         ( buffer.allocatorType < QC::Memory::QC_MEMORY_ALLOCATOR_LAST ) &&
         ( buffer.dmaHandle <= static_cast<uint64_t>( INT_MAX ) ) )
    {
        fd = static_cast<int>( buffer.dmaHandle );
    }
    return fd;
}

/**
 * @brief Begin or end a CPU write of a DMA-BUF, so the data reaches the consumer through the
 * caches. Best effort: a buffer without the sync ioctl needs no cache maintenance.
 */
void SyncDmaBuf( int fd, uint64_t flags )
{
    if ( fd >= 0 )
    {
        struct dma_buf_sync sync = {};
        sync.flags = flags;
        int ret = 0;
        do
        {
            ret = ioctl( fd, DMA_BUF_IOCTL_SYNC, &sync );
        } while ( ( 0 != ret ) && ( ( EINTR == errno ) || ( EAGAIN == errno ) ) );
    }
}

void SetDiagonal( double ( &covariance )[9], double value )
{
    for ( double &element : covariance )
    {
        element = 0.0;
    }
    covariance[0] = value;
    covariance[4] = value;
    covariance[8] = value;
}

/** @brief realsense2_camera create_graph_resource_name: lower case, '_' for other characters. */
std::string GraphName( const std::string &name )
{
    std::string out = name;
    for ( char &c : out )
    {
        const unsigned char u = static_cast<unsigned char>( c );
        c = ( std::isalnum( u ) || ( '_' == c ) ) ? static_cast<char>( std::tolower( u ) ) : '_';
    }
    return out;
}

std::string OptionName( rs2_option option )
{
    const char *pName = rs2_option_to_string( option );
    return GraphName( ( nullptr == pName ) ? "" : pName );
}

/** @brief The realsense2_camera module name of a sensor, e.g. depth_module, rgb_camera. */
std::string ModuleName( const rs2::sensor &sensor )
{
    std::string module;
    if ( sensor.is<rs2::depth_sensor>() )
    {
        module = "depth_module";
    }
    else
    {
        module = GraphName( sensor.supports( RS2_CAMERA_INFO_NAME )
                                    ? sensor.get_info( RS2_CAMERA_INFO_NAME )
                                    : "sensor" );
    }
    return module;
}

std::string DeviceInfoString( const rs2::device &dev, rs2_camera_info info )
{
    return dev.supports( info ) ? std::string( dev.get_info( info ) ) : std::string();
}

rs2_format EncodingFormat( const std::string &encoding )
{
    rs2_format format = RS2_FORMAT_ANY;
    if ( QC_REALSENSE_ENCODING_RGB8 == encoding )
    {
        format = RS2_FORMAT_RGB8;
    }
    else if ( QC_REALSENSE_ENCODING_BGR8 == encoding )
    {
        format = RS2_FORMAT_BGR8;
    }
    else if ( QC_REALSENSE_ENCODING_RGBA8 == encoding )
    {
        format = RS2_FORMAT_RGBA8;
    }
    else if ( QC_REALSENSE_ENCODING_BGRA8 == encoding )
    {
        format = RS2_FORMAT_BGRA8;
    }
    else if ( QC_REALSENSE_ENCODING_YUYV == encoding )
    {
        format = RS2_FORMAT_YUYV;
    }
    else if ( QC_REALSENSE_ENCODING_16UC1 == encoding )
    {
        format = RS2_FORMAT_Z16;
    }
    else if ( QC_REALSENSE_ENCODING_MONO8 == encoding )
    {
        format = RS2_FORMAT_Y8;
    }
    else if ( QC_REALSENSE_ENCODING_MONO16 == encoding )
    {
        format = RS2_FORMAT_Y16;
    }
    else
    {
        /* not a RealSense encoding, rejected by the configuration */
    }
    return format;
}

/* ---- Quaternions (x, y, z, w), as tf2::Quaternion ---- */

Quaternion_t QuatMul( const Quaternion_t &a, const Quaternion_t &b )
{
    Quaternion_t q;
    q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return q;
}

Quaternion_t QuatInverse( const Quaternion_t &a )
{
    const double n = a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w;
    Quaternion_t q;
    q.x = -a.x / n;
    q.y = -a.y / n;
    q.z = -a.z / n;
    q.w = a.w / n;
    return q;
}

/** @brief tf2::Quaternion::setRPY. */
Quaternion_t QuatFromRpy( double roll, double pitch, double yaw )
{
    const double cr = std::cos( roll * 0.5 ), sr = std::sin( roll * 0.5 );
    const double cp = std::cos( pitch * 0.5 ), sp = std::sin( pitch * 0.5 );
    const double cy = std::cos( yaw * 0.5 ), sy = std::sin( yaw * 0.5 );
    Quaternion_t q;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    q.w = cr * cp * cy + sr * sp * sy;
    return q;
}

/** @brief The rotation of a librealsense rotation matrix (column-major), as Eigen does. */
Quaternion_t QuatFromRotation( const float rotation[9] )
{
    double m[3][3];
    for ( int r = 0; r < 3; r++ )
    {
        for ( int c = 0; c < 3; c++ )
        {
            m[r][c] = rotation[c * 3 + r];
        }
    }
    Quaternion_t q;
    const double trace = m[0][0] + m[1][1] + m[2][2];
    if ( trace > 0.0 )
    {
        const double s = std::sqrt( trace + 1.0 ) * 2.0;
        q.w = 0.25 * s;
        q.x = ( m[2][1] - m[1][2] ) / s;
        q.y = ( m[0][2] - m[2][0] ) / s;
        q.z = ( m[1][0] - m[0][1] ) / s;
    }
    else if ( ( m[0][0] > m[1][1] ) && ( m[0][0] > m[2][2] ) )
    {
        const double s = std::sqrt( 1.0 + m[0][0] - m[1][1] - m[2][2] ) * 2.0;
        q.w = ( m[2][1] - m[1][2] ) / s;
        q.x = 0.25 * s;
        q.y = ( m[0][1] + m[1][0] ) / s;
        q.z = ( m[0][2] + m[2][0] ) / s;
    }
    else if ( m[1][1] > m[2][2] )
    {
        const double s = std::sqrt( 1.0 + m[1][1] - m[0][0] - m[2][2] ) * 2.0;
        q.w = ( m[0][2] - m[2][0] ) / s;
        q.x = ( m[0][1] + m[1][0] ) / s;
        q.y = 0.25 * s;
        q.z = ( m[1][2] + m[2][1] ) / s;
    }
    else
    {
        const double s = std::sqrt( 1.0 + m[2][2] - m[0][0] - m[1][1] ) * 2.0;
        q.w = ( m[1][0] - m[0][1] ) / s;
        q.x = ( m[0][2] + m[2][0] ) / s;
        q.y = ( m[1][2] + m[2][1] ) / s;
        q.z = 0.25 * s;
    }
    return q;
}

/** @brief realsense2_camera append_static_tf_msg: the translation from optical to ROS axes. */
TransformStamped_t MakeTransform( uint64_t stampNs, const float translation[3],
                                  const Quaternion_t &rotation, const std::string &from,
                                  const std::string &to )
{
    TransformStamped_t tf;
    tf.timestampNs = stampNs;
    (void) tf.frameId.Assign( from.c_str() );
    (void) tf.childFrameId.Assign( to.c_str() );
    tf.transform.translation.x = translation[2];
    tf.transform.translation.y = -translation[0];
    tf.transform.translation.z = -translation[1];
    tf.transform.rotation = rotation;
    return tf;
}

bool IsImageStream( QCRealSenseStream_e stream )
{
    return ( QC_REALSENSE_STREAM_COLOR == stream ) || ( QC_REALSENSE_STREAM_DEPTH == stream ) ||
           ( QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR == stream ) ||
           ( QC_REALSENSE_STREAM_INFRA1 == stream ) || ( QC_REALSENSE_STREAM_INFRA2 == stream );
}

bool ValidStream( QCRealSenseStream_e stream )
{
    return ( stream > QC_REALSENSE_STREAM_NONE ) && ( stream < QC_REALSENSE_STREAM_COUNT );
}

}   // namespace

RealSenseImpl::RealSenseImpl( QCNodeID_t &nodeId, Logger &logger, RealSenseMonitoring &monitor )
    : m_nodeId( nodeId ),
      m_logger( logger ),
      m_monitor( monitor )
{}

RealSenseImpl::~RealSenseImpl()
{
    StopStreaming();
    Reset();
}

/* ======================================== Initialize ======================================== */

QCStatus_e
RealSenseImpl::Initialize( const RealSenseConfig_t &config, QCNodeEventCallBack_t callback,
                           std::vector<std::reference_wrapper<QCBufferDescriptorBase_t>> &buffers )
{
    QCStatus_e status = QC_STATUS_OK;

    if ( QC_OBJECT_STATE_INITIAL != m_state )
    {
        QC_ERROR( "RealSense init failed due to wrong state: %d", m_state.load() );
        status = QC_STATUS_BAD_STATE;
    }
    else if ( nullptr == callback )
    {
        QC_ERROR( "RealSense requires an event callback" );
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else
    {
        m_config = config;
        m_callback = callback;
        SetupStreams();
        status = SetupBuffers( buffers );
    }

    if ( QC_STATUS_OK == status )
    {
        // The descriptors hold fixed-size frame ids, check them once here.
        QRB::Memory::FrameId_t frameId;
        std::vector<const std::string *> ids = { &m_config.baseFrameId, &m_config.cameraName };
        for ( int s = 0; s < QC_REALSENSE_STREAM_COUNT; s++ )
        {
            ids.push_back( &m_frameIds[s] );
            ids.push_back( &m_opticalIds[s] );
        }
        for ( const std::string *pId : ids )
        {
            if ( QC_STATUS_OK != frameId.Assign( pId->c_str() ) )
            {
                QC_ERROR( "frame id '%s' is longer than %u characters", pId->c_str(),
                          static_cast<uint32_t>( QRB::Memory::QRB_FRAME_ID_MAX_LEN ) );
                status = QC_STATUS_BAD_ARGUMENTS;
                break;
            }
        }
    }

    if ( QC_STATUS_OK == status )
    {
        try
        {
            status = OpenDevice();
            if ( QC_STATUS_OK == status )
            {
                status = SelectProfiles();
            }
            if ( QC_STATUS_OK == status )
            {
                status = RegisterSyncRole();
            }
            if ( QC_STATUS_OK == status )
            {
                status = ApplySensorOptions();
            }
            if ( QC_STATUS_OK == status )
            {
                status = ApplyInterCamSync();
            }
            if ( QC_STATUS_OK == status )
            {
                status = CheckSensorsOpen();
            }
            if ( QC_STATUS_OK == status )
            {
                ComputeCalibration();
                ComputeStaticTransforms();
                m_bSyncFrames =
                        m_config.enableSync || m_config.alignDepth || m_config.enablePointcloud;
                if ( m_config.alignDepth )
                {
                    m_align = std::make_unique<rs2::align>( RS2_STREAM_COLOR );
                }
                if ( m_config.enablePointcloud )
                {
                    m_pointcloud = std::make_unique<rs2::pointcloud>();
                }
            }
        }
        catch ( const std::exception &e )
        {
            QC_ERROR( "failed to open the RealSense camera (serial '%s'): %s",
                      m_config.serial.c_str(), e.what() );
            status = QC_STATUS_FAIL;
        }
    }

    if ( QC_STATUS_OK == status )
    {
        m_monitor.Reset();
        m_monitor.SetConnected( true );
        m_monitor.SetInterCamSyncMode( m_deviceInfo.interCamSyncMode );
        m_state = QC_OBJECT_STATE_READY;
    }
    else if ( QC_STATUS_BAD_STATE != status )
    {
        Reset();
    }
    else
    {
        /* nothing was changed */
    }

    return status;
}

void RealSenseImpl::SetupStreams()
{
    const RealSenseConfig_t &c = m_config;
    const std::string &cam = c.cameraName;

    for ( StreamSetup &s : m_streams )
    {
        s = StreamSetup();
    }
    StreamSetup &color = m_streams[QC_REALSENSE_STREAM_COLOR];
    color.bEnabled = c.enableColor;
    color.type = RS2_STREAM_COLOR;
    color.width = c.colorWidth;
    color.height = c.colorHeight;
    color.fps = c.colorFps;
    color.format = EncodingFormat( c.colorEncoding );

    StreamSetup &depth = m_streams[QC_REALSENSE_STREAM_DEPTH];
    depth.bEnabled = c.enableDepth;
    depth.type = RS2_STREAM_DEPTH;
    depth.width = c.depthWidth;
    depth.height = c.depthHeight;
    depth.fps = c.depthFps;
    depth.format = RS2_FORMAT_Z16;

    const bool infra[2] = { c.enableInfra1, c.enableInfra2 };
    for ( int i = 0; i < 2; i++ )
    {
        StreamSetup &ir = m_streams[QC_REALSENSE_STREAM_INFRA1 + i];
        ir.bEnabled = infra[i];
        ir.type = RS2_STREAM_INFRARED;
        ir.index = i + 1;
        ir.width = c.infraWidth;
        ir.height = c.infraHeight;
        ir.fps = c.infraFps;
        ir.format = EncodingFormat( c.infraEncoding );
    }

    StreamSetup &accel = m_streams[QC_REALSENSE_STREAM_ACCEL];
    accel.bEnabled = c.enableAccel;
    accel.type = RS2_STREAM_ACCEL;
    accel.fps = c.accelFps;
    accel.format = RS2_FORMAT_MOTION_XYZ32F;

    StreamSetup &gyro = m_streams[QC_REALSENSE_STREAM_GYRO];
    gyro.bEnabled = c.enableGyro;
    gyro.type = RS2_STREAM_GYRO;
    gyro.fps = c.gyroFps;
    gyro.format = RS2_FORMAT_MOTION_XYZ32F;

    // The realsense2_camera frame names: <camera>_<stream>_frame and
    // <camera>_<stream>_optical_frame.
    for ( QCRealSenseStream_e s : kPhysicalStreams )
    {
        m_frameIds[s] = cam + "_" + RealSenseStreamName( s ) + "_frame";
        m_opticalIds[s] = cam + "_" + RealSenseStreamName( s ) + "_optical_frame";
    }
    // The aligned depth image is in the color optical frame, the pointcloud in the depth one.
    m_frameIds[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR] = cam + "_aligned_depth_to_color_frame";
    m_opticalIds[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR] =
            m_opticalIds[QC_REALSENSE_STREAM_COLOR];
    m_frameIds[QC_REALSENSE_STREAM_IMU] = cam + "_imu_frame";
    m_opticalIds[QC_REALSENSE_STREAM_IMU] = cam + "_imu_optical_frame";
    m_frameIds[QC_REALSENSE_STREAM_POINTCLOUD] = m_frameIds[QC_REALSENSE_STREAM_DEPTH];
    m_opticalIds[QC_REALSENSE_STREAM_POINTCLOUD] = m_opticalIds[QC_REALSENSE_STREAM_DEPTH];
}

QCStatus_e RealSenseImpl::SetupBuffers(
        std::vector<std::reference_wrapper<QCBufferDescriptorBase_t>> &buffers )
{
    QCStatus_e status = QC_STATUS_OK;

    for ( Pool &pool : m_pools )
    {
        pool = Pool();
    }
    for ( size_t i = 0; ( QC_STATUS_OK == status ) && ( i < buffers.size() ); i++ )
    {
        status = AddBuffer( i, buffers[i].get() );
    }

    bool enabled[QC_REALSENSE_STREAM_COUNT] = {};
    enabled[QC_REALSENSE_STREAM_COLOR] = m_config.enableColor;
    enabled[QC_REALSENSE_STREAM_DEPTH] = m_config.enableDepth;
    enabled[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR] = m_config.alignDepth;
    enabled[QC_REALSENSE_STREAM_INFRA1] = m_config.enableInfra1;
    enabled[QC_REALSENSE_STREAM_INFRA2] = m_config.enableInfra2;
    enabled[QC_REALSENSE_STREAM_POINTCLOUD] = m_config.enablePointcloud;
    for ( int s = 0; ( QC_STATUS_OK == status ) && ( s < QC_REALSENSE_STREAM_COUNT ); s++ )
    {
        const QCRealSenseStream_e stream = static_cast<QCRealSenseStream_e>( s );
        const size_t count = m_pools[s].Size();
        if ( enabled[s] && ( ( 0 == count ) || ( count > QC_REALSENSE_MAX_BUFFERS ) ) )
        {
            QC_ERROR( "the %s stream requires 1 to %u buffers, got %u",
                      RealSenseStreamName( stream ), QC_REALSENSE_MAX_BUFFERS,
                      static_cast<uint32_t>( count ) );
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        else if ( ( false == enabled[s] ) && ( 0 != count ) )
        {
            QC_ERROR( "got %u buffers for the disabled %s stream", static_cast<uint32_t>( count ),
                      RealSenseStreamName( stream ) );
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        else
        {
            m_pools[s].inUse.assign( count, false );
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::AddBuffer( size_t index, const QCBufferDescriptorBase_t &bufDesc )
{
    QCStatus_e status = QC_STATUS_OK;
    const uint32_t i = static_cast<uint32_t>( index );
    const auto *pImage = dynamic_cast<const RealSenseImageDescriptor_t *>( &bufDesc );
    const auto *pCloud = dynamic_cast<const PointCloudDescriptor_t *>( &bufDesc );

    if ( ( nullptr == pImage ) && ( nullptr == pCloud ) )
    {
        QC_ERROR( "buffer %u is neither a RealSenseImageDescriptor_t nor a PointCloudDescriptor_t",
                  i );
        status = QC_STATUS_INVALID_BUF;
    }
    else if ( nullptr == bufDesc.pBuf )
    {
        QC_ERROR( "buffer %u has no memory", i );
        status = QC_STATUS_INVALID_BUF;
    }
    else if ( nullptr != pCloud )
    {
        // The largest cloud: an ordered one, one point per depth pixel.
        const uint64_t need = static_cast<uint64_t>( m_config.depthWidth ) * m_config.depthHeight *
                              RealSensePointStep( m_config.pointcloudTexture );
        if ( ( pCloud->offset > pCloud->size ) || ( ( pCloud->size - pCloud->offset ) < need ) )
        {
            QC_ERROR( "pointcloud buffer %u holds %u bytes, %llu needed", i,
                      static_cast<uint32_t>( pCloud->size ),
                      static_cast<unsigned long long>( need ) );
            status = QC_STATUS_INVALID_BUF;
        }
        else
        {
            m_pools[QC_REALSENSE_STREAM_POINTCLOUD].clouds.push_back( *pCloud );
        }
    }
    else
    {
        const QCRealSenseStream_e stream = pImage->stream;
        std::string encoding;
        uint32_t width = 0;
        uint32_t height = 0;
        switch ( stream )
        {
            case QC_REALSENSE_STREAM_COLOR:
                encoding = m_config.colorEncoding;
                width = m_config.colorWidth;
                height = m_config.colorHeight;
                break;
            case QC_REALSENSE_STREAM_DEPTH:
                encoding = QC_REALSENSE_ENCODING_16UC1;
                width = m_config.depthWidth;
                height = m_config.depthHeight;
                break;
            case QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR:
                // An aligned depth image has the color resolution.
                encoding = QC_REALSENSE_ENCODING_16UC1;
                width = m_config.colorWidth;
                height = m_config.colorHeight;
                break;
            case QC_REALSENSE_STREAM_INFRA1:
            case QC_REALSENSE_STREAM_INFRA2:
                encoding = m_config.infraEncoding;
                width = m_config.infraWidth;
                height = m_config.infraHeight;
                break;
            default:
                QC_ERROR( "buffer %u has stream %d, expected an image stream (color, depth, "
                          "aligned_depth_to_color, infra1, infra2)",
                          i, static_cast<int>( stream ) );
                status = QC_STATUS_INVALID_BUF;
                break;
        }

        if ( QC_STATUS_OK == status )
        {
            const uint64_t rowBytes = static_cast<uint64_t>( width ) *
                                      RealSenseEncodingBytesPerPixel( encoding.c_str() );
            if ( pImage->encoding != encoding.c_str() )
            {
                QC_ERROR( "%s buffer %u has encoding '%s', the stream is %s",
                          RealSenseStreamName( stream ), i, pImage->encoding.CStr(),
                          encoding.c_str() );
                status = QC_STATUS_INVALID_BUF;
            }
            else if ( ( width != pImage->width ) || ( height != pImage->height ) )
            {
                QC_ERROR( "%s buffer %u is %ux%u, the stream is %ux%u",
                          RealSenseStreamName( stream ), i, pImage->width, pImage->height, width,
                          height );
                status = QC_STATUS_INVALID_BUF;
            }
            else if ( ( pImage->step < rowBytes ) || ( pImage->offset > pImage->size ) ||
                      ( pImage->GetImageSize() > ( pImage->size - pImage->offset ) ) )
            {
                QC_ERROR( "%s buffer %u is too small: step=%u size=%u offset=%u",
                          RealSenseStreamName( stream ), i, pImage->step,
                          static_cast<uint32_t>( pImage->size ),
                          static_cast<uint32_t>( pImage->offset ) );
                status = QC_STATUS_INVALID_BUF;
            }
            else
            {
                m_pools[stream].images.push_back( *pImage );
            }
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::OpenDevice()
{
    QCStatus_e status = QC_STATUS_OK;

    m_ctx = std::make_unique<rs2::context>();
    rs2::device_list devices = m_ctx->query_devices();
    std::string serials;

    for ( uint32_t i = 0; i < devices.size(); i++ )
    {
        rs2::device dev = devices[i];
        const std::string serial = DeviceInfoString( dev, RS2_CAMERA_INFO_SERIAL_NUMBER );
        serials += ( serials.empty() ? "" : ", " ) + serial;
        if ( ( nullptr == m_device ) &&
             ( m_config.serial.empty() || ( serial == m_config.serial ) ) )
        {
            m_device = std::make_unique<rs2::device>( dev );
        }
    }

    if ( nullptr == m_device )
    {
        QC_ERROR( "no RealSense camera with serial '%s' (connected: %s)", m_config.serial.c_str(),
                  serials.empty() ? "none" : serials.c_str() );
        status = QC_STATUS_FAIL;
    }
    else
    {
        RealSenseDeviceInfo_t &info = m_deviceInfo;
        info = RealSenseDeviceInfo_t();
        info.name = DeviceInfoString( *m_device, RS2_CAMERA_INFO_NAME );
        info.serial = DeviceInfoString( *m_device, RS2_CAMERA_INFO_SERIAL_NUMBER );
        info.firmware = DeviceInfoString( *m_device, RS2_CAMERA_INFO_FIRMWARE_VERSION );
        info.productId = DeviceInfoString( *m_device, RS2_CAMERA_INFO_PRODUCT_ID );
        info.productLine = DeviceInfoString( *m_device, RS2_CAMERA_INFO_PRODUCT_LINE );
        info.usbType = DeviceInfoString( *m_device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR );
        info.physicalPort = DeviceInfoString( *m_device, RS2_CAMERA_INFO_PHYSICAL_PORT );
        info.cameraName = m_config.cameraName;
        info.syncGroup = m_config.syncGroup;
        info.bConnected = true;

        m_sensors = m_device->query_sensors();
        m_modules.clear();
        for ( const rs2::sensor &sensor : m_sensors )
        {
            m_modules.push_back( ModuleName( sensor ) );
            if ( sensor.is<rs2::depth_sensor>() )
            {
                m_depthScale = sensor.as<rs2::depth_sensor>().get_depth_scale();
            }
        }
        info.sensors = m_modules;
        info.depthScale = m_depthScale;
        m_sensorProfiles.assign( m_sensors.size(), std::vector<rs2::stream_profile>() );
        m_sensorOpen.assign( m_sensors.size(), false );
        m_sensorStarted.assign( m_sensors.size(), false );

        QC_INFO( "camera '%s': %s serial %s firmware %s usb %s port %s",
                 m_config.cameraName.c_str(), info.name.c_str(), info.serial.c_str(),
                 info.firmware.c_str(), info.usbType.c_str(), info.physicalPort.c_str() );
        QC_INFO( "depth unit %f m", m_depthScale );
        if ( ( false == info.usbType.empty() ) && ( '2' == info.usbType[0] ) )
        {
            QC_WARN( "camera '%s' is connected over USB %s: use a USB 3 port",
                     m_config.cameraName.c_str(), info.usbType.c_str() );
        }

        // Track the camera connection for the diagnostics.
        m_watch = std::make_shared<DeviceWatch>();
        m_watch->pImpl = this;
        std::shared_ptr<DeviceWatch> watch = m_watch;
        m_ctx->set_devices_changed_callback( [watch]( rs2::event_information &event ) {
            std::lock_guard<std::mutex> lock( watch->lock );
            if ( nullptr != watch->pImpl )
            {
                watch->pImpl->OnDevicesChanged( event );
            }
        } );
    }

    return status;
}

QCStatus_e RealSenseImpl::FindProfile( QCRealSenseStream_e stream, StreamSetup &setup,
                                       bool bReference )
{
    QCStatus_e status = QC_STATUS_FAIL;
    std::set<std::string> available;
    const bool bMotion = ( RS2_STREAM_ACCEL == setup.type ) || ( RS2_STREAM_GYRO == setup.type );

    for ( size_t i = 0; ( i < m_sensors.size() ) && ( QC_STATUS_OK != status ); i++ )
    {
        const std::vector<rs2::stream_profile> profiles = m_sensors[i].get_stream_profiles();
        const rs2::stream_profile *pFallback = nullptr;
        for ( const rs2::stream_profile &p : profiles )
        {
            if ( ( p.stream_type() != setup.type ) ||
                 ( ( setup.index >= 0 ) && ( p.stream_index() != setup.index ) ) )
            {
                continue;
            }
            if ( bReference )
            {
                // Any profile of the stream, for its extrinsics: the default one if any.
                if ( p.is_default() || ( nullptr == pFallback ) )
                {
                    pFallback = &p;
                }
                continue;
            }

            char text[96];
            if ( bMotion )
            {
                (void) snprintf( text, sizeof( text ), "%d Hz", p.fps() );
                available.insert( text );
                if ( ( p.format() == setup.format ) &&
                     ( ( ( 0 == setup.fps ) && p.is_default() ) ||
                       ( static_cast<uint32_t>( p.fps() ) == setup.fps ) ) )
                {
                    setup.profile = p;
                    setup.sensor = static_cast<int>( i );
                    status = QC_STATUS_OK;
                    break;
                }
                if ( ( 0 == setup.fps ) && ( p.format() == setup.format ) &&
                     ( nullptr == pFallback ) )
                {
                    pFallback = &p;
                }
            }
            else if ( auto vp = p.as<rs2::video_stream_profile>() )
            {
                (void) snprintf( text, sizeof( text ), "%dx%d@%d %s", vp.width(), vp.height(),
                                 vp.fps(), rs2_format_to_string( vp.format() ) );
                available.insert( text );
                if ( ( vp.format() == setup.format ) &&
                     ( static_cast<uint32_t>( vp.width() ) == setup.width ) &&
                     ( static_cast<uint32_t>( vp.height() ) == setup.height ) &&
                     ( static_cast<uint32_t>( vp.fps() ) == setup.fps ) )
                {
                    setup.profile = p;
                    setup.sensor = static_cast<int>( i );
                    status = QC_STATUS_OK;
                    break;
                }
            }
            else
            {
                /* not a video profile */
            }
        }
        if ( ( QC_STATUS_OK != status ) && ( nullptr != pFallback ) )
        {
            // A reference profile, or a motion stream at the camera rate without a default.
            setup.profile = *pFallback;
            setup.sensor = static_cast<int>( i );
            status = QC_STATUS_OK;
        }
    }

    if ( QC_STATUS_OK == status )
    {
        setup.bHaveProfile = true;
    }
    else if ( false == bReference )
    {
        std::string list;
        size_t n = 0;
        for ( const std::string &text : available )
        {
            if ( ++n > 24 )
            {
                list += ", ...";
                break;
            }
            list += ( list.empty() ? "" : ", " ) + text;
        }
        if ( bMotion )
        {
            QC_ERROR( "camera '%s' (%s): no %s profile at %u Hz (0 = default); supported: %s",
                      m_config.cameraName.c_str(), m_deviceInfo.name.c_str(),
                      RealSenseStreamName( stream ), setup.fps,
                      available.empty() ? "none, the camera has no IMU" : list.c_str() );
        }
        else
        {
            QC_ERROR( "camera '%s' (%s): no %s profile %ux%u@%u %s; supported: %s",
                      m_config.cameraName.c_str(), m_deviceInfo.name.c_str(),
                      RealSenseStreamName( stream ), setup.width, setup.height, setup.fps,
                      rs2_format_to_string( setup.format ),
                      available.empty() ? "none" : list.c_str() );
        }
    }
    else
    {
        /* the camera has no such stream: no extrinsics for it */
    }

    return status;
}

QCStatus_e RealSenseImpl::SelectProfiles()
{
    QCStatus_e status = QC_STATUS_OK;

    for ( QCRealSenseStream_e s : kPhysicalStreams )
    {
        StreamSetup &setup = m_streams[s];
        const QCStatus_e ret = FindProfile( s, setup, false == setup.bEnabled );
        if ( setup.bEnabled )
        {
            if ( QC_STATUS_OK != ret )
            {
                status = QC_STATUS_FAIL;
            }
            else
            {
                m_sensorProfiles[setup.sensor].push_back( setup.profile );
            }
        }
    }

    return status;
}

int RealSenseImpl::FindSensor( const std::string &module ) const
{
    int index = -1;
    for ( size_t i = 0; i < m_modules.size(); i++ )
    {
        if ( m_modules[i] == module )
        {
            index = static_cast<int>( i );
            break;
        }
    }
    return index;
}

QCStatus_e RealSenseImpl::ApplySensorOptions()
{
    QCStatus_e status = QC_STATUS_OK;

    for ( const RealSenseOption_t &opt : m_config.sensorOptions )
    {
        const int index = FindSensor( opt.module );
        if ( index < 0 )
        {
            std::string modules;
            for ( const std::string &m : m_modules )
            {
                modules += ( modules.empty() ? "" : ", " ) + m;
            }
            QC_ERROR( "sensor option %s.%s: camera '%s' has no module %s (modules: %s)",
                      opt.module.c_str(), opt.option.c_str(), m_config.cameraName.c_str(),
                      opt.module.c_str(), modules.c_str() );
            status = QC_STATUS_BAD_ARGUMENTS;
            break;
        }

        rs2::sensor &sensor = m_sensors[index];
        bool bFound = false;
        rs2_option option = RS2_OPTION_COUNT;
        std::string supported;
        for ( rs2_option o : sensor.get_supported_options() )
        {
            const std::string name = OptionName( o );
            supported += ( supported.empty() ? "" : ", " ) + name;
            if ( name == opt.option )
            {
                option = o;
                bFound = true;
            }
        }
        if ( false == bFound )
        {
            QC_ERROR( "sensor option %s.%s is not supported by camera '%s' (%s); supported: %s",
                      opt.module.c_str(), opt.option.c_str(), m_config.cameraName.c_str(),
                      m_deviceInfo.name.c_str(), supported.c_str() );
            status = QC_STATUS_BAD_ARGUMENTS;
            break;
        }
        if ( sensor.is_option_read_only( option ) )
        {
            QC_ERROR( "sensor option %s.%s is read only", opt.module.c_str(), opt.option.c_str() );
            status = QC_STATUS_BAD_ARGUMENTS;
            break;
        }
        const rs2::option_range range = sensor.get_option_range( option );
        if ( ( opt.value < range.min ) || ( opt.value > range.max ) )
        {
            QC_ERROR( "sensor option %s.%s = %g is out of range [%g, %g]", opt.module.c_str(),
                      opt.option.c_str(), static_cast<double>( opt.value ),
                      static_cast<double>( range.min ), static_cast<double>( range.max ) );
            status = QC_STATUS_BAD_ARGUMENTS;
            break;
        }
        try
        {
            sensor.set_option( option, opt.value );
            QC_INFO( "camera '%s': %s.%s = %g", m_config.cameraName.c_str(), opt.module.c_str(),
                     opt.option.c_str(), static_cast<double>( sensor.get_option( option ) ) );
        }
        catch ( const std::exception &e )
        {
            QC_ERROR( "failed to set sensor option %s.%s = %g: %s", opt.module.c_str(),
                      opt.option.c_str(), static_cast<double>( opt.value ), e.what() );
            status = QC_STATUS_FAIL;
            break;
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::ApplyInterCamSync()
{
    QCStatus_e status = QC_STATUS_OK;
    const int index = FindSensor( "depth_module" );
    const bool bSupported =
            ( index >= 0 ) && m_sensors[index].supports( RS2_OPTION_INTER_CAM_SYNC_MODE );

    if ( m_config.interCamSyncMode >= 0 )
    {
        if ( false == bSupported )
        {
            QC_ERROR( "camera '%s' (%s) does not support hardware sync "
                      "(depth_module.inter_cam_sync_mode)",
                      m_config.cameraName.c_str(), m_deviceInfo.name.c_str() );
            status = QC_STATUS_UNSUPPORTED;
        }
        else
        {
            rs2::sensor &sensor = m_sensors[index];
            const rs2::option_range range =
                    sensor.get_option_range( RS2_OPTION_INTER_CAM_SYNC_MODE );
            const float mode = static_cast<float>( m_config.interCamSyncMode );
            if ( ( mode < range.min ) || ( mode > range.max ) )
            {
                QC_ERROR( "interCamSyncMode %d is out of the camera range [%g, %g]",
                          m_config.interCamSyncMode, static_cast<double>( range.min ),
                          static_cast<double>( range.max ) );
                status = QC_STATUS_BAD_ARGUMENTS;
            }
            else
            {
                sensor.set_option( RS2_OPTION_INTER_CAM_SYNC_MODE, mode );
            }
        }
    }

    if ( ( QC_STATUS_OK == status ) && bSupported )
    {
        m_deviceInfo.interCamSyncMode = static_cast<int>(
                std::lround( m_sensors[index].get_option( RS2_OPTION_INTER_CAM_SYNC_MODE ) ) );
        const int mode = m_deviceInfo.interCamSyncMode;
        QC_INFO( "camera '%s': hardware sync group '%s', role %s (inter_cam_sync_mode %d)",
                 m_config.cameraName.c_str(), m_config.syncGroup.c_str(),
                 ( 0 == mode ) ? "default" : ( ( 1 == mode ) ? "master" : "slave" ), mode );
        if ( ( m_config.interCamSyncMode >= 0 ) && ( mode != m_config.interCamSyncMode ) )
        {
            QC_ERROR( "camera '%s' kept inter_cam_sync_mode %d instead of %d",
                      m_config.cameraName.c_str(), mode, m_config.interCamSyncMode );
            status = QC_STATUS_FAIL;
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::RegisterSyncRole()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( m_config.interCamSyncMode >= 0 )
    {
        std::lock_guard<std::mutex> lock( g_syncLock );
        std::vector<SyncMember> &group = g_syncGroups[m_config.syncGroup];
        for ( const SyncMember &member : group )
        {
            if ( ( 1 == m_config.interCamSyncMode ) && ( 1 == member.mode ) )
            {
                QC_ERROR( "sync group '%s' already has the master '%s' (serial %s): '%s' "
                          "(serial %s) cannot be a second master",
                          m_config.syncGroup.c_str(), member.cameraName.c_str(),
                          member.serial.c_str(), m_config.cameraName.c_str(),
                          m_deviceInfo.serial.c_str() );
                status = QC_STATUS_BAD_ARGUMENTS;
            }
            else if ( member.serial == m_deviceInfo.serial )
            {
                QC_ERROR( "camera serial %s is already in sync group '%s' as '%s'",
                          member.serial.c_str(), m_config.syncGroup.c_str(),
                          member.cameraName.c_str() );
                status = QC_STATUS_BAD_ARGUMENTS;
            }
            else
            {
                /* another slave */
            }
        }
        if ( QC_STATUS_OK == status )
        {
            group.push_back(
                    { this, m_deviceInfo.serial, m_config.cameraName, m_config.interCamSyncMode } );
            m_bSyncRegistered = true;
        }
    }

    return status;
}

void RealSenseImpl::UnregisterSyncRole()
{
    if ( m_bSyncRegistered )
    {
        std::lock_guard<std::mutex> lock( g_syncLock );
        auto it = g_syncGroups.find( m_config.syncGroup );
        if ( g_syncGroups.end() != it )
        {
            std::vector<SyncMember> &group = it->second;
            group.erase(
                    std::remove_if( group.begin(), group.end(),
                                    [this]( const SyncMember &m ) { return m.pOwner == this; } ),
                    group.end() );
            if ( group.empty() )
            {
                g_syncGroups.erase( it );
            }
        }
        m_bSyncRegistered = false;
    }
}

QCStatus_e RealSenseImpl::CheckSensorsOpen()
{
    QCStatus_e status = QC_STATUS_OK;

    // Open and close each sensor once, so that a profile combination the sensor does not support
    // together, or a camera used by another process, is reported here and not at Start().
    for ( size_t i = 0; ( QC_STATUS_OK == status ) && ( i < m_sensors.size() ); i++ )
    {
        if ( false == m_sensorProfiles[i].empty() )
        {
            try
            {
                m_sensors[i].open( m_sensorProfiles[i] );
                m_sensors[i].close();
            }
            catch ( const std::exception &e )
            {
                QC_ERROR( "camera '%s': cannot open %s with the configured streams: %s",
                          m_config.cameraName.c_str(), m_modules[i].c_str(), e.what() );
                status = QC_STATUS_FAIL;
            }
        }
    }

    return status;
}

const rs2::stream_profile *RealSenseImpl::ProfileOf( QCRealSenseStream_e stream ) const
{
    QCRealSenseStream_e physical = stream;
    if ( QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR == stream )
    {
        physical = QC_REALSENSE_STREAM_COLOR;
    }
    else if ( QC_REALSENSE_STREAM_IMU == stream )
    {
        physical = QC_REALSENSE_STREAM_GYRO;
    }
    else if ( QC_REALSENSE_STREAM_POINTCLOUD == stream )
    {
        physical = QC_REALSENSE_STREAM_DEPTH;
    }
    else
    {
        /* a physical stream */
    }
    return ( ValidStream( physical ) && m_streams[physical].bHaveProfile )
                   ? &m_streams[physical].profile
                   : nullptr;
}

void RealSenseImpl::ComputeCalibration()
{
    for ( RealSenseCameraInfo_t &info : m_cameraInfo )
    {
        info = RealSenseCameraInfo_t();
    }

    // realsense2_camera updateStreamCalibData.
    const QCRealSenseStream_e videoStreams[] = {
            QC_REALSENSE_STREAM_COLOR, QC_REALSENSE_STREAM_DEPTH, QC_REALSENSE_STREAM_INFRA1,
            QC_REALSENSE_STREAM_INFRA2 };
    for ( QCRealSenseStream_e s : videoStreams )
    {
        const StreamSetup &setup = m_streams[s];
        if ( ( false == setup.bEnabled ) || ( false == setup.bHaveProfile ) )
        {
            continue;
        }
        RealSenseCameraInfo_t &info = m_cameraInfo[s];
        rs2_intrinsics in = {};
        try
        {
            in = setup.profile.as<rs2::video_stream_profile>().get_intrinsics();
        }
        catch ( const std::exception &e )
        {
            QC_WARN( "no intrinsics for the %s stream, zeroed: %s", RealSenseStreamName( s ),
                     e.what() );
            in = rs2_intrinsics{ static_cast<int>( setup.width ),
                                 static_cast<int>( setup.height ),
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 0.0f,
                                 RS2_DISTORTION_NONE,
                                 { 0, 0, 0, 0, 0 } };
        }
        info.bValid = true;
        info.width = static_cast<uint32_t>( in.width );
        info.height = static_cast<uint32_t>( in.height );
        (void) info.frameId.Assign( m_opticalIds[s].c_str() );
        info.K[0] = in.fx;
        info.K[2] = in.ppx;
        info.K[4] = in.fy;
        info.K[5] = in.ppy;
        info.K[8] = 1.0;
        info.P[0] = in.fx;
        info.P[2] = in.ppx;
        info.P[5] = in.fy;
        info.P[6] = in.ppy;
        info.P[10] = 1.0;
        info.R[0] = 1.0;
        info.R[4] = 1.0;
        info.R[8] = 1.0;
        if ( RS2_DISTORTION_KANNALA_BRANDT4 == in.model )
        {
            (void) info.distortionModel.Assign( "equidistant" );
            info.numD = 4;
        }
        else
        {
            (void) info.distortionModel.Assign( "plumb_bob" );
            info.numD = 5;
        }
        for ( uint32_t i = 0; i < info.numD; i++ )
        {
            info.D[i] = in.coeffs[i];
        }
    }

    // realsense2_camera updateExtrinsicsCalibData: the right IR in the left IR optical frame,
    // with the stereo baseline in P.
    if ( m_streams[QC_REALSENSE_STREAM_INFRA1].bEnabled &&
         m_streams[QC_REALSENSE_STREAM_INFRA2].bEnabled )
    {
        try
        {
            const rs2_extrinsics ex =
                    m_streams[QC_REALSENSE_STREAM_INFRA2].profile.get_extrinsics_to(
                            m_streams[QC_REALSENSE_STREAM_INFRA1].profile );
            RealSenseCameraInfo_t &right = m_cameraInfo[QC_REALSENSE_STREAM_INFRA2];
            right.P[3] = -right.K[0] * ex.translation[0] + 0.0;
            right.P[7] = -right.K[4] * ex.translation[1] + 0.0;
            (void) right.frameId.Assign( m_opticalIds[QC_REALSENSE_STREAM_INFRA1].c_str() );
        }
        catch ( const std::exception &e )
        {
            QC_WARN( "no infra2 to infra1 extrinsics: %s", e.what() );
        }
    }

    // The aligned depth has the color calibration, the pointcloud the depth one.
    m_cameraInfo[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR] =
            m_cameraInfo[QC_REALSENSE_STREAM_COLOR];
    if ( false == m_config.alignDepth )
    {
        m_cameraInfo[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR].bValid = false;
    }
    m_cameraInfo[QC_REALSENSE_STREAM_POINTCLOUD] = m_cameraInfo[QC_REALSENSE_STREAM_DEPTH];
    if ( false == m_config.enablePointcloud )
    {
        m_cameraInfo[QC_REALSENSE_STREAM_POINTCLOUD].bValid = false;
    }
}

void RealSenseImpl::ComputeStaticTransforms()
{
    // realsense2_camera calcAndAppendTransformMsgs, with the depth stream as the base stream.
    const Quaternion_t optical = QuatFromRpy( -M_PI / 2.0, 0.0, -M_PI / 2.0 );
    const Quaternion_t identity;
    const float zero[3] = { 0.0f, 0.0f, 0.0f };
    const uint64_t stampNs = static_cast<uint64_t>( ToClockNs( ClockNs( CLOCK_REALTIME ) ) );
    const rs2::stream_profile *pBase = ProfileOf( QC_REALSENSE_STREAM_DEPTH );
    const bool bUnited = ( QC_REALSENSE_UNITE_IMU_NONE != m_config.uniteImuMethod ) &&
                         m_config.enableAccel && m_config.enableGyro;

    m_staticTfs.clear();
    for ( QCRealSenseStream_e s : kPhysicalStreams )
    {
        const StreamSetup &setup = m_streams[s];
        if ( ( false == setup.bEnabled ) || ( false == setup.bHaveProfile ) )
        {
            continue;
        }
        rs2_extrinsics ex = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 0, 0, 0 } };
        if ( nullptr != pBase )
        {
            try
            {
                ex = setup.profile.get_extrinsics_to( *pBase );
            }
            catch ( const std::exception &e )
            {
                QC_WARN( "%s -> depth: %s, using identity", RealSenseStreamName( s ), e.what() );
            }
        }
        const Quaternion_t q = QuatMul( QuatMul( optical, QuatFromRotation( ex.rotation ) ),
                                        QuatInverse( optical ) );
        m_staticTfs.push_back(
                MakeTransform( stampNs, ex.translation, q, m_config.baseFrameId, m_frameIds[s] ) );
        m_staticTfs.push_back(
                MakeTransform( stampNs, zero, optical, m_frameIds[s], m_opticalIds[s] ) );

        if ( ( QC_REALSENSE_STREAM_COLOR == s ) && m_config.alignDepth )
        {
            const std::string &aligned = m_frameIds[QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR];
            m_staticTfs.push_back(
                    MakeTransform( stampNs, ex.translation, q, m_config.baseFrameId, aligned ) );
            m_staticTfs.push_back( MakeTransform( stampNs, zero, optical, aligned,
                                                  m_opticalIds[QC_REALSENSE_STREAM_COLOR] ) );
        }
        if ( ( QC_REALSENSE_STREAM_GYRO == s ) && bUnited )
        {
            m_staticTfs.push_back( MakeTransform( stampNs, zero, identity, m_frameIds[s],
                                                  m_frameIds[QC_REALSENSE_STREAM_IMU] ) );
            m_staticTfs.push_back( MakeTransform( stampNs, zero, optical,
                                                  m_frameIds[QC_REALSENSE_STREAM_IMU],
                                                  m_opticalIds[QC_REALSENSE_STREAM_IMU] ) );
        }
    }
}

/* ======================================== Queries =========================================== */

QCStatus_e RealSenseImpl::GetDeviceInfo( RealSenseDeviceInfo_t &info )
{
    QCStatus_e status = QC_STATUS_OK;
    const QCObjectState_e state = m_state;

    if ( ( QC_OBJECT_STATE_READY != state ) && ( QC_OBJECT_STATE_RUNNING != state ) )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        std::lock_guard<std::mutex> lock( m_infoLock );
        info = m_deviceInfo;
    }

    return status;
}

QCStatus_e RealSenseImpl::GetCameraInfo( QCRealSenseStream_e stream, RealSenseCameraInfo_t &info )
{
    QCStatus_e status = QC_STATUS_OK;
    const QCObjectState_e state = m_state;

    if ( ( QC_OBJECT_STATE_READY != state ) && ( QC_OBJECT_STATE_RUNNING != state ) )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else if ( ( false == ValidStream( stream ) ) || ( false == m_cameraInfo[stream].bValid ) )
    {
        QC_ERROR( "no CameraInfo for stream %d (%s): enabled image or pointcloud streams only",
                  static_cast<int>( stream ), RealSenseStreamName( stream ) );
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else
    {
        info = m_cameraInfo[stream];
    }

    return status;
}

QCStatus_e RealSenseImpl::GetExtrinsics( QCRealSenseStream_e from, QCRealSenseStream_e to,
                                         RealSenseExtrinsics_t &extrinsics )
{
    QCStatus_e status = QC_STATUS_OK;
    const QCObjectState_e state = m_state;
    const rs2::stream_profile *pFrom = ProfileOf( from );
    const rs2::stream_profile *pTo = ProfileOf( to );

    if ( ( QC_OBJECT_STATE_READY != state ) && ( QC_OBJECT_STATE_RUNNING != state ) )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else if ( ( nullptr == pFrom ) || ( nullptr == pTo ) )
    {
        QC_ERROR( "no extrinsics from %s to %s: the camera lacks the stream",
                  RealSenseStreamName( from ), RealSenseStreamName( to ) );
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else
    {
        try
        {
            const rs2_extrinsics ex = pFrom->get_extrinsics_to( *pTo );
            std::copy( std::begin( ex.rotation ), std::end( ex.rotation ),
                       std::begin( extrinsics.rotation ) );
            std::copy( std::begin( ex.translation ), std::end( ex.translation ),
                       std::begin( extrinsics.translation ) );
        }
        catch ( const std::exception &e )
        {
            QC_ERROR( "no extrinsics from %s to %s: %s", RealSenseStreamName( from ),
                      RealSenseStreamName( to ), e.what() );
            status = QC_STATUS_FAIL;
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::GetImuInfo( QCRealSenseStream_e stream, RealSenseImuInfo_t &info )
{
    QCStatus_e status = QC_STATUS_OK;
    const QCObjectState_e state = m_state;

    if ( ( QC_OBJECT_STATE_READY != state ) && ( QC_OBJECT_STATE_RUNNING != state ) )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else if ( ( ( QC_REALSENSE_STREAM_ACCEL != stream ) &&
                ( QC_REALSENSE_STREAM_GYRO != stream ) ) ||
              ( false == m_streams[stream].bHaveProfile ) )
    {
        QC_ERROR( "no IMU info for stream %s: accel or gyro of a camera with an IMU",
                  RealSenseStreamName( stream ) );
        status = QC_STATUS_BAD_ARGUMENTS;
    }
    else
    {
        info = RealSenseImuInfo_t();
        (void) info.frameId.Assign( m_opticalIds[stream].c_str() );
        try
        {
            const rs2_motion_device_intrinsic in = m_streams[stream]
                                                           .profile.as<rs2::motion_stream_profile>()
                                                           .get_motion_intrinsics();
            for ( int r = 0; r < 3; r++ )
            {
                for ( int c = 0; c < 4; c++ )
                {
                    info.data[r * 4 + c] = in.data[r][c];
                }
                info.noiseVariances[r] = in.noise_variances[r];
                info.biasVariances[r] = in.bias_variances[r];
            }
            info.bValid = true;
        }
        catch ( const std::exception &e )
        {
            // As realsense2_camera, which then publishes a zero IMUInfo.
            QC_WARN( "no %s intrinsics: %s", RealSenseStreamName( stream ), e.what() );
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::GetStaticTransforms( std::vector<TransformStamped_t> &transforms )
{
    QCStatus_e status = QC_STATUS_OK;
    const QCObjectState_e state = m_state;

    if ( ( QC_OBJECT_STATE_READY != state ) && ( QC_OBJECT_STATE_RUNNING != state ) )
    {
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        transforms = m_staticTfs;
    }

    return status;
}

QCStatus_e RealSenseImpl::EnumerateDevices( std::vector<RealSenseDeviceInfo_t> &devices )
{
    QCStatus_e status = QC_STATUS_OK;

    devices.clear();
    try
    {
        rs2::context ctx;
        rs2::device_list list = ctx.query_devices();
        for ( uint32_t i = 0; i < list.size(); i++ )
        {
            rs2::device dev = list[i];
            RealSenseDeviceInfo_t info;
            info.name = DeviceInfoString( dev, RS2_CAMERA_INFO_NAME );
            info.serial = DeviceInfoString( dev, RS2_CAMERA_INFO_SERIAL_NUMBER );
            info.firmware = DeviceInfoString( dev, RS2_CAMERA_INFO_FIRMWARE_VERSION );
            info.productId = DeviceInfoString( dev, RS2_CAMERA_INFO_PRODUCT_ID );
            info.productLine = DeviceInfoString( dev, RS2_CAMERA_INFO_PRODUCT_LINE );
            info.usbType = DeviceInfoString( dev, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR );
            info.physicalPort = DeviceInfoString( dev, RS2_CAMERA_INFO_PHYSICAL_PORT );
            info.bConnected = true;
            for ( const rs2::sensor &sensor : dev.query_sensors() )
            {
                info.sensors.push_back( ModuleName( sensor ) );
                if ( sensor.is<rs2::depth_sensor>() )
                {
                    info.depthScale = sensor.as<rs2::depth_sensor>().get_depth_scale();
                    if ( sensor.supports( RS2_OPTION_INTER_CAM_SYNC_MODE ) )
                    {
                        info.interCamSyncMode = static_cast<int>( std::lround(
                                sensor.get_option( RS2_OPTION_INTER_CAM_SYNC_MODE ) ) );
                    }
                }
            }
            devices.push_back( info );
        }
    }
    catch ( const std::exception &e )
    {
        (void) e;
        status = QC_STATUS_FAIL;
    }

    return status;
}

/* ======================================== Streaming ========================================= */

QCStatus_e RealSenseImpl::Start()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( QC_OBJECT_STATE_READY != m_state )
    {
        QC_ERROR( "RealSense start failed due to wrong state: %d", m_state.load() );
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        {
            std::lock_guard<std::mutex> lock( m_imuLock );
            m_bHaveAccel = false;
            m_pendingGyros.clear();
        }
        {
            std::lock_guard<std::mutex> lock( m_timeBaseLock );
            m_bTimeBase = false;
            m_previousCameraMs.clear();
        }
        m_monitor.ResetContinuity();
        m_lastSampleNs = ClockNs( CLOCK_MONOTONIC );

        if ( m_bSyncRegistered && ( m_config.interCamSyncMode >= 2 ) )
        {
            std::lock_guard<std::mutex> lock( g_syncLock );
            const std::vector<SyncMember> &group = g_syncGroups[m_config.syncGroup];
            if ( group.end() == std::find_if( group.begin(), group.end(),
                                              []( const SyncMember &m ) { return 1 == m.mode; } ) )
            {
                QC_WARN( "sync group '%s' has no master in this process: the master camera must "
                         "be started by another process",
                         m_config.syncGroup.c_str() );
            }
        }

        // Running before the first sample can arrive, so that it is delivered as running.
        m_state = QC_OBJECT_STATE_RUNNING;
        size_t i = 0;
        try
        {
            if ( m_bSyncFrames )
            {
                m_asyncer = std::make_unique<rs2::asynchronous_syncer>();
                m_asyncer->start( [this]( rs2::frame frame ) { OnSynced( frame ); } );
            }
            for ( i = 0; i < m_sensors.size(); i++ )
            {
                if ( false == m_sensorProfiles[i].empty() )
                {
                    m_sensors[i].open( m_sensorProfiles[i] );
                    m_sensorOpen[i] = true;
                }
            }
            // All the sensors start together, once all are open.
            for ( i = 0; i < m_sensors.size(); i++ )
            {
                if ( m_sensorOpen[i] )
                {
                    const bool bMotion = ( "motion_module" == m_modules[i] ) ||
                                         m_sensors[i].is<rs2::motion_sensor>();
                    if ( bMotion )
                    {
                        m_sensors[i].start( [this]( rs2::frame frame ) { OnMotion( frame ); } );
                    }
                    else
                    {
                        m_sensors[i].start( [this]( rs2::frame frame ) { OnVideo( frame ); } );
                    }
                    m_sensorStarted[i] = true;
                }
            }
        }
        catch ( const std::exception &e )
        {
            QC_ERROR( "camera '%s': failed to start %s: %s", m_config.cameraName.c_str(),
                      ( i < m_modules.size() ) ? m_modules[i].c_str() : "the streaming", e.what() );
            m_monitor.IncStartFail();
            StopStreaming();
            m_state = QC_OBJECT_STATE_READY;
            status = QC_STATUS_FAIL;
        }
    }

    if ( QC_STATUS_OK == status )
    {
        {
            std::lock_guard<std::mutex> lock( m_watchdogLock );
            m_bRunning = true;
        }
        m_watchdog = std::thread( &RealSenseImpl::WatchdogMain, this );
    }

    return status;
}

void RealSenseImpl::OnVideo( const rs2::frame &frame )
{
    m_lastSampleNs = ClockNs( CLOCK_MONOTONIC );

    try
    {
        if ( m_bSyncFrames && ( nullptr != m_asyncer ) )
        {
            m_asyncer->invoke( frame );
        }
        else
        {
            DispatchVideo( frame );
        }
    }
    catch ( const std::exception &e )
    {
        m_monitor.IncError();
        QC_ERROR( "RealSense frame error: %s", e.what() );
    }
}

void RealSenseImpl::OnSynced( const rs2::frame &frame )
{
    try
    {
        if ( rs2::frameset frames = frame.as<rs2::frameset>() )
        {
            for ( size_t i = 0; i < frames.size(); i++ )
            {
                DispatchVideo( frames[i] );
            }

            rs2::depth_frame depth = frames.get_depth_frame();
            rs2::video_frame color = frames.get_color_frame();
            if ( m_config.enablePointcloud && depth )
            {
                if ( ( false == m_config.pointcloudTexture ) || color )
                {
                    HandlePointcloud( depth, color );
                }
                else
                {
                    QC_DEBUG( "no color frame to texture the pointcloud of depth frame %llu",
                              static_cast<unsigned long long>( depth.get_frame_number() ) );
                }
            }
            if ( ( nullptr != m_align ) && depth && color )
            {
                // Aligning maps depth to the color viewpoint.
                rs2::frameset aligned = m_align->process( frames );
                if ( rs2::depth_frame alignedDepth = aligned.get_depth_frame() )
                {
                    HandleImage( QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR, alignedDepth );
                }
            }
        }
        else
        {
            DispatchVideo( frame );
        }
    }
    catch ( const std::exception &e )
    {
        m_monitor.IncError();
        QC_ERROR( "RealSense frameset error: %s", e.what() );
    }
}

void RealSenseImpl::DispatchVideo( const rs2::frame &frame )
{
    const rs2::stream_profile profile = frame.get_profile();
    QCRealSenseStream_e stream = QC_REALSENSE_STREAM_NONE;

    switch ( profile.stream_type() )
    {
        case RS2_STREAM_COLOR:
            stream = QC_REALSENSE_STREAM_COLOR;
            break;
        case RS2_STREAM_DEPTH:
            stream = QC_REALSENSE_STREAM_DEPTH;
            break;
        case RS2_STREAM_INFRARED:
            stream = ( 2 == profile.stream_index() ) ? QC_REALSENSE_STREAM_INFRA2
                                                     : QC_REALSENSE_STREAM_INFRA1;
            break;
        default:
            break;
    }

    if ( ( QC_REALSENSE_STREAM_NONE != stream ) && m_streams[stream].bEnabled )
    {
        if ( rs2::video_frame video = frame.as<rs2::video_frame>() )
        {
            HandleImage( stream, video );
        }
    }
}

void RealSenseImpl::HandleImage( QCRealSenseStream_e stream, const rs2::video_frame &frame )
{
    const uint32_t index = AcquireBuffer( stream );

    if ( UINT32_MAX == index )
    {
        // Every buffer of the stream is held by the application: drop the frame like a full ROS
        // queue.
        m_monitor.IncDrop( stream );
        QC_DEBUG( "no free %s buffer, drop frame %llu", RealSenseStreamName( stream ),
                  static_cast<unsigned long long>( frame.get_frame_number() ) );
    }
    else
    {
        RealSenseImageDescriptor_t &out = m_pools[stream].images[index];

        if ( QC_STATUS_OK != CopyImage( frame, out ) )
        {
            ReleaseBuffer( stream, index );
            m_monitor.IncError();
        }
        else
        {
            const FrameTiming timing = Timing( frame );
            RealSenseFrameInfo_t info;
            (void) out.frameId.Assign( m_opticalIds[stream].c_str() ); /* checked in Initialize() */
            out.timestampNs = timing.stampNs;
            out.frameNumber = timing.frameNumber;
            FillInfo( info, stream, timing );
            m_monitor.OnFrame( stream, timing.stampNs, timing.frameNumber );
            Deliver( out, info );
        }
    }
}

QCStatus_e RealSenseImpl::CopyImage( const rs2::video_frame &frame,
                                     RealSenseImageDescriptor_t &out )
{
    QCStatus_e status = QC_STATUS_OK;
    const int width = frame.get_width();
    const int height = frame.get_height();
    const int srcStride = frame.get_stride_in_bytes();
    const uint32_t bytesPerPixel = RealSenseEncodingBytesPerPixel( out.encoding.CStr() );
    const size_t rowBytes = static_cast<size_t>( out.width ) * bytesPerPixel;

    if ( ( static_cast<int>( out.width ) != width ) ||
         ( static_cast<int>( out.height ) != height ) )
    {
        QC_ERROR( "unexpected %s frame size %dx%d, expected %ux%u",
                  RealSenseStreamName( out.stream ), width, height, out.width, out.height );
        status = QC_STATUS_FAIL;
    }
    else if ( static_cast<int>( bytesPerPixel ) != frame.get_bytes_per_pixel() )
    {
        QC_ERROR( "unexpected frame format %s for %s",
                  rs2_format_to_string( frame.get_profile().format() ), out.encoding.CStr() );
        status = QC_STATUS_FAIL;
    }
    else if ( ( srcStride < 0 ) || ( static_cast<size_t>( srcStride ) < rowBytes ) ||
              ( nullptr == frame.get_data() ) )
    {
        QC_ERROR( "invalid frame data, stride=%d", srcStride );
        status = QC_STATUS_FAIL;
    }
    else
    {
        const uint8_t *pSrc = static_cast<const uint8_t *>( frame.get_data() );
        uint8_t *pDst = static_cast<uint8_t *>( out.GetDataPtr() );
        const int fd = DmaBufFd( out );

        SyncDmaBuf( fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE );
        if ( ( static_cast<size_t>( srcStride ) == rowBytes ) && ( out.step == rowBytes ) )
        {
            (void) memcpy( pDst, pSrc, rowBytes * out.height );
        }
        else
        {
            // The rows of the DMA buffer may be padded for the HW alignment: copy row by row.
            for ( uint32_t row = 0; row < out.height; row++ )
            {
                (void) memcpy( pDst, pSrc, rowBytes );
                pDst += out.step;
                pSrc += srcStride;
            }
        }
        SyncDmaBuf( fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE );

        out.isBigEndian = false;
        out.validSize = out.GetImageSize();
    }

    return status;
}

void RealSenseImpl::HandlePointcloud( const rs2::depth_frame &depth, const rs2::video_frame &color )
{
    const QCRealSenseStream_e stream = QC_REALSENSE_STREAM_POINTCLOUD;
    const uint32_t index = AcquireBuffer( stream );

    if ( UINT32_MAX == index )
    {
        m_monitor.IncDrop( stream );
        return;
    }

    PointCloudDescriptor_t &out = m_pools[stream].clouds[index];
    const bool bTexture = m_config.pointcloudTexture && color;
    if ( bTexture )
    {
        m_pointcloud->map_to( color );
    }
    const rs2::points points = m_pointcloud->calculate( depth );
    const size_t count = points.size();
    const rs2::vertex *pVertex = points.get_vertices();
    const rs2::texture_coordinate *pUv = bTexture ? points.get_texture_coordinates() : nullptr;
    const uint32_t depthWidth = static_cast<uint32_t>( depth.get_width() );
    const uint32_t depthHeight = static_cast<uint32_t>( depth.get_height() );
    const uint32_t pointStep = RealSensePointStep( m_config.pointcloudTexture );
    const uint64_t capacity = ( out.size - out.offset ) / pointStep;

    if ( ( nullptr == pVertex ) || ( count > capacity ) ||
         ( m_config.orderedPointcloud &&
           ( static_cast<uint64_t>( depthWidth ) * depthHeight > capacity ) ) )
    {
        QC_ERROR( "pointcloud of %u points does not fit buffer %u", static_cast<uint32_t>( count ),
                  index );
        ReleaseBuffer( stream, index );
        m_monitor.IncError();
        return;
    }

    // The color texture, in PointCloud2 rgb order (b, g, r bytes of a float32).
    const uint8_t *pColor = bTexture ? static_cast<const uint8_t *>( color.get_data() ) : nullptr;
    const int colorWidth = bTexture ? color.get_width() : 0;
    const int colorHeight = bTexture ? color.get_height() : 0;
    const int colorStride = bTexture ? color.get_stride_in_bytes() : 0;
    const int colorBpp = bTexture ? color.get_bytes_per_pixel() : 0;
    const bool bBgr = ( QC_REALSENSE_ENCODING_BGR8 == m_config.colorEncoding ) ||
                      ( QC_REALSENSE_ENCODING_BGRA8 == m_config.colorEncoding );

    uint8_t *pDst = static_cast<uint8_t *>( out.GetDataPtr() );
    const int fd = DmaBufFd( out );
    uint32_t written = 0;
    SyncDmaBuf( fd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE );
    for ( size_t i = 0; i < count; i++ )
    {
        const rs2::vertex &v = pVertex[i];
        bool bValidColor = false;
        int px = 0;
        int py = 0;
        if ( nullptr != pUv )
        {
            const float u = pUv[i].u;
            const float w = pUv[i].v;
            bValidColor = ( u >= 0.0f ) && ( u <= 1.0f ) && ( w >= 0.0f ) && ( w <= 1.0f );
            px = std::min( static_cast<int>( u * static_cast<float>( colorWidth ) ),
                           colorWidth - 1 );
            py = std::min( static_cast<int>( w * static_cast<float>( colorHeight ) ),
                           colorHeight - 1 );
        }
        const bool bValid = ( v.z > 0.0f ) &&
                            ( ( nullptr == pUv ) || bValidColor || m_config.allowNoTexturePoints );
        if ( ( false == bValid ) && ( false == m_config.orderedPointcloud ) )
        {
            continue;
        }

        uint8_t *pPoint = pDst + static_cast<size_t>( written ) * pointStep;
        (void) memcpy( pPoint, &v.x, sizeof( float ) );
        (void) memcpy( pPoint + 4, &v.y, sizeof( float ) );
        (void) memcpy( pPoint + 8, &v.z, sizeof( float ) );
        if ( m_config.pointcloudTexture )
        {
            uint8_t rgb[4] = { 0u, 0u, 0u, 0u };
            if ( bValidColor && ( nullptr != pColor ) )
            {
                const uint8_t *pPixel = pColor + static_cast<size_t>( py ) * colorStride +
                                        static_cast<size_t>( px ) * colorBpp;
                rgb[0] = bBgr ? pPixel[0] : pPixel[2]; /* b */
                rgb[1] = pPixel[1];                    /* g */
                rgb[2] = bBgr ? pPixel[2] : pPixel[0]; /* r */
            }
            (void) memcpy( pPoint + 12, rgb, sizeof( rgb ) );
        }
        written++;
    }
    SyncDmaBuf( fd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE );

    using QRB::Memory::PointField_t;
    out.numFields = 0;
    out.fields[out.numFields++] = PointField_t( "x", 0, QRB::Memory::POINT_FIELD_TYPE_FLOAT32 );
    out.fields[out.numFields++] = PointField_t( "y", 4, QRB::Memory::POINT_FIELD_TYPE_FLOAT32 );
    out.fields[out.numFields++] = PointField_t( "z", 8, QRB::Memory::POINT_FIELD_TYPE_FLOAT32 );
    if ( m_config.pointcloudTexture )
    {
        out.fields[out.numFields++] =
                PointField_t( "rgb", 12, QRB::Memory::POINT_FIELD_TYPE_FLOAT32 );
    }
    if ( m_config.orderedPointcloud )
    {
        out.width = depthWidth;
        out.height = depthHeight;
        out.isDense = false;
    }
    else
    {
        out.width = written;
        out.height = 1;
        out.isDense = true;
    }
    out.isBigEndian = false;
    out.pointStep = pointStep;
    out.rowStep = out.width * pointStep;
    out.validSize = static_cast<size_t>( out.rowStep ) * out.height;

    const FrameTiming timing = Timing( depth );
    out.timestampNs = timing.stampNs;
    (void) out.frameId.Assign( m_opticalIds[stream].c_str() );
    RealSenseFrameInfo_t info;
    FillInfo( info, stream, timing );
    m_monitor.OnFrame( stream, timing.stampNs, timing.frameNumber );
    Deliver( out, info );
}

void RealSenseImpl::OnMotion( const rs2::frame &frame )
{
    m_lastSampleNs = ClockNs( CLOCK_MONOTONIC );

    try
    {
        if ( rs2::motion_frame motion = frame.as<rs2::motion_frame>() )
        {
            HandleMotion( motion );
        }
    }
    catch ( const std::exception &e )
    {
        m_monitor.IncError();
        QC_ERROR( "RealSense motion error: %s", e.what() );
    }
}

void RealSenseImpl::HandleMotion( const rs2::motion_frame &frame )
{
    const bool bAccel = ( RS2_STREAM_ACCEL == frame.get_profile().stream_type() );
    const QCRealSenseStream_e stream =
            bAccel ? QC_REALSENSE_STREAM_ACCEL : QC_REALSENSE_STREAM_GYRO;
    const MotionSample sample{ frame.get_motion_data(), Timing( frame ) };
    const bool bUnite = ( QC_REALSENSE_UNITE_IMU_NONE != m_config.uniteImuMethod ) &&
                        m_config.enableAccel && m_config.enableGyro;
    /* The united samples to deliver, outside m_imuLock: gyro sample and accel of each. */
    std::vector<std::pair<MotionSample, rs2_vector>> united;

    if ( ( false == bUnite ) || m_config.rawImu )
    {
        DeliverMotion( stream, sample );
    }
    if ( false == bUnite )
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock( m_imuLock );
        const uint64_t ts = sample.timing.stampNs;
        if ( QC_REALSENSE_UNITE_IMU_COPY == m_config.uniteImuMethod )
        {
            // Each gyro sample with the latest accel sample.
            if ( bAccel )
            {
                m_lastAccel = sample;
                m_bHaveAccel = true;
            }
            else if ( m_bHaveAccel )
            {
                united.emplace_back( sample, m_lastAccel.data );
            }
            else
            {
                /* no accel sample yet to unite the gyro sample with */
            }
        }
        else if ( bAccel )
        {
            // The gyro samples since the previous accel sample get the accel interpolated at their
            // timestamp, as realsense2_camera.
            if ( m_bHaveAccel )
            {
                const MotionSample &a0 = m_lastAccel;
                const double t0 = static_cast<double>( a0.timing.stampNs );
                const double dt = static_cast<double>( ts ) - t0;
                for ( const MotionSample &gyro : m_pendingGyros )
                {
                    const double alpha =
                            ( dt > 0.0 ) ? ( static_cast<double>( gyro.timing.stampNs ) - t0 ) / dt
                                         : 0.0;
                    const rs2_vector accel{ static_cast<float>( a0.data.x * ( 1.0 - alpha ) +
                                                                sample.data.x * alpha ),
                                            static_cast<float>( a0.data.y * ( 1.0 - alpha ) +
                                                                sample.data.y * alpha ),
                                            static_cast<float>( a0.data.z * ( 1.0 - alpha ) +
                                                                sample.data.z * alpha ) };
                    united.emplace_back( gyro, accel );
                }
            }
            m_pendingGyros.clear();
            m_lastAccel = sample;
            m_bHaveAccel = true;
        }
        else if ( m_bHaveAccel && ( ts >= m_lastAccel.timing.stampNs ) )
        {
            m_pendingGyros.push_back( sample );
            if ( m_pendingGyros.size() > kMaxPendingGyros )
            {
                m_pendingGyros.pop_front();
            }
        }
        else
        {
            /* a gyro sample before the first accel sample cannot be interpolated */
        }
    }

    for ( const auto &entry : united )
    {
        DeliverImu( entry.first, entry.second );
    }
}

void RealSenseImpl::DeliverMotion( QCRealSenseStream_e stream, const MotionSample &sample )
{
    ImuDescriptor_t imu;
    RealSenseFrameInfo_t info;

    (void) imu.frameId.Assign( m_opticalIds[stream].c_str() ); /* checked in Initialize() */
    imu.timestampNs = sample.timing.stampNs;
    imu.id = sample.timing.frameNumber;
    imu.orientation = Quaternion_t();
    SetDiagonal( imu.orientation_covariance, 0.0 );
    imu.orientation_covariance[0] = -1.0;
    if ( QC_REALSENSE_STREAM_GYRO == stream )
    {
        imu.angular_velocity.x = sample.data.x;
        imu.angular_velocity.y = sample.data.y;
        imu.angular_velocity.z = sample.data.z;
    }
    else
    {
        imu.linear_acceleration.x = sample.data.x;
        imu.linear_acceleration.y = sample.data.y;
        imu.linear_acceleration.z = sample.data.z;
    }
    SetDiagonal( imu.angular_velocity_covariance, m_config.angularVelocityCov );
    SetDiagonal( imu.linear_acceleration_covariance, m_config.linearAccelCov );

    FillInfo( info, stream, sample.timing );
    m_monitor.OnFrame( stream, imu.timestampNs, sample.timing.frameNumber );
    Deliver( imu, info );
}

void RealSenseImpl::DeliverImu( const MotionSample &gyro, const rs2_vector &accel )
{
    ImuDescriptor_t imu;
    RealSenseFrameInfo_t info;

    (void) imu.frameId.Assign( m_opticalIds[QC_REALSENSE_STREAM_IMU].c_str() );
    imu.timestampNs = gyro.timing.stampNs;
    // The gyro frame number, as the image frameNumber: a gap is a lost gyro sample.
    imu.id = gyro.timing.frameNumber;
    // The orientation is not estimated: identity, marked unknown as in sensor_msgs/Imu.
    imu.orientation = Quaternion_t();
    SetDiagonal( imu.orientation_covariance, 0.0 );
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = gyro.data.x;
    imu.angular_velocity.y = gyro.data.y;
    imu.angular_velocity.z = gyro.data.z;
    SetDiagonal( imu.angular_velocity_covariance, m_config.angularVelocityCov );
    imu.linear_acceleration.x = accel.x;
    imu.linear_acceleration.y = accel.y;
    imu.linear_acceleration.z = accel.z;
    SetDiagonal( imu.linear_acceleration_covariance, m_config.linearAccelCov );

    FillInfo( info, QC_REALSENSE_STREAM_IMU, gyro.timing );
    m_monitor.OnFrame( QC_REALSENSE_STREAM_IMU, imu.timestampNs, gyro.timing.frameNumber );
    Deliver( imu, info );
}

int64_t RealSenseImpl::ToClockNs( int64_t realtimeNs ) const
{
    int64_t ns = realtimeNs;
    if ( QC_REALSENSE_CLOCK_MONOTONIC == m_config.clock )
    {
        ns += ClockNs( CLOCK_MONOTONIC ) - ClockNs( CLOCK_REALTIME );
    }
    return ns;
}

uint64_t RealSenseImpl::StampNs( const rs2::frame &frame, double sensorMs,
                                 rs2_timestamp_domain domain )
{
    int64_t realtimeNs = 0;

    if ( RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK == domain )
    {
        // realsense2_camera frameSystemTimeSec: the camera clock mapped to the host time at the
        // first frame, mapped again when the camera clock goes back (reset).
        std::lock_guard<std::mutex> lock( m_timeBaseLock );
        const int uid = frame.get_profile().unique_id();
        if ( false == m_bTimeBase )
        {
            QC_WARN( "camera '%s' timestamps are in the hardware clock domain",
                     m_config.cameraName.c_str() );
            m_hostTimeBaseNs = ClockNs( CLOCK_REALTIME );
            m_cameraTimeBaseMs = sensorMs;
            m_bTimeBase = true;
        }
        else
        {
            auto it = m_previousCameraMs.find( uid );
            if ( ( m_previousCameraMs.end() != it ) && ( it->second > sensorMs ) )
            {
                QC_WARN( "camera '%s' hardware clock reset, new time base",
                         m_config.cameraName.c_str() );
                m_hostTimeBaseNs = ClockNs( CLOCK_REALTIME );
                m_cameraTimeBaseMs = sensorMs;
                m_previousCameraMs.clear();
            }
        }
        m_previousCameraMs[uid] = sensorMs;
        realtimeNs = m_hostTimeBaseNs + static_cast<int64_t>( std::llround(
                                                ( sensorMs - m_cameraTimeBaseMs ) * kNsPerMs ) );
    }
    else
    {
        // System and global time are milliseconds of the host CLOCK_REALTIME.
        realtimeNs = static_cast<int64_t>( std::llround( sensorMs * kNsPerMs ) );
    }

    const int64_t ns = ToClockNs( realtimeNs );
    return ( ns < 0 ) ? 0u : static_cast<uint64_t>( ns );
}

RealSenseImpl::FrameTiming RealSenseImpl::Timing( const rs2::frame &frame )
{
    FrameTiming t;
    const rs2_timestamp_domain domain = frame.get_frame_timestamp_domain();

    t.frameNumber = static_cast<uint64_t>( frame.get_frame_number() );
    t.sensorMs = frame.get_timestamp();
    switch ( domain )
    {
        case RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK:
            t.domain = QC_REALSENSE_TIMESTAMP_DOMAIN_HARDWARE_CLOCK;
            break;
        case RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME:
            t.domain = QC_REALSENSE_TIMESTAMP_DOMAIN_SYSTEM_TIME;
            break;
        case RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME:
            t.domain = QC_REALSENSE_TIMESTAMP_DOMAIN_GLOBAL_TIME;
            break;
        default:
            t.domain = QC_REALSENSE_TIMESTAMP_DOMAIN_UNKNOWN;
            break;
    }
    t.stampNs = StampNs( frame, t.sensorMs, domain );
    if ( frame.supports_frame_metadata( RS2_FRAME_METADATA_SENSOR_TIMESTAMP ) )
    {
        t.bHaveMetadataSensorTs = true;
        t.metadataSensorTsUs = static_cast<uint64_t>(
                frame.get_frame_metadata( RS2_FRAME_METADATA_SENSOR_TIMESTAMP ) );
    }
    // The host time of arrival of the frame (ms, CLOCK_REALTIME), else now.
    int64_t hostNs = ClockNs( CLOCK_REALTIME );
    if ( frame.supports_frame_metadata( RS2_FRAME_METADATA_TIME_OF_ARRIVAL ) )
    {
        hostNs = static_cast<int64_t>(
                         frame.get_frame_metadata( RS2_FRAME_METADATA_TIME_OF_ARRIVAL ) ) *
                 kNsPerMs;
    }
    hostNs = ToClockNs( hostNs );
    t.hostReceiveNs = ( hostNs < 0 ) ? 0u : static_cast<uint64_t>( hostNs );

    return t;
}

void RealSenseImpl::FillInfo( RealSenseFrameInfo_t &info, QCRealSenseStream_e stream,
                              const FrameTiming &timing ) const
{
    info.stream = stream;
    (void) info.cameraSerial.Assign( m_deviceInfo.serial.c_str() );
    (void) info.cameraName.Assign( m_config.cameraName.c_str() );
    info.frameNumber = timing.frameNumber;
    info.timestampNs = timing.stampNs;
    info.sensorTimestampMs = timing.sensorMs;
    info.timestampDomain = timing.domain;
    info.bHaveMetadataSensorTimestamp = timing.bHaveMetadataSensorTs;
    info.metadataSensorTimestampUs = timing.metadataSensorTsUs;
    info.hostReceiveNs = timing.hostReceiveNs;
    if ( ( QC_REALSENSE_STREAM_DEPTH == stream ) ||
         ( QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR == stream ) ||
         ( QC_REALSENSE_STREAM_POINTCLOUD == stream ) )
    {
        info.depthScale = m_depthScale;
    }
    if ( IsImageStream( stream ) || ( QC_REALSENSE_STREAM_POINTCLOUD == stream ) )
    {
        info.cameraInfo = m_cameraInfo[stream];
    }
}

void RealSenseImpl::Deliver( QCBufferDescriptorBase_t &payload, RealSenseFrameInfo_t &info )
{
    NodeFrameDescriptorFlat<2> frameDesc;
    (void) frameDesc.SetBuffer( 0, payload );
    (void) frameDesc.SetBuffer( 1, info );
    QCNodeEventInfo_t eventInfo( frameDesc, m_nodeId, QC_STATUS_OK, m_state.load() );

    std::lock_guard<std::mutex> lock( m_callbackLock );
    const void *pPrevious = tl_pCallbackOwner;
    tl_pCallbackOwner = this;
    m_callback( eventInfo );
    tl_pCallbackOwner = pPrevious;
}

void RealSenseImpl::OnDevicesChanged( rs2::event_information &info )
{
    try
    {
        if ( ( nullptr != m_device ) && info.was_removed( *m_device ) )
        {
            QC_ERROR( "camera '%s' (serial %s) disconnected", m_config.cameraName.c_str(),
                      m_deviceInfo.serial.c_str() );
            m_monitor.SetConnected( false );
            m_monitor.IncError();
            {
                std::lock_guard<std::mutex> lock( m_infoLock );
                m_deviceInfo.bConnected = false;
            }
            if ( QC_OBJECT_STATE_RUNNING == m_state )
            {
                // Tell the application: an event with an error status and no sample.
                RealSenseFrameInfo_t frameInfo;
                frameInfo.stream = QC_REALSENSE_STREAM_NONE;
                (void) frameInfo.cameraSerial.Assign( m_deviceInfo.serial.c_str() );
                (void) frameInfo.cameraName.Assign( m_config.cameraName.c_str() );
                NodeFrameDescriptorFlat<2> frameDesc;
                (void) frameDesc.SetBuffer( 0, frameInfo );
                (void) frameDesc.SetBuffer( 1, frameInfo );
                QCNodeEventInfo_t eventInfo( frameDesc, m_nodeId, QC_STATUS_FAIL, m_state.load() );
                std::lock_guard<std::mutex> lock( m_callbackLock );
                const void *pPrevious = tl_pCallbackOwner;
                tl_pCallbackOwner = this;
                m_callback( eventInfo );
                tl_pCallbackOwner = pPrevious;
            }
        }
        for ( rs2::device dev : info.get_new_devices() )
        {
            if ( DeviceInfoString( dev, RS2_CAMERA_INFO_SERIAL_NUMBER ) == m_deviceInfo.serial )
            {
                QC_WARN( "camera '%s' (serial %s) connected again: stop, deinitialize and "
                         "initialize the node to use it",
                         m_config.cameraName.c_str(), m_deviceInfo.serial.c_str() );
            }
        }
    }
    catch ( const std::exception &e )
    {
        QC_ERROR( "RealSense device event error: %s", e.what() );
    }
}

uint32_t RealSenseImpl::AcquireBuffer( QCRealSenseStream_e stream )
{
    uint32_t index = UINT32_MAX;
    std::lock_guard<std::mutex> lock( m_bufLock );
    std::vector<bool> &inUse = m_pools[stream].inUse;

    for ( size_t i = 0; i < inUse.size(); i++ )
    {
        if ( false == inUse[i] )
        {
            inUse[i] = true;
            index = static_cast<uint32_t>( i );
            break;
        }
    }

    return index;
}

void RealSenseImpl::ReleaseBuffer( QCRealSenseStream_e stream, uint32_t index )
{
    std::lock_guard<std::mutex> lock( m_bufLock );
    std::vector<bool> &inUse = m_pools[stream].inUse;
    if ( index < inUse.size() )
    {
        inUse[index] = false;
    }
}

QCStatus_e RealSenseImpl::ReleaseFrame( QCFrameDescriptorNodeIfs &frameDesc )
{
    QCStatus_e status = QC_STATUS_OK;
    const QCObjectState_e state = m_state;
    const QCBufferDescriptorBase_t &bufDesc = frameDesc.GetBuffer( 0 );

    /* Buffers may still be released after Stop(), as consumers can hold frames for a while. */
    if ( ( QC_OBJECT_STATE_RUNNING != state ) && ( QC_OBJECT_STATE_READY != state ) )
    {
        QC_ERROR( "RealSense release failed due to wrong state: %d", state );
        status = QC_STATUS_BAD_STATE;
    }
    else if ( ( nullptr != dynamic_cast<const ImuDescriptor_t *>( &bufDesc ) ) ||
              ( nullptr != dynamic_cast<const RealSenseFrameInfo_t *>( &bufDesc ) ) )
    {
        /* An IMU sample or a frame info holds no buffer: nothing to give back. */
    }
    else
    {
        /* Match by memory address, so that a copy of the delivered descriptor can be released
         * too. */
        int stream = -1;
        uint32_t index = UINT32_MAX;
        for ( int s = 0; ( s < QC_REALSENSE_STREAM_COUNT ) && ( UINT32_MAX == index ); s++ )
        {
            const Pool &pool = m_pools[s];
            for ( size_t i = 0; i < pool.Size(); i++ )
            {
                const void *pBuf = ( i < pool.images.size() )
                                           ? pool.images[i].pBuf
                                           : pool.clouds[i - pool.images.size()].pBuf;
                if ( ( nullptr != bufDesc.pBuf ) && ( pBuf == bufDesc.pBuf ) )
                {
                    stream = s;
                    index = static_cast<uint32_t>( i );
                    break;
                }
            }
        }

        if ( UINT32_MAX == index )
        {
            QC_ERROR( "the buffer to release is not a RealSense buffer" );
            status = QC_STATUS_INVALID_BUF;
        }
        else
        {
            std::lock_guard<std::mutex> lock( m_bufLock );
            std::vector<bool> &inUse = m_pools[stream].inUse;
            if ( false == inUse[index] )
            {
                QC_ERROR( "%s buffer %u released twice",
                          RealSenseStreamName( static_cast<QCRealSenseStream_e>( stream ) ),
                          index );
                status = QC_STATUS_BAD_ARGUMENTS;
            }
            else
            {
                inUse[index] = false;
            }
        }
    }

    return status;
}

void RealSenseImpl::WatchdogMain()
{
    const int64_t timeoutNs = static_cast<int64_t>( m_config.timeoutMs ) * kNsPerMs;
    std::unique_lock<std::mutex> lock( m_watchdogLock );

    while ( m_bRunning )
    {
        if ( m_watchdogCv.wait_for( lock, std::chrono::milliseconds( m_config.timeoutMs ),
                                    [this]() { return false == m_bRunning; } ) )
        {
            break;
        }
        if ( ( ClockNs( CLOCK_MONOTONIC ) - m_lastSampleNs ) >= timeoutNs )
        {
            m_monitor.IncTimeout();
            QC_WARN( "camera '%s': no RealSense sample within %u ms", m_config.cameraName.c_str(),
                     m_config.timeoutMs );
        }
    }
}

void RealSenseImpl::StopStreaming()
{
    {
        std::lock_guard<std::mutex> lock( m_watchdogLock );
        m_bRunning = false;
    }
    m_watchdogCv.notify_all();
    if ( m_watchdog.joinable() )
    {
        m_watchdog.join();
    }

    // sensor::stop() returns once the callbacks of the sensor have returned, so no frame reaches
    // the syncer afterwards.
    for ( size_t i = 0; i < m_sensorStarted.size(); i++ )
    {
        if ( m_sensorStarted[i] )
        {
            try
            {
                m_sensors[i].stop();
            }
            catch ( const std::exception &e )
            {
                QC_ERROR( "failed to stop %s: %s", m_modules[i].c_str(), e.what() );
            }
            m_sensorStarted[i] = false;
        }
    }
    for ( size_t i = 0; i < m_sensorOpen.size(); i++ )
    {
        if ( m_sensorOpen[i] )
        {
            try
            {
                m_sensors[i].close();
            }
            catch ( const std::exception &e )
            {
                QC_ERROR( "failed to close %s: %s", m_modules[i].c_str(), e.what() );
            }
            m_sensorOpen[i] = false;
        }
    }
    m_asyncer.reset();
}

QCStatus_e RealSenseImpl::Stop()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( QC_OBJECT_STATE_RUNNING != m_state )
    {
        QC_ERROR( "RealSense stop failed due to wrong state: %d", m_state.load() );
        status = QC_STATUS_BAD_STATE;
    }
    else if ( this == tl_pCallbackOwner )
    {
        QC_ERROR( "RealSense Stop must not be called from the event callback" );
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        StopStreaming();
        m_state = QC_OBJECT_STATE_READY;
    }

    return status;
}

void RealSenseImpl::Reset()
{
    if ( nullptr != m_watch )
    {
        // No device event reaches the node from now on.
        std::lock_guard<std::mutex> lock( m_watch->lock );
        m_watch->pImpl = nullptr;
    }
    UnregisterSyncRole();
    m_align.reset();
    m_pointcloud.reset();
    m_asyncer.reset();
    m_sensorProfiles.clear();
    m_sensorOpen.clear();
    m_sensorStarted.clear();
    for ( StreamSetup &setup : m_streams )
    {
        setup = StreamSetup();
    }
    m_sensors.clear();
    m_modules.clear();
    m_staticTfs.clear();
    m_device.reset();
    m_ctx.reset();
    m_watch.reset();
    for ( Pool &pool : m_pools )
    {
        pool = Pool();
    }
    m_depthScale = 0.0f;
    m_callback = nullptr;
}

QCStatus_e RealSenseImpl::DeInitialize()
{
    QCStatus_e status = QC_STATUS_OK;

    if ( QC_OBJECT_STATE_READY != m_state )
    {
        QC_ERROR( "RealSense deinit failed due to wrong state: %d", m_state.load() );
        status = QC_STATUS_BAD_STATE;
    }
    else
    {
        Reset();
        m_state = QC_OBJECT_STATE_INITIAL;
    }

    return status;
}

}   // namespace Node
}   // namespace QC
