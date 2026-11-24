// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "RealSenseImpl.hpp"

#include <linux/dma-buf.h>
#include <sys/ioctl.h>
#include <time.h>

#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <exception>
#include <utility>
#include <vector>

namespace QC
{
namespace Node
{

namespace
{

using QRB::Memory::ImuDescriptor_t;

constexpr int64_t kNsPerMs = 1000000;

/* linear_interpolation: gyro samples kept at most while waiting for an accel sample, so that a
 * stalled accel stream does not grow the queue without bound. */
constexpr size_t kMaxPendingGyros = 64;

/* The RealSense node whose application callback runs on this thread, to reject Stop() from it:
 * librealsense waits in pipeline::stop() for its callbacks to return. */
thread_local const void *tl_pCallbackOwner = nullptr;

int64_t ClockNs( clockid_t clock )
{
    struct timespec ts = {};
    (void) clock_gettime( clock, &ts );
    return static_cast<int64_t>( ts.tv_sec ) * 1000000000LL + static_cast<int64_t>( ts.tv_nsec );
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
 * @brief Begin or end a CPU write of a DMA-BUF, so the pixels reach the consumer through the
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

}   // namespace

RealSenseImpl::RealSenseImpl( QCNodeID_t &nodeId, Logger &logger, RealSenseMonitoring &monitor )
    : m_nodeId( nodeId ),
      m_logger( logger ),
      m_monitor( monitor )
{}

RealSenseImpl::~RealSenseImpl()
{
    StopStreaming();
}

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
        status = SetupBuffers( buffers );
    }

    if ( QC_STATUS_OK == status )
    {
        // The descriptors hold fixed-size frame ids, check them once here.
        QRB::Memory::FrameId_t frameId;
        for ( const std::string *pId :
              { &m_config.colorFrameId, &m_config.depthFrameId, &m_config.imuFrameId } )
        {
            if ( QC_STATUS_OK != frameId.Assign( pId->c_str() ) )
            {
                QC_ERROR( "frame id '%s' is longer than %u characters", pId->c_str(),
                          static_cast<uint32_t>( QRB::Memory::QRB_FRAME_ID_MAX_LEN ) );
                status = QC_STATUS_BAD_ARGUMENTS;
            }
        }
    }

    if ( QC_STATUS_OK == status )
    {
        status = OpenCamera();
    }

    if ( QC_STATUS_OK == status )
    {
        m_monitor.Reset();
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

void RealSenseImpl::Reset()
{
    m_align.reset();
    m_pipeline.reset();
    m_rsConfig.reset();
    for ( ImagePool &pool : m_pools )
    {
        pool.images.clear();
        pool.inUse.clear();
    }
    m_callback = nullptr;
}

QCStatus_e RealSenseImpl::SetupBuffers(
        std::vector<std::reference_wrapper<QCBufferDescriptorBase_t>> &buffers )
{
    QCStatus_e status = QC_STATUS_OK;

    for ( ImagePool &pool : m_pools )
    {
        pool.images.clear();
        pool.inUse.clear();
    }
    for ( size_t i = 0; ( QC_STATUS_OK == status ) && ( i < buffers.size() ); i++ )
    {
        status = AddBuffer( i, buffers[i].get() );
    }

    const bool enabled[kImageStreamCount] = { m_config.enableColor, m_config.enableDepth };
    const char *names[kImageStreamCount] = { "color", "depth" };
    for ( int s = 0; ( QC_STATUS_OK == status ) && ( s < kImageStreamCount ); s++ )
    {
        const size_t count = m_pools[s].images.size();
        if ( enabled[s] && ( ( 0 == count ) || ( count > QC_REALSENSE_MAX_BUFFERS ) ) )
        {
            QC_ERROR( "the %s stream requires 1 to %u buffers, got %u", names[s],
                      QC_REALSENSE_MAX_BUFFERS, static_cast<uint32_t>( count ) );
            status = QC_STATUS_BAD_ARGUMENTS;
        }
        else if ( ( false == enabled[s] ) && ( 0 != count ) )
        {
            QC_ERROR( "got %u buffers for the disabled %s stream", static_cast<uint32_t>( count ),
                      names[s] );
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
    ImageStream stream = kColor;
    uint32_t width = 0;
    uint32_t height = 0;

    if ( nullptr == pImage )
    {
        QC_ERROR( "buffer %u is not a RealSenseImageDescriptor_t", i );
        status = QC_STATUS_INVALID_BUF;
    }
    else if ( nullptr == pImage->pBuf )
    {
        QC_ERROR( "buffer %u has no memory", i );
        status = QC_STATUS_INVALID_BUF;
    }
    else if ( pImage->encoding == QC_REALSENSE_ENCODING_RGB8 )
    {
        stream = kColor;
        width = m_config.colorWidth;
        height = m_config.colorHeight;
    }
    else if ( pImage->encoding == QC_REALSENSE_ENCODING_16UC1 )
    {
        // An aligned depth image has the color resolution.
        stream = kDepth;
        width = m_config.alignDepth ? m_config.colorWidth : m_config.depthWidth;
        height = m_config.alignDepth ? m_config.colorHeight : m_config.depthHeight;
    }
    else
    {
        QC_ERROR( "buffer %u has encoding '%s', expected rgb8 (color) or 16UC1 (depth)", i,
                  pImage->encoding.CStr() );
        status = QC_STATUS_INVALID_BUF;
    }

    if ( QC_STATUS_OK == status )
    {
        const uint64_t rowBytes = static_cast<uint64_t>( width ) *
                                  RealSenseEncodingBytesPerPixel( pImage->encoding.CStr() );
        if ( ( width != pImage->width ) || ( height != pImage->height ) )
        {
            QC_ERROR( "buffer %u is %ux%u, the stream is %ux%u", i, pImage->width, pImage->height,
                      width, height );
            status = QC_STATUS_INVALID_BUF;
        }
        else if ( ( pImage->step < rowBytes ) || ( pImage->offset > pImage->size ) ||
                  ( pImage->GetImageSize() > ( pImage->size - pImage->offset ) ) )
        {
            QC_ERROR( "buffer %u is too small: step=%u size=%u offset=%u", i, pImage->step,
                      static_cast<uint32_t>( pImage->size ),
                      static_cast<uint32_t>( pImage->offset ) );
            status = QC_STATUS_INVALID_BUF;
        }
        else
        {
            m_pools[stream].images.push_back( *pImage );
        }
    }

    return status;
}

QCStatus_e RealSenseImpl::OpenCamera()
{
    QCStatus_e status = QC_STATUS_OK;

    try
    {
        m_pipeline = std::make_unique<rs2::pipeline>();
        m_rsConfig = std::make_unique<rs2::config>();

        if ( false == m_config.serial.empty() )
        {
            m_rsConfig->enable_device( m_config.serial );
        }
        if ( m_config.enableColor )
        {
            m_rsConfig->enable_stream( RS2_STREAM_COLOR, static_cast<int>( m_config.colorWidth ),
                                       static_cast<int>( m_config.colorHeight ), RS2_FORMAT_RGB8,
                                       static_cast<int>( m_config.colorFps ) );
        }
        if ( m_config.enableDepth )
        {
            m_rsConfig->enable_stream( RS2_STREAM_DEPTH, static_cast<int>( m_config.depthWidth ),
                                       static_cast<int>( m_config.depthHeight ), RS2_FORMAT_Z16,
                                       static_cast<int>( m_config.depthFps ) );
        }
        if ( m_config.enableImu )
        {
            m_rsConfig->enable_stream( RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,
                                       static_cast<int>( m_config.accelFps ) );
            m_rsConfig->enable_stream( RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,
                                       static_cast<int>( m_config.gyroFps ) );
        }

        if ( false == m_rsConfig->can_resolve( *m_pipeline ) )
        {
            QC_ERROR( "no RealSense camera (serial '%s') supports color %s %ux%u@%u, depth %s "
                      "%ux%u@%u, imu %s",
                      m_config.serial.c_str(), m_config.enableColor ? "on" : "off",
                      m_config.colorWidth, m_config.colorHeight, m_config.colorFps,
                      m_config.enableDepth ? "on" : "off", m_config.depthWidth,
                      m_config.depthHeight, m_config.depthFps, m_config.enableImu ? "on" : "off" );
            status = QC_STATUS_FAIL;
        }
        else
        {
            rs2::pipeline_profile profile = m_rsConfig->resolve( *m_pipeline );
            rs2::device dev = profile.get_device();
            const char *pName = dev.supports( RS2_CAMERA_INFO_NAME )
                                        ? dev.get_info( RS2_CAMERA_INFO_NAME )
                                        : "unknown";
            const char *pSerial = dev.supports( RS2_CAMERA_INFO_SERIAL_NUMBER )
                                          ? dev.get_info( RS2_CAMERA_INFO_SERIAL_NUMBER )
                                          : "unknown";
            const char *pFirmware = dev.supports( RS2_CAMERA_INFO_FIRMWARE_VERSION )
                                            ? dev.get_info( RS2_CAMERA_INFO_FIRMWARE_VERSION )
                                            : "unknown";
            QC_INFO( "opened %s (serial %s, firmware %s)", pName, pSerial, pFirmware );
            if ( m_config.enableDepth )
            {
                const float depthScale = dev.first<rs2::depth_sensor>().get_depth_scale();
                QC_INFO( "depth unit %f m%s", depthScale,
                         m_config.alignDepth ? ", aligned to color" : "" );
                (void) depthScale; /* unused when the log is disabled */
            }
            (void) pName;
            (void) pSerial;
            (void) pFirmware;

            if ( m_config.alignDepth )
            {
                m_align = std::make_unique<rs2::align>( RS2_STREAM_COLOR );
            }
        }
    }
    catch ( const std::exception &e )
    {
        QC_ERROR( "failed to open the RealSense camera: %s", e.what() );
        status = QC_STATUS_FAIL;
    }

    return status;
}

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
        m_lastSampleNs = ClockNs( CLOCK_MONOTONIC );
        // Running before the first sample can arrive, so that it is delivered as running.
        m_state = QC_OBJECT_STATE_RUNNING;
        try
        {
            (void) m_pipeline->start( *m_rsConfig,
                                      [this]( rs2::frame frame ) { OnFrame( frame ); } );
            m_bStreaming = true;
        }
        catch ( const std::exception &e )
        {
            QC_ERROR( "failed to start the RealSense streaming: %s", e.what() );
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

void RealSenseImpl::OnFrame( const rs2::frame &frame )
{
    m_lastSampleNs = ClockNs( CLOCK_MONOTONIC );

    try
    {
        if ( rs2::frameset frames = frame.as<rs2::frameset>() )
        {
            // The synchronized color/depth frames. Aligning maps depth to the color viewpoint.
            if ( nullptr != m_align )
            {
                frames = m_align->process( frames );
            }
            for ( size_t i = 0; i < frames.size(); i++ )
            {
                Dispatch( frames[i] );
            }
        }
        else
        {
            // A stream that bypasses the synchronization, the IMU.
            Dispatch( frame );
        }
    }
    catch ( const std::exception &e )
    {
        m_monitor.IncError();
        QC_ERROR( "RealSense frame error: %s", e.what() );
    }
}

void RealSenseImpl::Dispatch( const rs2::frame &frame )
{
    const rs2_stream stream = frame.get_profile().stream_type();

    if ( ( RS2_STREAM_COLOR == stream ) && m_config.enableColor )
    {
        if ( rs2::video_frame video = frame.as<rs2::video_frame>() )
        {
            HandleImage( kColor, video );
        }
    }
    else if ( ( RS2_STREAM_DEPTH == stream ) && m_config.enableDepth )
    {
        if ( rs2::video_frame video = frame.as<rs2::video_frame>() )
        {
            HandleImage( kDepth, video );
        }
    }
    else if ( ( ( RS2_STREAM_ACCEL == stream ) || ( RS2_STREAM_GYRO == stream ) ) &&
              m_config.enableImu )
    {
        if ( rs2::motion_frame motion = frame.as<rs2::motion_frame>() )
        {
            HandleMotion( motion );
        }
    }
    else
    {
        /* a stream that is not delivered */
    }
}

void RealSenseImpl::HandleImage( ImageStream stream, const rs2::video_frame &frame )
{
    const uint32_t index = AcquireBuffer( stream );

    if ( UINT32_MAX == index )
    {
        // Every buffer of the stream is held by the application: drop the frame like a full ROS
        // queue.
        if ( kColor == stream )
        {
            m_monitor.IncColorDrop();
        }
        else
        {
            m_monitor.IncDepthDrop();
        }
        QC_DEBUG( "no free %s buffer, drop frame %llu", ( kColor == stream ) ? "color" : "depth",
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
            const std::string &frameId =
                    ( kColor == stream ) ? m_config.colorFrameId : m_config.depthFrameId;
            (void) out.frameId.Assign( frameId.c_str() ); /* length checked in Initialize() */
            out.timestampNs = TimestampNs( frame );
            out.frameNumber = static_cast<uint64_t>( frame.get_frame_number() );
            if ( kColor == stream )
            {
                m_monitor.OnColor( out.timestampNs );
            }
            else
            {
                m_monitor.OnDepth( out.timestampNs );
            }
            Deliver( out );
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
        QC_ERROR( "unexpected %s frame size %dx%d, expected %ux%u", out.encoding.CStr(), width,
                  height, out.width, out.height );
        status = QC_STATUS_FAIL;
    }
    else if ( static_cast<int>( bytesPerPixel ) != frame.get_bytes_per_pixel() )
    {
        QC_ERROR( "unexpected frame format %d for %s",
                  static_cast<int>( frame.get_profile().format() ), out.encoding.CStr() );
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

void RealSenseImpl::HandleMotion( const rs2::motion_frame &frame )
{
    const MotionSample sample{ frame.get_motion_data(), TimestampNs( frame ),
                               static_cast<uint64_t>( frame.get_frame_number() ) };
    const bool bAccel = ( RS2_STREAM_ACCEL == frame.get_profile().stream_type() );
    /* The united samples to deliver, outside m_imuLock: gyro, accel and timestamp of each. */
    std::vector<std::pair<MotionSample, rs2_vector>> united;

    {
        std::lock_guard<std::mutex> lock( m_imuLock );
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
            // timestamp, as realsense2_camera: alpha past 1 for a gyro sample stamped after this
            // accel sample, which is then extrapolated.
            if ( m_bHaveAccel )
            {
                const MotionSample &a0 = m_lastAccel;
                const double dt = static_cast<double>( sample.timestampNs ) -
                                  static_cast<double>( a0.timestampNs );
                for ( const MotionSample &gyro : m_pendingGyros )
                {
                    const double alpha = ( dt > 0.0 ) ? ( static_cast<double>( gyro.timestampNs ) -
                                                          static_cast<double>( a0.timestampNs ) ) /
                                                                dt
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
        else if ( m_bHaveAccel && ( sample.timestampNs >= m_lastAccel.timestampNs ) )
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

void RealSenseImpl::DeliverImu( const MotionSample &gyro, const rs2_vector &accel )
{
    ImuDescriptor_t imu;
    (void) imu.frameId.Assign( m_config.imuFrameId.c_str() ); /* checked in Initialize() */
    imu.timestampNs = gyro.timestampNs;
    // The gyro frame number, as the image frameNumber: a gap is a lost gyro sample.
    imu.id = gyro.frameNumber;
    // The orientation is not estimated: identity, marked unknown as in sensor_msgs/Imu.
    imu.orientation = QRB::Memory::Quaternion_t();
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

    m_monitor.OnImu( imu.timestampNs );
    Deliver( imu );
}

uint64_t RealSenseImpl::TimestampNs( const rs2::frame &frame ) const
{
    int64_t realtimeNs = 0;

    if ( RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK == frame.get_frame_timestamp_domain() )
    {
        // The camera clock has an arbitrary origin: take the arrival time instead.
        realtimeNs = ClockNs( CLOCK_REALTIME );
    }
    else
    {
        // System and global time are milliseconds of the host CLOCK_REALTIME.
        realtimeNs = static_cast<int64_t>( std::llround( frame.get_timestamp() * kNsPerMs ) );
    }

    int64_t ns = realtimeNs;
    if ( QC_REALSENSE_CLOCK_MONOTONIC == m_config.clock )
    {
        ns += ClockNs( CLOCK_MONOTONIC ) - ClockNs( CLOCK_REALTIME );
    }

    return ( ns < 0 ) ? 0u : static_cast<uint64_t>( ns );
}

void RealSenseImpl::Deliver( QCBufferDescriptorBase_t &bufDesc )
{
    NodeFrameDescriptorFlat<1> frameDesc;
    (void) frameDesc.SetBuffer( 0, bufDesc );
    QCNodeEventInfo_t eventInfo( frameDesc, m_nodeId, QC_STATUS_OK, m_state.load() );

    std::lock_guard<std::mutex> lock( m_callbackLock );
    const void *pPrevious = tl_pCallbackOwner;
    tl_pCallbackOwner = this;
    m_callback( eventInfo );
    tl_pCallbackOwner = pPrevious;
}

uint32_t RealSenseImpl::AcquireBuffer( ImageStream stream )
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

void RealSenseImpl::ReleaseBuffer( ImageStream stream, uint32_t index )
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
    else if ( nullptr != dynamic_cast<const ImuDescriptor_t *>( &bufDesc ) )
    {
        /* An IMU sample holds no buffer: nothing to give back. */
    }
    else
    {
        /* Match by memory address, so that a copy of the delivered descriptor can be released
         * too. */
        int stream = -1;
        uint32_t index = UINT32_MAX;
        for ( int s = 0; ( s < kImageStreamCount ) && ( UINT32_MAX == index ); s++ )
        {
            const std::vector<RealSenseImageDescriptor_t> &images = m_pools[s].images;
            for ( size_t i = 0; i < images.size(); i++ )
            {
                if ( ( nullptr != bufDesc.pBuf ) && ( images[i].pBuf == bufDesc.pBuf ) )
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
                QC_ERROR( "%s buffer %u released twice", ( kColor == stream ) ? "color" : "depth",
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
            QC_WARN( "no RealSense sample within %u ms", m_config.timeoutMs );
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

    if ( true == m_bStreaming )
    {
        try
        {
            // Returns once the librealsense callbacks have returned.
            m_pipeline->stop();
        }
        catch ( const std::exception &e )
        {
            QC_ERROR( "failed to stop the RealSense streaming: %s", e.what() );
        }
        m_bStreaming = false;
    }
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
