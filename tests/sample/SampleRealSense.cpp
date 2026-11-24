// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file SampleRealSense.cpp
 * @brief Sample for the RealSense QCNode, runnable with QCNodeSampleApp (-t RealSense).
 *
 * The sample allocates buffer_num DMA image buffers for each enabled image stream (rgb8 color,
 * 16UC1 depth), gives them to the RealSense node and starts the camera. For every sample delivered
 * through the node callback it:
 *  - prints it, if echo is true (like `ros2 topic echo`; one IMU sample out of 100);
 *  - measures the rate of its stream, if hz is true (like `ros2 topic hz`): once a second, and
 *    again at Stop, each enabled stream prints its configured profile, its average rate, the
 *    min/max/std dev of the intervals between the samples as they arrive, and the window (the
 *    number of intervals since Start), e.g.
 *      rs0: color 640x480 average rate: 30.002  min: 0.032s max: 0.035s std dev: 0.00045s
 *      window: 299   (on one line);
 *  - publishes it on the DataBroker topic of its stream (color_topic, depth_topic, imu_topic), if
 *    set, as a std::shared_ptr<QCBufferDescriptorBase_t> holding the descriptor, for other samples
 *    to consume. An image buffer goes back to the node when the last subscriber drops it;
 *  - otherwise gives the image buffer back to the node at once.
 *
 * Attributes:
 *   serial       : (optional) the camera serial number, default "" (first camera found)
 *   camera_name  : (optional) the prefix of the frame ids, default camera
 *   color        : (optional) enable the color stream, default true
 *   color_width, color_height, color_fps : (optional) default 640, 480, 30
 *   depth        : (optional) enable the depth stream, default true
 *   depth_width, depth_height, depth_fps : (optional) default 640, 480, 30
 *   align_depth  : (optional) align depth to color, default false
 *   imu          : (optional) enable the IMU, default false
 *   accel_fps, gyro_fps : (optional) default 0 (camera default)
 *   unite_imu_method : (optional) copy (default) or linear_interpolation
 *   clock        : (optional) realtime (default) or monotonic
 *   buffer_num   : (optional) image buffers per image stream, default 4
 *   timeout_ms   : (optional) time without any sample reported as a timeout, default 1000
 *   hz           : (optional) print the rate of each stream once a second, default false
 *   echo         : (optional) print the samples, default true, or false with hz
 *   color_topic, depth_topic, imu_topic : (optional) DataBroker topics, default "" (no publish)
 *   color_frames_topic : (optional) DataBroker topic of DataFrames_t for the QCNode samples that
 *                  take images (Recorder, TinyViz, VideoEncoder, ...): the color image as a
 *                  QC::Memory::ImageDescriptor_t RGB888 sharing the buffer, frameId the
 *                  librealsense frame number, default "" (no publish)
 *   depth_frames_topic : (optional) the same for the depth image, as a plain BufferDescriptor_t
 *                  (QCNode has no 16-bit image format), default "" (no publish)
 *   imu_file     : (optional) write every IMU sample to this CSV file, default "" (none):
 *                  index,frame_number,timestamp_ns,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
 *                  (frame_number the gyro frame number, rad/s and m/s^2)
 *   log_level    : (optional) the node log level, default ERROR
 *
 * Examples:
 *   # print color and depth frames
 *   ./bin/qcrun ./bin/QCNodeSampleApp -n rs0 -t RealSense -T 10
 *
 *   # record 300 color and depth frames to /tmp with the Recorder sample
 *   ./bin/qcrun ./bin/QCNodeSampleApp -T 15 \
 *     -n rs0 -t RealSense -k align_depth -v true -k buffer_num -v 8 \
 *       -k color_frames_topic -v /rs/color/frames -k depth_frames_topic -v /rs/depth/frames \
 *     -n RS_COLOR -t Recorder -k max -v 300 -k topic -v /rs/color/frames \
 *     -n RS_DEPTH -t Recorder -k max -v 300 -k topic -v /rs/depth/frames
 *
 *   # record the IMU samples to /tmp/rs_imu.csv
 *   ./bin/qcrun ./bin/QCNodeSampleApp -T 15 -n rs0 -t RealSense -k imu -v true \
 *     -k gyro_fps -v 200 -k accel_fps -v 200 -k imu_file -v /tmp/rs_imu.csv
 *
 *   # the rate of each stream, like `ros2 topic hz`
 *   ./bin/qcrun ./bin/QCNodeSampleApp -n rs0 -t RealSense -k imu -v true -k hz -v true
 *
 *   # color, aligned depth and IMU, published for other samples
 *   ./bin/qcrun ./bin/QCNodeSampleApp -n rs0 -t RealSense -k align_depth -v true -k imu -v true \
 *     -k color_topic -v /rs/color -k depth_topic -v /rs/depth -k imu_topic -v /rs/imu
 */

#include "QC/Node/RealSense.hpp"
#include "QC/sample/BufferManager.hpp"
#include "QC/sample/SampleIF.hpp"
#include "QRB/Memory/Sensor/ImuDescriptor.hpp"

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
     * @brief Gives image buffers back to the node. Shared with the published descriptors, which
     * a subscriber may drop after the node is gone: the release is then skipped.
     */
    struct Releaser
    {
        std::mutex lock;
        QCNodeIfs *pNode = nullptr;
        std::atomic<uint32_t> errors{ 0 };

        void Release( QCBufferDescriptorBase_t &bufDesc );
    };

    /**
     * @brief Gives one delivered image back to the node when destroyed. Shared by everything
     * published for the image, so the buffer goes back once, when the last of them is dropped.
     */
    struct ReleaseToken
    {
        ReleaseToken( std::shared_ptr<Releaser> releaserIn,
                      const RealSenseImageDescriptor_t &image )
            : releaser( std::move( releaserIn ) ),
              descriptor( image )
        {}
        ~ReleaseToken() { releaser->Release( descriptor ); }

        std::shared_ptr<Releaser> releaser;
        RealSenseImageDescriptor_t descriptor;
    };

    QCStatus_e AllocateImages( const char *encoding, uint32_t width, uint32_t height );
    void PublishFrames( DataPublisher<DataFrames_t> &pub, const RealSenseImageDescriptor_t &image,
                        const std::shared_ptr<ReleaseToken> &token );
    void OnEvent( const QCNodeEventInfo_t &eventInfo );
    void OnImage( const RealSenseImageDescriptor_t &image );
    void OnImu( const QRB::Memory::ImuDescriptor_t &imu );
    void EchoImage( const RealSenseImageDescriptor_t &image );
    void EchoImu( const QRB::Memory::ImuDescriptor_t &imu );
    void FreeBuffers();

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
    enum RateStream
    {
        kRateColor = 0,
        kRateDepth = 1,
        kRateImu = 2,
        kRateStreamCount = 3
    };
    void OnRate( RateStream stream );
    void PrintRates();

private:
    std::unique_ptr<QCNodeIfs> m_node;
    std::shared_ptr<Releaser> m_releaser;
    BufferManager *m_pBufMgr = nullptr;
    /* Sized once in Init() and never resized after the node got references to the elements. */
    std::vector<RealSenseImageDescriptor_t> m_images;

    bool m_bColor = true;
    bool m_bDepth = true;
    bool m_bImu = false;
    bool m_bEcho = true;
    bool m_bHz = false;
    uint32_t m_bufferNum = 4;
    /* Updated from the node callback, which the node serializes, and printed from it. */
    RateStats m_rates[kRateStreamCount];
    bool m_bRateEnabled[kRateStreamCount] = { false, false, false };
    std::chrono::steady_clock::time_point m_lastRatePrint;
    uint64_t m_imuCount = 0;

    DataPublisher<DescriptorRef_t> m_colorPub;
    DataPublisher<DescriptorRef_t> m_depthPub;
    DataPublisher<DescriptorRef_t> m_imuPub;
    DataPublisher<DataFrames_t> m_colorFramesPub;
    DataPublisher<DataFrames_t> m_depthFramesPub;
    bool m_bColorFramesPub = false;
    /* imu_file: written from the node callback, which the node serializes. */
    FILE *m_pImuFile = nullptr;
    bool m_bDepthFramesPub = false;
    bool m_bColorPub = false;
    bool m_bDepthPub = false;
    bool m_bImuPub = false;

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

    if ( QC_STATUS_OK == ret )
    {
        m_bColor = Get( config, "color", true );
        m_bDepth = Get( config, "depth", true );
        m_bImu = Get( config, "imu", false );
        m_bHz = Get( config, "hz", false );
        m_bEcho = Get( config, "echo", !m_bHz );
        m_bufferNum = Get( config, "buffer_num", (uint32_t) 4 );
        const bool bAlign = Get( config, "align_depth", false );
        const uint32_t colorWidth = Get( config, "color_width", (uint32_t) 640 );
        const uint32_t colorHeight = Get( config, "color_height", (uint32_t) 480 );
        const uint32_t depthWidth = Get( config, "depth_width", (uint32_t) 640 );
        const uint32_t depthHeight = Get( config, "depth_height", (uint32_t) 480 );
        const uint32_t colorFps = Get( config, "color_fps", (uint32_t) 30 );
        const uint32_t depthFps = Get( config, "depth_fps", (uint32_t) 30 );
        const uint32_t gyroFps = Get( config, "gyro_fps", (uint32_t) 0 );

        // The configured resolutions printed with the measured rates; an aligned depth image
        // has the color resolution.
        m_rates[kRateColor].label =
                "color " + std::to_string( colorWidth ) + "x" + std::to_string( colorHeight );
        m_rates[kRateDepth].label = "depth " + std::to_string( bAlign ? colorWidth : depthWidth ) +
                                    "x" + std::to_string( bAlign ? colorHeight : depthHeight );
        m_rates[kRateImu].label =
                ( 0 == gyroFps ) ? std::string( "imu" ) : "imu " + std::to_string( gyroFps ) + "Hz";
        m_bRateEnabled[kRateColor] = m_bColor;
        m_bRateEnabled[kRateDepth] = m_bDepth;
        m_bRateEnabled[kRateImu] = m_bImu;

        dt.Set<std::string>( "static.name", name );
        dt.Set<uint32_t>( "static.id", m_nodeId.id );
        dt.Set<std::string>( "static.logLevel", Get( config, "log_level", "ERROR" ) );
        dt.Set<std::string>( "static.serial", Get( config, "serial", "" ) );
        dt.Set<std::string>( "static.cameraName", Get( config, "camera_name", "camera" ) );
        dt.Set<bool>( "static.enableColor", m_bColor );
        dt.Set<uint32_t>( "static.colorWidth", colorWidth );
        dt.Set<uint32_t>( "static.colorHeight", colorHeight );
        dt.Set<uint32_t>( "static.colorFps", colorFps );
        dt.Set<bool>( "static.enableDepth", m_bDepth );
        dt.Set<uint32_t>( "static.depthWidth", depthWidth );
        dt.Set<uint32_t>( "static.depthHeight", depthHeight );
        dt.Set<uint32_t>( "static.depthFps", depthFps );
        dt.Set<bool>( "static.alignDepth", bAlign );
        dt.Set<bool>( "static.enableImu", m_bImu );
        dt.Set<uint32_t>( "static.accelFps", Get( config, "accel_fps", (uint32_t) 0 ) );
        dt.Set<uint32_t>( "static.gyroFps", gyroFps );
        dt.Set<std::string>( "static.uniteImuMethod", Get( config, "unite_imu_method", "copy" ) );
        dt.Set<std::string>( "static.clock", Get( config, "clock", "realtime" ) );
        dt.Set<uint32_t>( "static.timeoutMs", Get( config, "timeout_ms", (uint32_t) 1000 ) );

        if ( ( 0 == m_bufferNum ) || ( m_bufferNum > QC_REALSENSE_MAX_BUFFERS ) )
        {
            QC_ERROR( "buffer_num must be 1 to %u", QC_REALSENSE_MAX_BUFFERS );
            ret = QC_STATUS_BAD_ARGUMENTS;
        }

        const std::string colorTopic = Get( config, "color_topic", "" );
        const std::string depthTopic = Get( config, "depth_topic", "" );
        const std::string imuTopic = Get( config, "imu_topic", "" );
        if ( ( QC_STATUS_OK == ret ) && ( false == colorTopic.empty() ) )
        {
            ret = m_colorPub.Init( name, colorTopic );
            m_bColorPub = ( QC_STATUS_OK == ret );
        }
        if ( ( QC_STATUS_OK == ret ) && ( false == depthTopic.empty() ) )
        {
            ret = m_depthPub.Init( name, depthTopic );
            m_bDepthPub = ( QC_STATUS_OK == ret );
        }
        if ( ( QC_STATUS_OK == ret ) && ( false == imuTopic.empty() ) )
        {
            ret = m_imuPub.Init( name, imuTopic );
            m_bImuPub = ( QC_STATUS_OK == ret );
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
                                     "accel_x,accel_y,accel_z\n" );
            }
        }
        const std::string colorFramesTopic = Get( config, "color_frames_topic", "" );
        const std::string depthFramesTopic = Get( config, "depth_frames_topic", "" );
        if ( ( QC_STATUS_OK == ret ) && ( false == colorFramesTopic.empty() ) )
        {
            ret = m_colorFramesPub.Init( name, colorFramesTopic );
            m_bColorFramesPub = ( QC_STATUS_OK == ret );
        }
        if ( ( QC_STATUS_OK == ret ) && ( false == depthFramesTopic.empty() ) )
        {
            ret = m_depthFramesPub.Init( name, depthFramesTopic );
            m_bDepthFramesPub = ( QC_STATUS_OK == ret );
        }

        /* The DMA buffers the node copies the images into. The vector is sized once here and
         * never resized, as the node keeps references to its elements. */
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
            m_images.reserve( 2 * m_bufferNum );
            if ( m_bColor )
            {
                ret = AllocateImages( QC_REALSENSE_ENCODING_RGB8, colorWidth, colorHeight );
            }
        }
        if ( ( QC_STATUS_OK == ret ) && m_bDepth )
        {
            // An aligned depth image has the color resolution.
            ret = AllocateImages( QC_REALSENSE_ENCODING_16UC1, bAlign ? colorWidth : depthWidth,
                                  bAlign ? colorHeight : depthHeight );
        }
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
    }
    else
    {
        m_node.reset();
        FreeBuffers();
        if ( nullptr != m_pImuFile )
        {
            (void) fclose( m_pImuFile );
            m_pImuFile = nullptr;
        }
    }

    return ret;
}

QCStatus_e SampleRealSense::AllocateImages( const char *encoding, uint32_t width, uint32_t height )
{
    QCStatus_e ret = QC_STATUS_OK;
    const uint32_t step = width * RealSenseEncodingBytesPerPixel( encoding );

    for ( uint32_t i = 0; ( i < m_bufferNum ) && ( QC_STATUS_OK == ret ); i++ )
    {
        m_images.emplace_back();
        RealSenseImageDescriptor_t &image = m_images.back();
        ret = m_pBufMgr->Allocate(
                BufferProps_t( static_cast<size_t>( step ) * height, QC_MEMORY_ALLOCATOR_DMA ),
                image );
        if ( QC_STATUS_OK != ret )
        {
            QC_ERROR( "failed to allocate %s buffer %u: %d", encoding, i, ret );
            m_images.pop_back();
        }
        else
        {
            image.offset = 0;
            image.validSize = 0;
            image.width = width;
            image.height = height;
            image.step = step;
            (void) image.encoding.Assign( encoding );
        }
    }

    return ret;
}

QCStatus_e SampleRealSense::Start()
{
    m_imuCount = 0;
    for ( RateStats &rate : m_rates )
    {
        rate.Reset();
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
    const auto *pImage = dynamic_cast<const RealSenseImageDescriptor_t *>( &bufDesc );
    const auto *pImu = dynamic_cast<const QRB::Memory::ImuDescriptor_t *>( &bufDesc );

    if ( QC_STATUS_OK != eventInfo.status )
    {
        QC_ERROR( "RealSense event error: %d", eventInfo.status );
    }
    else if ( nullptr != pImage )
    {
        OnImage( *pImage );
    }
    else if ( nullptr != pImu )
    {
        OnImu( *pImu );
    }
    else
    {
        QC_ERROR( "unexpected RealSense event buffer" );
    }
}

void SampleRealSense::OnImage( const RealSenseImageDescriptor_t &image )
{
    const bool bColor = ( image.encoding == QC_REALSENSE_ENCODING_RGB8 );
    DataPublisher<DescriptorRef_t> &pub = bColor ? m_colorPub : m_depthPub;
    const bool bPublish = bColor ? m_bColorPub : m_bDepthPub;
    DataPublisher<DataFrames_t> &framesPub = bColor ? m_colorFramesPub : m_depthFramesPub;
    const bool bPublishFrames = bColor ? m_bColorFramesPub : m_bDepthFramesPub;

    if ( m_bHz )
    {
        OnRate( bColor ? kRateColor : kRateDepth );
    }
    if ( m_bEcho )
    {
        EchoImage( image );
    }

    if ( bPublish || bPublishFrames )
    {
        /* Everything published points to the same buffer, which goes back to the node when the
         * last subscriber drops what it holds. */
        auto token = std::make_shared<ReleaseToken>( m_releaser, image );
        if ( bPublish )
        {
            // The deleter holds the token: the image goes back when the descriptor is dropped.
            DescriptorRef_t descriptor( new RealSenseImageDescriptor_t( image ),
                                        [token]( QCBufferDescriptorBase_t *pDesc ) {
                                            (void) token;
                                            delete pDesc;
                                        } );
            (void) pub.Publish( descriptor );
        }
        if ( bPublishFrames )
        {
            PublishFrames( framesPub, image, token );
        }
    }
    else
    {
        RealSenseImageDescriptor_t copy( image );
        m_releaser->Release( copy );
    }
}

void SampleRealSense::PublishFrames( DataPublisher<DataFrames_t> &pub,
                                     const RealSenseImageDescriptor_t &image,
                                     const std::shared_ptr<ReleaseToken> &token )
{
    SharedBuffer_t *pSharedBuffer = new SharedBuffer_t;

    if ( image.encoding == QC_REALSENSE_ENCODING_RGB8 )
    {
        // The color image as the QCNode RGB888 image the image samples take, on the same buffer.
        ImageDescriptor_t qcImage;
        qcImage = static_cast<const BufferDescriptor_t &>( image );
        qcImage.format = QC_IMAGE_FORMAT_RGB888;
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
        // QCNode has no 16-bit image format: the depth pixels as a plain buffer.
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

void SampleRealSense::OnImu( const QRB::Memory::ImuDescriptor_t &imu )
{
    m_imuCount++;
    if ( m_bHz )
    {
        OnRate( kRateImu );
    }
    if ( m_bEcho && ( 1 == ( m_imuCount % kImuEchoEvery ) ) )
    {
        EchoImu( imu );
    }

    if ( nullptr != m_pImuFile )
    {
        fprintf( m_pImuFile, "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                 m_imuCount, imu.id, imu.timestampNs, imu.angular_velocity.x,
                 imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x,
                 imu.linear_acceleration.y, imu.linear_acceleration.z );
    }

    if ( m_bImuPub )
    {
        // The delivered sample is valid during the callback only: publish a copy.
        DescriptorRef_t descriptor = std::make_shared<QRB::Memory::ImuDescriptor_t>( imu );
        (void) m_imuPub.Publish( descriptor );
    }
}

void SampleRealSense::EchoImage( const RealSenseImageDescriptor_t &image )
{
    /* Print the center pixel as a quick sanity check of the image content. */
    const uint8_t *pCenter = image.GetRowPtr( image.height / 2 ) +
                             static_cast<size_t>( image.width / 2 ) *
                                     RealSenseEncodingBytesPerPixel( image.encoding.CStr() );
    const uint64_t nsPerSec = 1000000000u;

    if ( image.encoding == QC_REALSENSE_ENCODING_RGB8 )
    {
        printf( "%s: color stamp %" PRIu64 ".%09" PRIu64 " %s %ux%u rgb8 step %u, center pixel "
                "(%u, %u, %u)\n",
                m_name.c_str(), image.timestampNs / nsPerSec, image.timestampNs % nsPerSec,
                image.frameId.CStr(), image.width, image.height, image.step,
                static_cast<uint32_t>( pCenter[0] ), static_cast<uint32_t>( pCenter[1] ),
                static_cast<uint32_t>( pCenter[2] ) );
    }
    else
    {
        uint16_t depth = 0;
        (void) memcpy( &depth, pCenter, sizeof( depth ) );
        printf( "%s: depth stamp %" PRIu64 ".%09" PRIu64 " %s %ux%u 16UC1 step %u, center depth "
                "%u\n",
                m_name.c_str(), image.timestampNs / nsPerSec, image.timestampNs % nsPerSec,
                image.frameId.CStr(), image.width, image.height, image.step,
                static_cast<uint32_t>( depth ) );
    }
}

void SampleRealSense::EchoImu( const QRB::Memory::ImuDescriptor_t &imu )
{
    const uint64_t nsPerSec = 1000000000u;
    printf( "%s: imu stamp %" PRIu64 ".%09" PRIu64 " %s angular_velocity (%.4f, %.4f, %.4f) "
            "linear_acceleration (%.4f, %.4f, %.4f), sample %" PRIu64 "\n",
            m_name.c_str(), imu.timestampNs / nsPerSec, imu.timestampNs % nsPerSec,
            imu.frameId.CStr(), imu.angular_velocity.x, imu.angular_velocity.y,
            imu.angular_velocity.z, imu.linear_acceleration.x, imu.linear_acceleration.y,
            imu.linear_acceleration.z, m_imuCount );
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

void SampleRealSense::OnRate( RateStream stream )
{
    const auto now = std::chrono::steady_clock::now();
    m_rates[stream].Add( now );
    if ( ( now - m_lastRatePrint ) >= std::chrono::seconds( 1 ) )
    {
        m_lastRatePrint = now;
        PrintRates();
    }
}

void SampleRealSense::PrintRates()
{
    for ( int s = 0; s < kRateStreamCount; s++ )
    {
        if ( m_bRateEnabled[s] )
        {
            m_rates[s].Print( m_name );
        }
    }
    fflush( stdout );
}

QCStatus_e SampleRealSense::Stop()
{
    QCStatus_e ret = m_node->Stop();

    // Drop the descriptors still queued for the subscribers, giving their buffers back.
    m_colorPub.Clear();
    m_depthPub.Clear();
    m_imuPub.Clear();
    m_colorFramesPub.Clear();
    m_depthFramesPub.Clear();
    if ( nullptr != m_pImuFile )
    {
        // The node is stopped: no callback writes any more.
        fflush( m_pImuFile );
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
            printf( "%s: color %" PRIu64 " (dropped %" PRIu64 "), depth %" PRIu64
                    " (dropped %" PRIu64 "), imu %" PRIu64 ", timeouts %" PRIu64 ", errors %" PRIu64
                    ", release errors %u\n",
                    m_name.c_str(), monitor.colorFrameCount, monitor.colorDropCount,
                    monitor.depthFrameCount, monitor.depthDropCount, monitor.imuSampleCount,
                    monitor.timeoutCount, monitor.errorCount, m_releaser->errors.load() );
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
    if ( nullptr != m_pImuFile )
    {
        (void) fclose( m_pImuFile );
        m_pImuFile = nullptr;
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
        BufferManager::Put( m_pBufMgr );
        m_pBufMgr = nullptr;
    }
    m_images.clear();
}

REGISTER_SAMPLE( RealSense, SampleRealSense );

}   // namespace sample
}   // namespace QC
