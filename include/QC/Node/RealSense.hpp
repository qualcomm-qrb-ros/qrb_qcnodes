// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file RealSense.hpp
 * @brief The Intel RealSense depth camera QCNode (color, depth and IMU streams).
 *
 * The RealSense node is a source node, the QCNode counterpart of the realsense2_camera ROS node.
 * It opens a RealSense camera (e.g. D455) through librealsense2 and delivers, through the
 * QCNodeInit_t::callback, one event per sample of each enabled stream, with the sample at index 0
 * of the frame descriptor:
 *
 * | Stream | Descriptor at index 0                      | realsense2_camera topic        |
 * |--------|--------------------------------------------|--------------------------------|
 * | color  | RealSenseImageDescriptor_t, encoding rgb8  | /camera/camera/color/image_raw |
 * | depth  | RealSenseImageDescriptor_t, encoding 16UC1 | (1)                            |
 * | IMU    | QRB::Memory::ImuDescriptor_t (qrb_contracts) | /camera/camera/imu           |
 *
 * (1) /camera/camera/depth/image_rect_raw; with alignDepth,
 *     /camera/camera/aligned_depth_to_color/image_raw
 *
 * RealSenseImageDescriptor_t mirrors sensor_msgs/Image (encoding, step, stamp, frame id); unlike
 * QC::Memory::ImageDescriptor_t it can carry the 16-bit depth images. The callback tells the
 * streams apart by the descriptor type and the image encoding. Depth pixels are the raw Z16
 * values: millimetres for the D400 depth unit of 0.001 m.
 *
 * Image buffers (QCNodeInit_t::buffers):
 *   For each enabled image stream, 1 to QC_REALSENSE_MAX_BUFFERS RealSenseImageDescriptor_t,
 *   with pBuf, size and offset set by the allocator, and set by the application before
 *   Initialize():
 *     - encoding: "rgb8" for a color buffer, "16UC1" for a depth buffer;
 *     - width and height: the resolution of the stream (the color one for an aligned depth);
 *     - step: the row stride in bytes, at least width times 3 (rgb8) or 2 (16UC1).
 *   The node copies each sample into a free buffer of its stream; a delivered buffer is the
 *   application's until it gives it back by calling ProcessFrameDescriptor() with the buffer at
 *   index 0. A sample that finds every buffer of its stream held is dropped, like a full ROS queue.
 *
 * IMU samples need no buffer: the delivered ImuDescriptor_t is valid during the callback only,
 * and the application copies it if it keeps it. One sample is delivered per gyro sample, united
 * with the accel stream as realsense2_camera's unite_imu_method (uniteImuMethod):
 *   - "copy": with the latest accel sample, delivered at once;
 *   - "linear_interpolation": with the accel linearly interpolated at the gyro timestamp between
 *     the accel samples before and after it, so delivered when the next accel sample arrives.
 * The orientation is unknown. The id of the ImuDescriptor_t (a BufferDescriptor_t field) is the
 * librealsense frame number of the gyro sample: a gap is a lost gyro sample.
 *
 * Threading: librealsense calls the node from its own threads. The node delivers one event at a
 * time, so the callback needs no lock of its own, but it should return quickly: it holds back
 * the other streams. The callback may call ProcessFrameDescriptor(), but not Stop().
 *
 * Example static configuration (every field but name and id is optional):
 * @code
 * {
 *   "static": {
 *     "name": "rs0",
 *     "id": 0,
 *     "serial": "",              // camera serial number, empty means the first camera found
 *     "cameraName": "camera",    // prefix of the default frame ids
 *     "enableColor": true,
 *     "colorWidth": 640, "colorHeight": 480, "colorFps": 30,
 *     "enableDepth": true,
 *     "depthWidth": 640, "depthHeight": 480, "depthFps": 30,
 *     "alignDepth": false,       // align depth to color: depth then has the color resolution
 *     "enableImu": false,
 *     "accelFps": 0, "gyroFps": 0,  // 0: the camera default
 *     "uniteImuMethod": "copy",  // "copy" or "linear_interpolation"
 *     "linearAccelCov": 0.01, "angularVelocityCov": 0.01,
 *     "clock": "realtime",       // "realtime" (ROS time, as realsense2_camera) or "monotonic"
 *     "colorFrameId": "",        // default <cameraName>_color_optical_frame
 *     "depthFrameId": "",        // default <cameraName>_depth_optical_frame, or the color one
 *                                // with alignDepth
 *     "imuFrameId": "",          // default <cameraName>_imu_optical_frame
 *     "timeoutMs": 1000          // report a timeout after this long without any sample
 *   }
 * }
 * @endcode
 */

#ifndef QRB_QC_NODE_REALSENSE_HPP
#define QRB_QC_NODE_REALSENSE_HPP

#include "QC/Common/QCString.hpp"
#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QC/Node/NodeBase.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

namespace QC
{
namespace Node
{

/** @brief The node type used to register the RealSense node with QCNodeFactory. */
#define QC_NODE_TYPE_REALSENSE QC_NODE_TYPE_CUSTOM_1

/** @brief The maximum number of buffers of one image stream. */
#define QC_REALSENSE_MAX_BUFFERS 16u

/** @brief The maximum number of characters of an image encoding name. */
#define QC_REALSENSE_ENCODING_MAX_LEN 31u

/** @brief The sensor_msgs/image_encodings name of the color images. */
static constexpr const char *QC_REALSENSE_ENCODING_RGB8 = "rgb8";
/** @brief The sensor_msgs/image_encodings name of the depth images. */
static constexpr const char *QC_REALSENSE_ENCODING_16UC1 = "16UC1";

/** @brief The bytes per pixel of a RealSense image encoding, 0 for another encoding. */
inline uint32_t RealSenseEncodingBytesPerPixel( const char *encoding )
{
    uint32_t bytes = 0u;
    if ( nullptr != encoding )
    {
        if ( 0 == std::strcmp( encoding, QC_REALSENSE_ENCODING_RGB8 ) )
        {
            bytes = 3u;
        }
        else if ( 0 == std::strcmp( encoding, QC_REALSENSE_ENCODING_16UC1 ) )
        {
            bytes = 2u;
        }
        else
        {
            /* not a RealSense image encoding */
        }
    }
    return bytes;
}

/**
 * @brief A RealSense color or depth image: one single-plane image in the DMA buffer, mirroring
 * sensor_msgs/Image.
 *
 * Inherited members of QC::Memory::BufferDescriptor_t: pBuf, size, offset (of the first row),
 * validSize (step * height once filled), dmaHandle, allocatorType, ...
 */
typedef struct RealSenseImageDescriptor : public BufferDescriptor_t
{
public:
    RealSenseImageDescriptor() : BufferDescriptor_t() {}
    ~RealSenseImageDescriptor() override = default;

    RealSenseImageDescriptor( const RealSenseImageDescriptor &other ) = default;
    RealSenseImageDescriptor( RealSenseImageDescriptor &&other ) = default;

    using BufferDescriptor_t::operator=;

    /** @brief Copy the base fields only, the image fields are left unchanged. */
    RealSenseImageDescriptor &operator=( const BufferDescriptor_t &other ) noexcept override
    {
        BufferDescriptor_t::operator=( other );
        return *this;
    }

    RealSenseImageDescriptor &operator=( const RealSenseImageDescriptor &other ) noexcept
    {
        if ( this != &other )
        {
            BufferDescriptor_t::operator=( other );
            CopyImageFields( other );
        }
        return *this;
    }

    RealSenseImageDescriptor &operator=( RealSenseImageDescriptor &&other ) noexcept
    {
        return operator=( static_cast<const RealSenseImageDescriptor &>( other ) );
    }

    /** @brief Dispatch to the most derived overload @p other is an instance of. */
    RealSenseImageDescriptor &operator=( const QCBufferDescriptorBase_t &other ) noexcept override
    {
        if ( this != &other )
        {
            const auto *pImage = dynamic_cast<const RealSenseImageDescriptor *>( &other );
            const auto *pBuffer = dynamic_cast<const BufferDescriptor_t *>( &other );
            if ( nullptr != pImage )
            {
                operator=( *pImage );
            }
            else if ( nullptr != pBuffer )
            {
                operator=( *pBuffer );
            }
            else
            {
                BufferDescriptor_t::operator=( other );
            }
        }
        return *this;
    }

    /** @brief The bytes of the image layout, step * height. */
    size_t GetImageSize() const { return static_cast<size_t>( step ) * height; }

    /** @brief The first byte of row @p row, no bounds check; nullptr without memory. */
    uint8_t *GetRowPtr( uint32_t row ) const
    {
        return ( nullptr == pBuf )
                       ? nullptr
                       : static_cast<uint8_t *>( GetDataPtr() ) + static_cast<size_t>( row ) * step;
    }

    uint32_t height = 0;                              /**< Rows */
    uint32_t width = 0;                               /**< Pixels per row */
    QCString<QC_REALSENSE_ENCODING_MAX_LEN> encoding; /**< "rgb8" or "16UC1" */
    bool isBigEndian = false;
    uint32_t step = 0;              /**< Bytes per row, >= width * bytes per pixel */
    uint64_t timestampNs = 0;       /**< Acquisition time, see RealSenseConfig_t::clock */
    uint64_t frameNumber = 0;       /**< librealsense frame counter, a gap is a lost frame */
    QRB::Memory::FrameId_t frameId; /**< e.g. camera_color_optical_frame */

private:
    void CopyImageFields( const RealSenseImageDescriptor &other )
    {
        height = other.height;
        width = other.width;
        encoding = other.encoding;
        isBigEndian = other.isBigEndian;
        step = other.step;
        timestampNs = other.timestampNs;
        frameNumber = other.frameNumber;
        frameId = other.frameId;
    }
} RealSenseImageDescriptor_t;

/** @brief The clock of the delivered timestamps. */
typedef enum
{
    QC_REALSENSE_CLOCK_REALTIME = 0, /**< CLOCK_REALTIME, the ROS time of realsense2_camera */
    QC_REALSENSE_CLOCK_MONOTONIC     /**< CLOCK_MONOTONIC */
} QCRealSenseClock_e;

/** @brief How the accel samples are united with the gyro samples, as realsense2_camera. */
typedef enum
{
    QC_REALSENSE_UNITE_IMU_COPY = 0,            /**< The latest accel sample */
    QC_REALSENSE_UNITE_IMU_LINEAR_INTERPOLATION /**< Accel interpolated at the gyro timestamp */
} QCRealSenseUniteImu_e;

/** @brief The RealSense node configuration. */
typedef struct RealSenseConfig_s : public QCNodeConfigBase_t
{
    std::string serial;      /**< Camera serial number, empty means the first camera found */
    bool enableColor = true; /**< Deliver the rgb8 color stream */
    uint32_t colorWidth = 640;
    uint32_t colorHeight = 480;
    uint32_t colorFps = 30;
    bool enableDepth = true; /**< Deliver the 16UC1 depth stream */
    uint32_t depthWidth = 640;
    uint32_t depthHeight = 480;
    uint32_t depthFps = 30;
    bool alignDepth = false; /**< Align depth to color, needs both streams */
    bool enableImu = false;  /**< Deliver the IMU samples, needs a camera with an IMU */
    uint32_t accelFps = 0;   /**< Accel rate, 0 for the camera default */
    uint32_t gyroFps = 0;    /**< Gyro rate, 0 for the camera default */
    QCRealSenseUniteImu_e uniteImuMethod = QC_REALSENSE_UNITE_IMU_COPY;
    double linearAccelCov = 0.01;     /**< Diagonal of the linear acceleration covariance */
    double angularVelocityCov = 0.01; /**< Diagonal of the angular velocity covariance */
    QCRealSenseClock_e clock = QC_REALSENSE_CLOCK_REALTIME;
    std::string colorFrameId;  /**< Frame id of the color images */
    std::string depthFrameId;  /**< Frame id of the depth images */
    std::string imuFrameId;    /**< Frame id of the IMU samples */
    uint32_t timeoutMs = 1000; /**< Time without any sample reported as a timeout */
} RealSenseConfig_t;

/** @brief The RealSense node monitoring data. */
typedef struct RealSenseMonitor_s : public QCNodeMonitoringBase_t
{
    uint64_t colorFrameCount; /**< Color images delivered */
    uint64_t depthFrameCount; /**< Depth images delivered */
    uint64_t imuSampleCount;  /**< IMU samples delivered */
    uint64_t colorDropCount;  /**< Color images dropped as every color buffer was held */
    uint64_t depthDropCount;  /**< Depth images dropped as every depth buffer was held */
    uint64_t timeoutCount;    /**< Periods of timeoutMs without any sample */
    uint64_t errorCount;      /**< librealsense or copy errors */
    uint64_t lastTimestampNs; /**< Timestamp of the last delivered sample */
} RealSenseMonitor_t;

class RealSenseConfig final : public NodeConfigBase
{
public:
    explicit RealSenseConfig( Logger &logger );
    ~RealSenseConfig() = default;

    QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    QCStatus_e GetOptions( std::string &options ) override;
    const QCNodeConfigBase_t &Get() override;

    /** @brief Get the RealSense specific configuration. */
    const RealSenseConfig_t &GetConfig() const { return m_config; }

private:
    RealSenseConfig_t m_config;
    std::string m_options;
};

class RealSenseMonitoring final : public QCNodeMonitoringIfs
{
public:
    explicit RealSenseMonitoring( Logger &logger );
    ~RealSenseMonitoring() = default;

    QCStatus_e VerifyAndSet( const std::string config, std::string &errors ) override;
    QCStatus_e GetOptions( std::string &options ) override;
    const QCNodeMonitoringBase_t &Get() override;
    uint32_t GetMaximalSize() override;
    uint32_t GetCurrentSize() override;
    QCStatus_e Place( void *ptr, uint32_t &size ) override;

    /* The counters are updated by the librealsense threads, so they are atomic. */
    void OnColor( uint64_t timestampNs );
    void OnDepth( uint64_t timestampNs );
    void OnImu( uint64_t timestampNs );
    void IncColorDrop() { m_colorDropCount++; }
    void IncDepthDrop() { m_depthDropCount++; }
    void IncTimeout() { m_timeoutCount++; }
    void IncError() { m_errorCount++; }
    void Reset();

private:
    void Snapshot();

    RealSenseMonitor_t m_data;
    std::atomic<uint64_t> m_colorFrameCount{ 0 };
    std::atomic<uint64_t> m_depthFrameCount{ 0 };
    std::atomic<uint64_t> m_imuSampleCount{ 0 };
    std::atomic<uint64_t> m_colorDropCount{ 0 };
    std::atomic<uint64_t> m_depthDropCount{ 0 };
    std::atomic<uint64_t> m_timeoutCount{ 0 };
    std::atomic<uint64_t> m_errorCount{ 0 };
    std::atomic<uint64_t> m_lastTimestampNs{ 0 };
    Logger &m_logger;
};

/* Private implementation, hides librealsense2 from the users of this header. */
class RealSenseImpl;

class RealSense final : public NodeBase
{
public:
    RealSense();
    ~RealSense();

    RealSense( const RealSense & ) = delete;
    RealSense &operator=( const RealSense & ) = delete;
    RealSense( RealSense && ) = delete;
    RealSense &operator=( RealSense && ) = delete;

    /**
     * @brief Initializes the node: parses the configuration, checks the buffers and opens the
     * camera.
     * @param[in] config The JSON configuration, the event callback (mandatory) and the image
     * buffers (see the file description).
     */
    QCStatus_e Initialize( QCNodeInit_t &config ) override;
    QCNodeConfigIfs &GetConfigurationIfs() override;
    QCNodeMonitoringIfs &GetMonitoringIfs() override;

    /** @brief Starts the camera streaming, samples are then delivered through the callback. */
    QCStatus_e Start() override;

    /**
     * @brief Gives an image buffer delivered through the callback back to the node.
     * @param[in] frameDesc The frame descriptor with the image to release at index 0.
     */
    QCStatus_e ProcessFrameDescriptor( QCFrameDescriptorNodeIfs &frameDesc ) override;

    /** @brief Stops the camera streaming. Must not be called from the callback. */
    QCStatus_e Stop() override;
    QCStatus_e DeInitialize() override;
    QCObjectState_e GetState() override;

private:
    RealSenseConfig m_configIfs;
    RealSenseMonitoring m_monitorIfs;
    std::unique_ptr<RealSenseImpl> m_pImpl;
};

}   // namespace Node
}   // namespace QC

#endif   // QRB_QC_NODE_REALSENSE_HPP
