// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

/**
 * @file RealSense.hpp
 * @brief The Intel RealSense depth camera QCNode, the QCNode counterpart of the realsense2_camera
 * ROS 2 node (D455, D405).
 *
 * The RealSense node is a source node. It opens one RealSense camera through librealsense2, by
 * serial number, and delivers, through the QCNodeInit_t::callback, one event per sample of each
 * enabled stream:
 *
 * | Stream (QCRealSenseStream_e) | Index 0 of the event         | realsense2_camera topic (1)   |
 * |------------------------------|------------------------------|-------------------------------|
 * | COLOR                        | RealSenseImageDescriptor_t   | color/image_raw               |
 * | DEPTH                        | RealSenseImageDescriptor_t   | depth/image_rect_raw          |
 * | ALIGNED_DEPTH_TO_COLOR       | RealSenseImageDescriptor_t   | aligned_depth_to_color/image_raw
 * | | INFRA1, INFRA2               | RealSenseImageDescriptor_t   | infra1/image_rect_raw,
 * infra2/... | | ACCEL, GYRO                  | QRB::Memory::ImuDescriptor_t | accel/sample,
 * gyro/sample     | | IMU (accel + gyro united)    | QRB::Memory::ImuDescriptor_t | imu | |
 * POINTCLOUD                   | QRB::Memory::PointCloudDescriptor_t | depth/color/points     |
 *
 * (1) under /<camera_namespace>/<camera_name>/
 *
 * Index 1 of every event is a RealSenseFrameInfo_t: the stream, the camera identity (serial
 * number and logical camera name), the librealsense frame number, the original sensor timestamp
 * with its timestamp domain, the metadata sensor timestamp, the host receive time, the depth
 * scale and, for the image and point cloud streams, the CameraInfo of the stream.
 *
 * The static information, which realsense2_camera publishes on latched topics, is queried from
 * the node once it is initialized (cast the QCNodeIfs to RealSense):
 *   - GetDeviceInfo(): model, serial, firmware, USB type and port, hardware sync role;
 *   - GetCameraInfo(): sensor_msgs/CameraInfo of a stream (K, D, R, P, distortion model);
 *   - GetExtrinsics(): realsense2_camera_msgs/Extrinsics between two streams;
 *   - GetImuInfo(): realsense2_camera_msgs/IMUInfo of the accel or gyro;
 *   - GetStaticTransforms(): the /tf_static transforms of realsense2_camera (camera_link, the
 *     <stream>_frame and <stream>_optical_frame of each enabled stream), as
 *     QRB::Memory::TransformStamped_t or into a QRB::Memory::TFMessageDescriptor_t;
 *   - EnumerateDevices(): the connected cameras, without creating a node.
 *
 * Image buffers (QCNodeInit_t::buffers):
 *   For each enabled image stream, 1 to QC_REALSENSE_MAX_BUFFERS RealSenseImageDescriptor_t with
 *   pBuf, size and offset set by the allocator, and set by the application before Initialize():
 *     - stream: the stream the buffer is for (COLOR, DEPTH, ALIGNED_DEPTH_TO_COLOR, INFRA1,
 * INFRA2);
 *     - encoding: the encoding of that stream (colorEncoding, "16UC1", infraEncoding);
 *     - width and height: the resolution of the stream (the color one for the aligned depth);
 *     - step: the row stride in bytes, at least width times the bytes per pixel.
 *   With enablePointcloud, 1 to QC_REALSENSE_MAX_BUFFERS QRB::Memory::PointCloudDescriptor_t
 *   of at least depthWidth * depthHeight * RealSensePointStep(texture) bytes.
 *   The node copies each sample into a free buffer of its stream; a delivered buffer is the
 *   application's until it gives it back by calling ProcessFrameDescriptor() with the buffer at
 *   index 0. A sample that finds every buffer of its stream held is dropped and counted, like a
 *   full ROS queue, and never blocks the camera threads.
 *
 * IMU samples need no buffer: the delivered ImuDescriptor_t is valid during the callback only.
 * uniteImuMethod, as realsense2_camera's unite_imu_method, selects the IMU events:
 *   - "none": separate ACCEL and GYRO events (the other vector is zero);
 *   - "copy": one IMU event per gyro sample, with the latest accel sample;
 *   - "linear_interpolation": one IMU event per gyro sample, with the accel linearly
 *     interpolated at the gyro timestamp, so delivered when the next accel sample arrives.
 *   With copy or linear_interpolation, rawImu also delivers the separate ACCEL and GYRO events,
 *   as realsense2_camera always publishes accel/sample and gyro/sample. The orientation is
 *   unknown (orientation_covariance[0] = -1). The id of the ImuDescriptor_t is the librealsense
 *   frame number of the sample (the gyro one for IMU): a gap is a lost sample.
 *
 * Timestamps (as realsense2_camera): timestampNs is the frame timestamp of librealsense, in the
 * system or global time domain host time, and in the hardware clock domain the camera clock
 * mapped to the host time at the first frame (re-mapped when the camera clock resets). The
 * original sensor timestamp, its domain and the host receive time stay available in
 * RealSenseFrameInfo_t.
 *
 * Synchronization:
 *   - enableSync (realsense2_camera enable_sync) groups the frames of the streams of this camera
 *     into framesets; it is on when alignDepth or enablePointcloud is set, as both need a depth
 *     frame and the matching color frame.
 *   - interCamSyncMode (realsense2_camera depth_module.inter_cam_sync_mode) is the multi-camera
 *     hardware synchronization role, set before streaming: 0 default, 1 master, 2 slave (any
 *     value the camera supports, e.g. 3 full slave). -1 leaves the camera setting unchanged.
 *     The nodes of one process with the same syncGroup may have only one master.
 *
 * Threading: librealsense calls the node from its own threads (one per sensor, and one for the
 * synchronized framesets). The node delivers one event at a time, so the callback needs no lock
 * of its own, but it should return quickly: it holds back the other streams. The callback may
 * call ProcessFrameDescriptor(), but not Stop().
 *
 * Example static configuration (every field but name and id is optional):
 * @code
 * {
 *   "static": {
 *     "name": "rs0", "id": 0,
 *     "serial": "",               // camera serial number, empty means the first camera found
 *     "cameraName": "camera",     // logical camera name, prefix of the frame ids (e.g. cam1)
 *     "baseFrameId": "",          // default <cameraName>_link
 *     "enableColor": true, "colorWidth": 640, "colorHeight": 480, "colorFps": 30,
 *     "colorEncoding": "rgb8",    // rgb8, bgr8, rgba8, bgra8 or yuv422_yuy2
 *     "enableDepth": true, "depthWidth": 640, "depthHeight": 480, "depthFps": 30,
 *     "enableInfra1": false, "enableInfra2": false,
 *     "infraWidth": 640, "infraHeight": 480, "infraFps": 30,
 *     "infraEncoding": "mono8",   // mono8 or mono16
 *     "alignDepth": false,        // also deliver the depth aligned to color
 *     "enablePointcloud": false,
 *     "pointcloudTexture": "color", // color (xyz + rgb) or none (xyz)
 *     "orderedPointcloud": false, // keep the depth image layout, invalid points included
 *     "allowNoTexturePoints": false,
 *     "enableSync": false,
 *     "enableAccel": false, "enableGyro": false,
 *     "enableImu": false,         // shorthand for enableAccel and enableGyro
 *     "accelFps": 0, "gyroFps": 0, // 0: the camera default
 *     "uniteImuMethod": "copy",   // none, copy or linear_interpolation
 *     "rawImu": false,
 *     "linearAccelCov": 0.01, "angularVelocityCov": 0.01,
 *     "interCamSyncMode": -1,     // -1 unchanged, 0 default, 1 master, 2 slave, ...
 *     "syncGroup": "default",
 *     "sensorOptions": "",        // e.g.
 * "depth_module.exposure=8500,rgb_camera.enable_auto_exposure=0" "clock": "realtime",        //
 * realtime (ROS time) or monotonic "timeoutMs": 1000           // report a timeout after this long
 * without any sample
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
#include "QRB/Memory/Geometry/TFMessageDescriptor.hpp"
#include "QRB/Memory/Sensor/ImuDescriptor.hpp"
#include "QRB/Memory/Sensor/PointCloudDescriptor.hpp"

#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace QC
{
namespace Node
{

/** @brief The node type used to register the RealSense node with QCNodeFactory. */
#define QC_NODE_TYPE_REALSENSE QC_NODE_TYPE_CUSTOM_1

/** @brief The maximum number of buffers of one stream. */
#define QC_REALSENSE_MAX_BUFFERS 16u

/** @brief The maximum number of characters of an image encoding or a distortion model name. */
#define QC_REALSENSE_NAME_MAX_LEN 31u

/** @brief The point cloud point sizes: x, y, z float32, and rgb packed in a float32. */
#define QC_REALSENSE_POINT_STEP_XYZ 12u
#define QC_REALSENSE_POINT_STEP_XYZRGB 16u

/** @brief The sensor_msgs/image_encodings names of the RealSense images. */
static constexpr const char *QC_REALSENSE_ENCODING_RGB8 = "rgb8";
static constexpr const char *QC_REALSENSE_ENCODING_BGR8 = "bgr8";
static constexpr const char *QC_REALSENSE_ENCODING_RGBA8 = "rgba8";
static constexpr const char *QC_REALSENSE_ENCODING_BGRA8 = "bgra8";
static constexpr const char *QC_REALSENSE_ENCODING_YUYV = "yuv422_yuy2";
static constexpr const char *QC_REALSENSE_ENCODING_16UC1 = "16UC1";
static constexpr const char *QC_REALSENSE_ENCODING_MONO8 = "mono8";
static constexpr const char *QC_REALSENSE_ENCODING_MONO16 = "mono16";

/** @brief The bytes per pixel of a RealSense image encoding, 0 for another encoding. */
inline uint32_t RealSenseEncodingBytesPerPixel( const char *encoding )
{
    static const struct
    {
        const char *name;
        uint32_t bytes;
    } kEncodings[] = { { QC_REALSENSE_ENCODING_RGB8, 3u },  { QC_REALSENSE_ENCODING_BGR8, 3u },
                       { QC_REALSENSE_ENCODING_RGBA8, 4u }, { QC_REALSENSE_ENCODING_BGRA8, 4u },
                       { QC_REALSENSE_ENCODING_YUYV, 2u },  { QC_REALSENSE_ENCODING_16UC1, 2u },
                       { QC_REALSENSE_ENCODING_MONO8, 1u }, { QC_REALSENSE_ENCODING_MONO16, 2u } };
    uint32_t bytes = 0u;
    if ( nullptr != encoding )
    {
        for ( const auto &entry : kEncodings )
        {
            if ( 0 == std::strcmp( encoding, entry.name ) )
            {
                bytes = entry.bytes;
                break;
            }
        }
    }
    return bytes;
}

/** @brief The bytes of one point cloud point, with or without the rgb texture. */
inline uint32_t RealSensePointStep( bool bTexture )
{
    return bTexture ? QC_REALSENSE_POINT_STEP_XYZRGB : QC_REALSENSE_POINT_STEP_XYZ;
}

/** @brief The streams of the RealSense node. */
typedef enum
{
    QC_REALSENSE_STREAM_NONE = 0,
    QC_REALSENSE_STREAM_COLOR,
    QC_REALSENSE_STREAM_DEPTH,
    QC_REALSENSE_STREAM_ALIGNED_DEPTH_TO_COLOR,
    QC_REALSENSE_STREAM_INFRA1,
    QC_REALSENSE_STREAM_INFRA2,
    QC_REALSENSE_STREAM_ACCEL,
    QC_REALSENSE_STREAM_GYRO,
    QC_REALSENSE_STREAM_IMU, /**< accel and gyro united */
    QC_REALSENSE_STREAM_POINTCLOUD,
    QC_REALSENSE_STREAM_COUNT
} QCRealSenseStream_e;

/** @brief The realsense2_camera name of a stream (color, depth, infra1, ...), "" for NONE. */
inline const char *RealSenseStreamName( QCRealSenseStream_e stream )
{
    static const char *kNames[QC_REALSENSE_STREAM_COUNT] = {
            "",     "color", "depth",     "aligned_depth_to_color", "infra1", "infra2", "accel",
            "gyro", "imu",   "pointcloud" };
    return ( ( stream > QC_REALSENSE_STREAM_NONE ) && ( stream < QC_REALSENSE_STREAM_COUNT ) )
                   ? kNames[stream]
                   : "";
}

/** @brief The librealsense timestamp domains (rs2_timestamp_domain). */
typedef enum
{
    QC_REALSENSE_TIMESTAMP_DOMAIN_HARDWARE_CLOCK = 0, /**< camera clock */
    QC_REALSENSE_TIMESTAMP_DOMAIN_SYSTEM_TIME = 1,    /**< host time of arrival */
    QC_REALSENSE_TIMESTAMP_DOMAIN_GLOBAL_TIME = 2,    /**< camera clock converted to host time */
    QC_REALSENSE_TIMESTAMP_DOMAIN_UNKNOWN = 3
} QCRealSenseTimestampDomain_e;

/**
 * @brief A RealSense image: one single-plane image in the DMA buffer, mirroring sensor_msgs/Image.
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

    QCRealSenseStream_e stream = QC_REALSENSE_STREAM_NONE; /**< Set by the application */
    uint32_t height = 0;                                   /**< Rows */
    uint32_t width = 0;                                    /**< Pixels per row */
    QCString<QC_REALSENSE_NAME_MAX_LEN> encoding;          /**< e.g. "rgb8", "16UC1", "mono8" */
    bool isBigEndian = false;
    uint32_t step = 0;              /**< Bytes per row, >= width * bytes per pixel */
    uint64_t timestampNs = 0;       /**< Acquisition time, see RealSenseConfig_t::clock */
    uint64_t frameNumber = 0;       /**< librealsense frame counter, a gap is a lost frame */
    QRB::Memory::FrameId_t frameId; /**< e.g. camera_color_optical_frame */

private:
    void CopyImageFields( const RealSenseImageDescriptor &other )
    {
        stream = other.stream;
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

/**
 * @brief The calibration of one image stream, mirroring sensor_msgs/CameraInfo as
 * realsense2_camera fills it.
 *
 * The infra2 CameraInfo is, as in realsense2_camera, in the infra1 optical frame with
 * P[3] = -fx * baseline (and P[7] = -fy * baseline y), for stereo processing.
 */
typedef struct RealSenseCameraInfo
{
    bool bValid = false; /**< False for the streams without intrinsics (IMU) */
    uint32_t width = 0;
    uint32_t height = 0;
    QCString<QC_REALSENSE_NAME_MAX_LEN> distortionModel; /**< "plumb_bob" or "equidistant" */
    uint32_t numD = 0;                                   /**< 5 (plumb_bob) or 4 (equidistant) */
    double D[5] = { 0.0 };                               /**< Distortion coefficients */
    double K[9] = { 0.0 };                               /**< Intrinsic matrix, row-major */
    double R[9] = { 0.0 };                               /**< Rectification matrix, identity */
    double P[12] = { 0.0 };                              /**< Projection matrix, row-major */
    QRB::Memory::FrameId_t frameId;
} RealSenseCameraInfo_t;

/**
 * @brief The information of one delivered sample, at index 1 of every event.
 *
 * Not backed by memory (pBuf is nullptr): valid during the callback only.
 */
typedef struct RealSenseFrameInfo : public QCBufferDescriptorBase_t
{
public:
    RealSenseFrameInfo() : QCBufferDescriptorBase_t() {}
    ~RealSenseFrameInfo() override = default;
    RealSenseFrameInfo( const RealSenseFrameInfo &other ) = default;
    using QCBufferDescriptorBase_t::operator=;
    RealSenseFrameInfo &operator=( const RealSenseFrameInfo &other ) = default;

    QCRealSenseStream_e stream = QC_REALSENSE_STREAM_NONE;
    QCString<QC_REALSENSE_NAME_MAX_LEN> cameraSerial; /**< Serial number of the camera */
    QRB::Memory::FrameId_t cameraName;                /**< Logical camera name, e.g. cam1 */
    uint64_t frameNumber = 0;       /**< librealsense frame counter (gyro one for IMU) */
    uint64_t timestampNs = 0;       /**< The delivered timestamp, as the payload one */
    double sensorTimestampMs = 0.0; /**< The original librealsense frame timestamp */
    QCRealSenseTimestampDomain_e timestampDomain = QC_REALSENSE_TIMESTAMP_DOMAIN_UNKNOWN;
    bool bHaveMetadataSensorTimestamp = false;
    uint64_t metadataSensorTimestampUs = 0; /**< RS2_FRAME_METADATA_SENSOR_TIMESTAMP */
    uint64_t hostReceiveNs = 0;             /**< Host time of arrival, in the configured clock */
    float depthScale = 0.0f; /**< Metres per depth unit (depth, aligned depth, pointcloud) */
    RealSenseCameraInfo_t cameraInfo; /**< Image and pointcloud streams, the CameraInfo */
} RealSenseFrameInfo_t;

/**
 * @brief Extrinsics from one stream to another, as realsense2_camera_msgs/Extrinsics:
 * rotation 3x3 column-major and translation in metres, in the optical frames.
 */
typedef struct RealSenseExtrinsics
{
    float rotation[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    float translation[3] = { 0.0f, 0.0f, 0.0f };
} RealSenseExtrinsics_t;

/** @brief The IMU calibration of the accel or gyro, as realsense2_camera_msgs/IMUInfo. */
typedef struct RealSenseImuInfo
{
    bool bValid = false;       /**< False if the camera exposes no motion intrinsics */
    double data[12] = { 0.0 }; /**< Scale and bias, 3x4 row-major */
    double noiseVariances[3] = { 0.0 };
    double biasVariances[3] = { 0.0 };
    QRB::Memory::FrameId_t frameId;
} RealSenseImuInfo_t;

/** @brief A RealSense camera, as found by EnumerateDevices() or opened by the node. */
typedef struct RealSenseDeviceInfo
{
    std::string name; /**< e.g. Intel RealSense D455 */
    std::string serial;
    std::string firmware;
    std::string productId;
    std::string productLine;
    std::string usbType; /**< e.g. 3.2 */
    std::string physicalPort;
    std::string cameraName; /**< Logical camera name (node only) */
    bool bConnected = false;
    int interCamSyncMode = -1; /**< Hardware sync role read from the camera, -1 unsupported */
    std::string syncGroup;     /**< (node only) */
    float depthScale = 0.0f;   /**< Metres per depth unit, 0 without depth */
    std::vector<std::string> sensors; /**< realsense2_camera module names, e.g. depth_module */
} RealSenseDeviceInfo_t;

/** @brief The clock of the delivered timestamps. */
typedef enum
{
    QC_REALSENSE_CLOCK_REALTIME = 0, /**< CLOCK_REALTIME, the ROS time of realsense2_camera */
    QC_REALSENSE_CLOCK_MONOTONIC     /**< CLOCK_MONOTONIC */
} QCRealSenseClock_e;

/** @brief How the accel samples are united with the gyro samples, as realsense2_camera. */
typedef enum
{
    QC_REALSENSE_UNITE_IMU_NONE = 0,            /**< Separate accel and gyro events */
    QC_REALSENSE_UNITE_IMU_COPY,                /**< The latest accel sample */
    QC_REALSENSE_UNITE_IMU_LINEAR_INTERPOLATION /**< Accel interpolated at the gyro timestamp */
} QCRealSenseUniteImu_e;

/** @brief One sensor option, realsense2_camera <module>.<option> (e.g. depth_module.exposure). */
typedef struct RealSenseOption
{
    std::string module; /**< depth_module, rgb_camera, motion_module, ... */
    std::string option; /**< e.g. exposure, gain, enable_auto_exposure */
    float value = 0.0f;
} RealSenseOption_t;

/** @brief The RealSense node configuration. */
typedef struct RealSenseConfig_s : public QCNodeConfigBase_t
{
    std::string serial; /**< Camera serial number, empty means the first camera found */
    std::string cameraName = "camera"; /**< Logical camera name and frame id prefix */
    std::string baseFrameId;           /**< Default <cameraName>_link */

    bool enableColor = true;
    uint32_t colorWidth = 640;
    uint32_t colorHeight = 480;
    uint32_t colorFps = 30;
    std::string colorEncoding = QC_REALSENSE_ENCODING_RGB8;

    bool enableDepth = true;
    uint32_t depthWidth = 640;
    uint32_t depthHeight = 480;
    uint32_t depthFps = 30;

    bool enableInfra1 = false;
    bool enableInfra2 = false;
    uint32_t infraWidth = 640;
    uint32_t infraHeight = 480;
    uint32_t infraFps = 30;
    std::string infraEncoding = QC_REALSENSE_ENCODING_MONO8;

    bool alignDepth = false; /**< Also deliver the depth aligned to color, needs both streams */

    bool enablePointcloud = false; /**< Needs the depth stream */
    bool pointcloudTexture = true; /**< rgb from the color stream, needs the color stream */
    bool orderedPointcloud = false;
    bool allowNoTexturePoints = false;

    bool enableSync = false; /**< Synchronized framesets, on with alignDepth or the pointcloud */

    bool enableAccel = false;
    bool enableGyro = false;
    uint32_t accelFps = 0; /**< 0 for the camera default */
    uint32_t gyroFps = 0;  /**< 0 for the camera default */
    QCRealSenseUniteImu_e uniteImuMethod = QC_REALSENSE_UNITE_IMU_COPY;
    bool rawImu = false; /**< With copy or linear_interpolation, also the accel and gyro */
    double linearAccelCov = 0.01;
    double angularVelocityCov = 0.01;

    int interCamSyncMode = -1; /**< -1: unchanged, 0 default, 1 master, 2 slave, ... */
    std::string syncGroup = "default";
    std::vector<RealSenseOption_t> sensorOptions; /**< Set in this order before streaming */

    QCRealSenseClock_e clock = QC_REALSENSE_CLOCK_REALTIME;
    uint32_t timeoutMs = 1000; /**< Time without any sample reported as a timeout */
} RealSenseConfig_t;

/** @brief The RealSense node monitoring data (diagnostics), per QCRealSenseStream_e. */
typedef struct RealSenseMonitor_s : public QCNodeMonitoringBase_t
{
    uint64_t frameCount[QC_REALSENSE_STREAM_COUNT];   /**< Samples delivered */
    uint64_t dropCount[QC_REALSENSE_STREAM_COUNT];    /**< Dropped as every buffer was held */
    uint64_t gapCount[QC_REALSENSE_STREAM_COUNT];     /**< Frame counter discontinuities */
    uint64_t missedFrames[QC_REALSENSE_STREAM_COUNT]; /**< Frames missing in the gaps */
    uint64_t maxConsecutiveMissed[QC_REALSENSE_STREAM_COUNT];
    uint64_t lastTimestampNs[QC_REALSENSE_STREAM_COUNT];
    uint64_t lastFrameNumber[QC_REALSENSE_STREAM_COUNT];
    double measuredFps[QC_REALSENSE_STREAM_COUNT]; /**< Over the last second or more */
    uint64_t timeoutCount;                         /**< Periods of timeoutMs without any sample */
    uint64_t errorCount;                           /**< librealsense or copy errors */
    uint64_t startFailCount;                       /**< Failed Start() calls */
    uint64_t disconnectCount;                      /**< Camera disconnections */
    bool bConnected;                               /**< The camera is connected */
    int32_t interCamSyncMode; /**< The hardware sync role set on the camera, -1 unchanged */
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

    /** @brief Parse "module.option=value[,module.option=value...]" ("," or ";" separated). */
    static bool ParseSensorOptions( const std::string &text,
                                    std::vector<RealSenseOption_t> &options, std::string &errors );

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

    /* Updated from the librealsense threads. */
    void OnFrame( QCRealSenseStream_e stream, uint64_t timestampNs, uint64_t frameNumber );
    void IncDrop( QCRealSenseStream_e stream );
    void IncTimeout();
    void IncError();
    void IncStartFail();
    void SetConnected( bool bConnected );
    void SetInterCamSyncMode( int32_t mode );
    /** @brief Restart the frame counter continuity, e.g. at Start(). */
    void ResetContinuity();
    void Reset();

private:
    void Snapshot();

    std::mutex m_lock;
    RealSenseMonitor_t m_live; /* updated under m_lock */
    RealSenseMonitor_t m_data; /* the snapshot returned by Get() */
    uint64_t m_windowStartNs[QC_REALSENSE_STREAM_COUNT];
    uint64_t m_windowFrames[QC_REALSENSE_STREAM_COUNT];
    bool m_bHaveFrameNumber[QC_REALSENSE_STREAM_COUNT];
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
     * @brief Initializes the node: parses the configuration, checks the buffers, binds to the
     * camera, checks the stream profiles, sets the sensor options and the hardware sync role.
     * @param[in] config The JSON configuration, the event callback (mandatory) and the buffers
     * (see the file description).
     */
    QCStatus_e Initialize( QCNodeInit_t &config ) override;
    QCNodeConfigIfs &GetConfigurationIfs() override;
    QCNodeMonitoringIfs &GetMonitoringIfs() override;

    /** @brief Opens and starts the sensors, samples are then delivered through the callback. */
    QCStatus_e Start() override;

    /**
     * @brief Gives a buffer delivered through the callback back to the node.
     * @param[in] frameDesc The frame descriptor with the buffer to release at index 0.
     */
    QCStatus_e ProcessFrameDescriptor( QCFrameDescriptorNodeIfs &frameDesc ) override;

    /** @brief Stops and closes the sensors. Must not be called from the callback. */
    QCStatus_e Stop() override;
    QCStatus_e DeInitialize() override;
    QCObjectState_e GetState() override;

    /** @brief The opened camera (READY or RUNNING). */
    QCStatus_e GetDeviceInfo( RealSenseDeviceInfo_t &info );

    /** @brief The CameraInfo of an enabled image stream or the pointcloud (READY or RUNNING). */
    QCStatus_e GetCameraInfo( QCRealSenseStream_e stream, RealSenseCameraInfo_t &info );

    /**
     * @brief The extrinsics from stream @p from to stream @p to (READY or RUNNING), e.g. DEPTH to
     * COLOR as realsense2_camera extrinsics/depth_to_color. The streams need not be enabled.
     */
    QCStatus_e GetExtrinsics( QCRealSenseStream_e from, QCRealSenseStream_e to,
                              RealSenseExtrinsics_t &extrinsics );

    /** @brief The IMU calibration of ACCEL or GYRO (READY or RUNNING). */
    QCStatus_e GetImuInfo( QCRealSenseStream_e stream, RealSenseImuInfo_t &info );

    /** @brief The static transforms of the enabled streams, as realsense2_camera /tf_static. */
    QCStatus_e GetStaticTransforms( std::vector<QRB::Memory::TransformStamped_t> &transforms );

    /**
     * @brief Writes the static transforms into @p tf, a buffer of at least the number of
     * transforms (see the vector overload) and sets tf.transformCount.
     */
    QCStatus_e GetStaticTransforms( QRB::Memory::TFMessageDescriptor_t &tf );

    /** @brief The connected RealSense cameras. */
    static QCStatus_e EnumerateDevices( std::vector<RealSenseDeviceInfo_t> &devices );

private:
    RealSenseConfig m_configIfs;
    RealSenseMonitoring m_monitorIfs;
    std::unique_ptr<RealSenseImpl> m_pImpl;
};

}   // namespace Node
}   // namespace QC

#endif   // QRB_QC_NODE_REALSENSE_HPP
