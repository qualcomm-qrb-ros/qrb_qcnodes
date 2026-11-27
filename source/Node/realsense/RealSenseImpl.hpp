// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_QC_NODE_REALSENSE_IMPL_HPP
#define QRB_QC_NODE_REALSENSE_IMPL_HPP

#include "QC/Node/RealSense.hpp"

#include <librealsense2/rs.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace QC
{
namespace Node
{

/**
 * @brief The RealSense node implementation.
 *
 * The camera is driven per sensor, as realsense2_camera does: each librealsense sensor
 * (depth_module, rgb_camera, motion_module) is opened with the stream profiles of the enabled
 * streams it carries, and started with its own callback.
 *
 * Threading model:
 *  - Initialize/Start/Stop/DeInitialize are called by the application thread.
 *  - librealsense calls OnVideo() and OnMotion() from one thread per sensor. With enableSync (or
 *    alignDepth, or the pointcloud) the video frames go through an asynchronous_syncer, which
 *    calls OnSynced() with the framesets. m_callbackLock serializes the application callback
 *    across all of them.
 *  - ReleaseFrame (ProcessFrameDescriptor) can be called from any thread, including from inside
 *    the callback. The buffer state is protected by m_bufLock, which is never held while the
 *    callback runs.
 *  - The watchdog thread counts the periods of timeoutMs without any sample.
 *  - librealsense calls OnDevicesChanged() from its own thread when a camera is connected or
 *    disconnected.
 */
class RealSenseImpl
{
public:
    RealSenseImpl( QCNodeID_t &nodeId, Logger &logger, RealSenseMonitoring &monitor );
    ~RealSenseImpl();

    RealSenseImpl( const RealSenseImpl & ) = delete;
    RealSenseImpl &operator=( const RealSenseImpl & ) = delete;

    QCStatus_e Initialize( const RealSenseConfig_t &config, QCNodeEventCallBack_t callback,
                           std::vector<std::reference_wrapper<QCBufferDescriptorBase_t>> &buffers );
    QCStatus_e Start();
    QCStatus_e ReleaseFrame( QCFrameDescriptorNodeIfs &frameDesc );
    QCStatus_e Stop();
    QCStatus_e DeInitialize();
    QCObjectState_e GetState() const { return m_state; }

    QCStatus_e GetDeviceInfo( RealSenseDeviceInfo_t &info );
    QCStatus_e GetCameraInfo( QCRealSenseStream_e stream, RealSenseCameraInfo_t &info );
    QCStatus_e GetExtrinsics( QCRealSenseStream_e from, QCRealSenseStream_e to,
                              RealSenseExtrinsics_t &extrinsics );
    QCStatus_e GetImuInfo( QCRealSenseStream_e stream, RealSenseImuInfo_t &info );
    QCStatus_e GetStaticTransforms( std::vector<QRB::Memory::TransformStamped_t> &transforms );

    static QCStatus_e EnumerateDevices( std::vector<RealSenseDeviceInfo_t> &devices );

private:
    /** @brief A librealsense stream the node opens: color, depth, infra1/2, accel, gyro. */
    struct StreamSetup
    {
        bool bEnabled = false;
        rs2_stream type = RS2_STREAM_ANY;
        int index = -1; /* -1: any stream index */
        uint32_t width = 0;
        uint32_t height = 0;
        uint32_t fps = 0; /* 0: the default profile (motion streams) */
        rs2_format format = RS2_FORMAT_ANY;
        int sensor = -1;             /* index in m_sensors of the sensor carrying it */
        rs2::stream_profile profile; /* the selected profile, or a reference one if disabled */
        bool bHaveProfile = false;
    };

    /** @brief The buffers of one stream, pointing to the application DMA memory. */
    struct Pool
    {
        std::vector<RealSenseImageDescriptor_t> images;
        std::vector<QRB::Memory::PointCloudDescriptor_t> clouds;
        std::vector<bool> inUse;
        size_t Size() const { return images.size() + clouds.size(); }
    };

    /** @brief The timestamps and counter of one librealsense frame. */
    struct FrameTiming
    {
        uint64_t frameNumber = 0;
        uint64_t stampNs = 0;
        double sensorMs = 0.0;
        QCRealSenseTimestampDomain_e domain = QC_REALSENSE_TIMESTAMP_DOMAIN_UNKNOWN;
        bool bHaveMetadataSensorTs = false;
        uint64_t metadataSensorTsUs = 0;
        uint64_t hostReceiveNs = 0;
    };

    /** @brief One motion sample. */
    struct MotionSample
    {
        rs2_vector data;
        FrameTiming timing;
    };

    /** @brief Calls OnDevicesChanged() as long as the node is alive. */
    struct DeviceWatch
    {
        std::mutex lock;
        RealSenseImpl *pImpl = nullptr;
    };

    QCStatus_e
    SetupBuffers( std::vector<std::reference_wrapper<QCBufferDescriptorBase_t>> &buffers );
    QCStatus_e AddBuffer( size_t index, const QCBufferDescriptorBase_t &bufDesc );
    void SetupStreams();
    QCStatus_e OpenDevice();
    QCStatus_e SelectProfiles();
    QCStatus_e FindProfile( QCRealSenseStream_e stream, StreamSetup &setup, bool bReference );
    QCStatus_e ApplySensorOptions();
    QCStatus_e ApplyInterCamSync();
    QCStatus_e CheckSensorsOpen();
    void ComputeCalibration();
    void ComputeStaticTransforms();
    QCStatus_e RegisterSyncRole();
    void UnregisterSyncRole();
    int FindSensor( const std::string &module ) const;
    const rs2::stream_profile *ProfileOf( QCRealSenseStream_e stream ) const;

    void OnVideo( const rs2::frame &frame );
    void OnSynced( const rs2::frame &frame );
    void OnMotion( const rs2::frame &frame );
    void OnDevicesChanged( rs2::event_information &info );
    void DispatchVideo( const rs2::frame &frame );
    void HandleImage( QCRealSenseStream_e stream, const rs2::video_frame &frame );
    void HandlePointcloud( const rs2::depth_frame &depth, const rs2::video_frame &color );
    void HandleMotion( const rs2::motion_frame &frame );
    void DeliverMotion( QCRealSenseStream_e stream, const MotionSample &sample );
    void DeliverImu( const MotionSample &gyro, const rs2_vector &accel );
    QCStatus_e CopyImage( const rs2::video_frame &frame, RealSenseImageDescriptor_t &out );
    FrameTiming Timing( const rs2::frame &frame );
    uint64_t StampNs( const rs2::frame &frame, double sensorMs, rs2_timestamp_domain domain );
    int64_t ToClockNs( int64_t realtimeNs ) const;
    void FillInfo( RealSenseFrameInfo_t &info, QCRealSenseStream_e stream,
                   const FrameTiming &timing ) const;
    void Deliver( QCBufferDescriptorBase_t &payload, RealSenseFrameInfo_t &info );

    uint32_t AcquireBuffer( QCRealSenseStream_e stream );
    void ReleaseBuffer( QCRealSenseStream_e stream, uint32_t index );

    void WatchdogMain();
    void StopStreaming();
    void Reset();

private:
    QCNodeID_t &m_nodeId;
    Logger &m_logger;
    RealSenseMonitoring &m_monitor;

    RealSenseConfig_t m_config;
    QCNodeEventCallBack_t m_callback;
    std::mutex m_callbackLock;
    std::atomic<QCObjectState_e> m_state{ QC_OBJECT_STATE_INITIAL };

    Pool m_pools[QC_REALSENSE_STREAM_COUNT];
    std::mutex m_bufLock;

    /* Created in Initialize() inside a try block, as librealsense reports errors by exceptions. */
    std::unique_ptr<rs2::context> m_ctx;
    std::unique_ptr<rs2::device> m_device;
    std::shared_ptr<DeviceWatch> m_watch;
    std::vector<rs2::sensor> m_sensors;
    std::vector<std::string> m_modules; /* realsense2_camera module name of each sensor */
    std::vector<std::vector<rs2::stream_profile>> m_sensorProfiles; /* to open, per sensor */
    std::vector<bool> m_sensorOpen;
    std::vector<bool> m_sensorStarted;
    StreamSetup m_streams[QC_REALSENSE_STREAM_COUNT];
    RealSenseCameraInfo_t m_cameraInfo[QC_REALSENSE_STREAM_COUNT];
    std::string m_frameIds[QC_REALSENSE_STREAM_COUNT];   /* <camera>_<stream>_frame */
    std::string m_opticalIds[QC_REALSENSE_STREAM_COUNT]; /* the frame id of the samples */
    std::vector<QRB::Memory::TransformStamped_t> m_staticTfs;
    RealSenseDeviceInfo_t m_deviceInfo;
    std::mutex m_infoLock;
    float m_depthScale = 0.0f;
    bool m_bSyncFrames = false;
    bool m_bSyncRegistered = false;

    std::unique_ptr<rs2::asynchronous_syncer> m_asyncer;
    std::unique_ptr<rs2::align> m_align;
    std::unique_ptr<rs2::pointcloud> m_pointcloud;

    /* Hardware clock domain: the camera time mapped to the host time, as realsense2_camera. */
    std::mutex m_timeBaseLock;
    bool m_bTimeBase = false;
    int64_t m_hostTimeBaseNs = 0;
    double m_cameraTimeBaseMs = 0.0;
    std::map<int, double> m_previousCameraMs;

    /* The accel/gyro union: the latest accel sample, and for linear_interpolation the gyro
     * samples received since it, waiting for the next accel sample. */
    std::mutex m_imuLock;
    bool m_bHaveAccel = false;
    MotionSample m_lastAccel{};
    std::deque<MotionSample> m_pendingGyros;

    /* Watchdog: the steady clock time of the last sample, in nanoseconds. */
    std::atomic<int64_t> m_lastSampleNs{ 0 };
    std::thread m_watchdog;
    std::mutex m_watchdogLock;
    std::condition_variable m_watchdogCv;
    bool m_bRunning = false;
};

}   // namespace Node
}   // namespace QC

#endif   // QRB_QC_NODE_REALSENSE_IMPL_HPP
