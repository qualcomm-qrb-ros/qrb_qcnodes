// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_QC_NODE_REALSENSE_IMPL_HPP
#define QRB_QC_NODE_REALSENSE_IMPL_HPP

#include "QC/Node/RealSense.hpp"
#include "QRB/Memory/Sensor/ImuDescriptor.hpp"

#include <librealsense2/rs.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace QC
{
namespace Node
{

/**
 * @brief The RealSense node implementation.
 *
 * Threading model:
 *  - Initialize/Start/Stop/DeInitialize are called by the application thread.
 *  - librealsense calls OnFrame() from its own threads: the synchronized color/depth framesets
 *    from one, the IMU samples, which bypass the synchronization, from another. m_callbackLock
 *    serializes the application callback across them.
 *  - ReleaseFrame (ProcessFrameDescriptor) can be called from any thread, including from inside
 *    the callback. The buffer state is protected by m_bufLock, which is never held while the
 *    callback runs.
 *  - The watchdog thread counts the periods of timeoutMs without any sample.
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

private:
    /** @brief The image streams, the index of their buffer pool. */
    enum ImageStream
    {
        kColor = 0,
        kDepth = 1,
        kImageStreamCount = 2
    };

    /** @brief The buffers of one image stream, pointing to the application DMA memory. */
    struct ImagePool
    {
        std::vector<RealSenseImageDescriptor_t> images;
        std::vector<bool> inUse;
    };

    /** @brief One motion sample, with the timestamp it is delivered with. */
    struct MotionSample
    {
        rs2_vector data;
        uint64_t timestampNs;
        uint64_t frameNumber; /* librealsense frame counter of the stream */
    };

    QCStatus_e
    SetupBuffers( std::vector<std::reference_wrapper<QCBufferDescriptorBase_t>> &buffers );
    QCStatus_e AddBuffer( size_t index, const QCBufferDescriptorBase_t &bufDesc );
    QCStatus_e OpenCamera();

    void OnFrame( const rs2::frame &frame );
    void Dispatch( const rs2::frame &frame );
    void HandleImage( ImageStream stream, const rs2::video_frame &frame );
    void HandleMotion( const rs2::motion_frame &frame );
    void DeliverImu( const MotionSample &gyro, const rs2_vector &accel );
    QCStatus_e CopyImage( const rs2::video_frame &frame, RealSenseImageDescriptor_t &out );
    uint64_t TimestampNs( const rs2::frame &frame ) const;
    void Deliver( QCBufferDescriptorBase_t &bufDesc );

    uint32_t AcquireBuffer( ImageStream stream );
    void ReleaseBuffer( ImageStream stream, uint32_t index );

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

    ImagePool m_pools[kImageStreamCount];
    std::mutex m_bufLock;

    /* Created in Initialize() inside a try block, as librealsense reports errors by exceptions. */
    std::unique_ptr<rs2::pipeline> m_pipeline;
    std::unique_ptr<rs2::config> m_rsConfig;
    std::unique_ptr<rs2::align> m_align;
    bool m_bStreaming = false;

    /* The accel/gyro union: the latest accel sample, and for linear_interpolation the gyro
     * samples received since it, waiting for the next accel sample. */
    std::mutex m_imuLock;
    bool m_bHaveAccel = false;
    MotionSample m_lastAccel{ { 0.0f, 0.0f, 0.0f }, 0, 0 };
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
