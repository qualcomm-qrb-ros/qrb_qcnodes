# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Repository documentation: README, contributing guidelines, code of conduct,
  security policy and license.
- RealSense QCNode (`source/Node/realsense`), the counterpart of the realsense2_camera
  ROS 2 node for the D455 and D405 through librealsense2: color, depth, depth aligned to
  color, infra1, infra2, point cloud (qrb_contracts `PointCloudDescriptor_t`, xyz + rgb,
  ordered or not), accel, gyro and united IMU (`none`, `copy`, `linear_interpolation`)
  streams; per-sample frame info (stream, camera serial and name, frame number, sensor
  timestamp and domain, host receive time, depth scale, CameraInfo); CameraInfo,
  extrinsics, IMU info, static TF (qrb_contracts `TransformStamped_t` /
  `TFMessageDescriptor_t`) and device enumeration queries; hardware sync role
  (`interCamSyncMode`) with a one-master-per-sync-group check, `enableSync`, generic
  `<module>.<option>` sensor options with explicit errors; per-stream diagnostics (counts,
  drops, frame counter gaps, measured fps, connection). Its `RealSense` sample measures
  the rates (`hz`), prints the calibration and TF (`info`), publishes every stream, records
  the IMU and a per-sample frame log; `tests/tools/realsense_sync_report.py` reports frame
  delivery and inter-camera synchronization from the frame logs.

### Changed
- RealSense QCNode: the camera is driven per sensor instead of through `rs2::pipeline`.
  Interface changes for the users of the first version: image buffers are tagged with
  their `stream`; `alignDepth` delivers the aligned depth in addition to the raw depth;
  `colorFrameId` / `depthFrameId` / `imuFrameId` are replaced by frame ids derived from
  `cameraName` / `baseFrameId`; events carry a `RealSenseFrameInfo_t` at index 1;
  `RealSenseMonitor_t` holds per-stream arrays; `uniteImuMethod` accepts `none`;
  hardware-clock timestamps are mapped to the host time instead of replaced by the
  arrival time.

[Unreleased]: https://github.com/qualcomm-qrb-ros/qrb_qcnodes/commits/main
