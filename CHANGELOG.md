# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Repository documentation: README, contributing guidelines, code of conduct,
  security policy and license.
- RealSense QCNode (`source/Node/realsense`): color (rgb8), depth (16UC1,
  optionally aligned to color) and IMU (accel united with gyro by copy or
  linear interpolation) streams of an Intel RealSense D400 camera through
  librealsense2, and its `RealSense` sample with rate measurement (`hz`),
  Recorder topics and IMU CSV recording.

[Unreleased]: https://github.com/qualcomm-qrb-ros/qrb_qcnodes/commits/main
