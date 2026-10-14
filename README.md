<div align="center">
  <h1>QRB QCNodes</h1>
  <p>Sensor QCNodes for Qualcomm Robotics Platforms</p>

  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <img src="https://img.shields.io/badge/Graph%20Framework-3253DC?style=for-the-badge&logo=qualcomm&logoColor=white" alt="Graph Framework">
</div>

---

## Overview

[QRB QCNodes](https://github.com/qualcomm-qrb-ros/qrb_qcnodes) provides sensor QCNodes for the Qualcomm Graph Framework. Each node implements the standard `QCNodeIfs` extension point, so the Graph Framework owns scheduling, threads, frame descriptors, and buffer lifetime, while the node handles sensor acquisition and conversion into Contract payloads.

The repository is organized as one directory per node, plus shared code:

- `common`: sensor-independent helpers shared by all nodes — configuration parsing, monitoring counters, and buffer utilities.
- `imu`: IMU QCNode. Acquires inertial samples and publishes them as frames.
- `lidar`: Lidar QCNode. Acquires scans and publishes them as point cloud frames.

Additional sensor nodes will be added to this repository over time.

Installation, build, runtime, and test workflows are not ready yet.

## Table of contents

- [APIs](#apis)
- [Supported targets](#supported-targets)
- [Installation](#installation)
- [Usage](#usage)
- [Build from source](#build-from-source)
- [Test](#test)
- [Contributing](#contributing)
- [License](#license)

## APIs

Every node in this repository implements `QCNodeIfs`, and exposes configuration and monitoring through `QCNodeConfigIfs` and `QCNodeMonitoringIfs`. The Graph Framework creates each node through a `NodeFactoryFn`-compatible factory and drives it through `Initialize()`, `Start()`, `ProcessFrameDescriptor()`, `Stop()`, and `DeInitialize()`.

| Component | Responsibility |
| --- | --- |
| `common` | Shared configuration parsing, monitoring counters, and buffer helpers. |
| `imu` | Acquires inertial samples and converts them into Contract payloads. |
| `lidar` | Acquires scans and converts them into point cloud Contract payloads. |

Below is a summary of the currently planned nodes.

| QCNode | Sensor interface | Processor | Features |
| --- | --- | --- | --- |
| IMU | Not ready yet | CPU | Inertial sample acquisition and timestamping |
| Lidar | Not ready yet | CPU | Scan acquisition and point cloud frame generation |

## Supported targets

| Development hardware | Operating system | Graph Framework |
| --- | --- | --- |
| Qualcomm Dragonwing IQ-10 EVK | Qualcomm Ubuntu 24.04 | Release 1.0 |

## Installation

Installation packages and setup instructions are not ready yet.

## Usage

Node configuration and graph integration instructions are not ready yet.

## Build from source

Source build instructions are not ready yet.

## Test

Test instructions are not ready yet.

## Contributing

Contribution guidance is not ready yet for this initial release.

## License

This project is licensed under the BSD-3-Clause-Clear license.
