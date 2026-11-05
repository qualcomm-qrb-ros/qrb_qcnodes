<div align="center">
  <h1>QRB QCNodes</h1>
  <p>Sensor QCNodes for Qualcomm Robotics Platforms</p>

  <a href="https://ubuntu.com/download/qualcomm-iot" target="_blank"><img src="https://img.shields.io/badge/Qualcomm%20Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" alt="Qualcomm Ubuntu"></a>
  <img src="https://img.shields.io/badge/Graph%20Framework-3253DC?style=for-the-badge&logo=qualcomm&logoColor=white" alt="Graph Framework">
</div>

---

## Overview

[QRB QCNodes](https://github.com/qualcomm-qrb-ros/qrb_qcnodes) provides sensor QCNodes for the Qualcomm Graph Framework. Each node implements the standard `QCNodeIfs` extension point, so the Graph Framework owns scheduling, threads, frame descriptors, and buffer lifetime, while the node handles sensor acquisition and conversion into Contract payloads.

The repository is organized as one directory per node under `source/Node/`, plus shared code:

- `source/Node/common`: sensor-independent helpers shared by all nodes — configuration parsing, monitoring counters, and buffer utilities.
- `source/Node/imu`: IMU QCNode. Acquires inertial samples and publishes them as frames.
- `source/Node/lidar`: Lidar QCNode. Acquires scans and publishes them as point cloud frames.

Public headers are installed from `include/QC/`, and tests live under `tests/`. See [directory structure](docs/directory-structure.md) for the full layout and for what belongs in each directory.

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

The extension-point interfaces are declared under `include/QC/Node/`, alongside the per-node public headers through which each node exposes its configuration struct and factory entry point.

| Component | Responsibility |
| --- | --- |
| `source/Node/common` | Shared configuration parsing, monitoring counters, and buffer helpers. |
| `source/Node/imu` | Acquires inertial samples and converts them into Contract payloads. |
| `source/Node/lidar` | Acquires scans and converts them into point cloud Contract payloads. |

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

Please read our [contributing guidelines](CONTRIBUTING.md) before submitting a pull request, and follow our [code of conduct](CODE-OF-CONDUCT.md). Commits must be signed off under the [DCO](http://developercertificate.org/).

To report a potential security vulnerability, see [SECURITY.md](SECURITY.md).

## License

This project is licensed under the BSD-3-Clause-Clear license. See [LICENSE](LICENSE) for the full license text.
