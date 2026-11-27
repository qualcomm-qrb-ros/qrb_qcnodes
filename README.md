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
- `source/Node/realsense`: Intel RealSense QCNode. Acquires the color, depth and IMU streams of a RealSense depth camera (e.g. D455) through librealsense2.

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
| `source/Node/realsense` | Acquires RealSense color (rgb8), depth (16UC1, optionally aligned to color) and IMU samples and delivers them as `RealSenseImageDescriptor_t` images and qrb_contracts `ImuDescriptor_t` samples. |

Below is a summary of the currently planned nodes.

| QCNode | Sensor interface | Processor | Features |
| --- | --- | --- | --- |
| IMU | Not ready yet | CPU | Inertial sample acquisition and timestamping |
| Lidar | Not ready yet | CPU | Scan acquisition and point cloud frame generation |
| RealSense | USB (librealsense2) | CPU | Color, depth (optionally aligned to color) and united accel/gyro IMU samples, the QCNode counterpart of the realsense2_camera ROS node; sample: `qcrun QCNodeSampleApp -n rs0 -t RealSense` (see `tests/sample/SampleRealSense.cpp`) |

## Supported targets

| Development hardware | Operating system | Graph Framework |
| --- | --- | --- |
| Qualcomm Dragonwing IQ-10 EVK | Qualcomm Ubuntu 24.04 | Release 1.0 |

## Installation

Installation packages and setup instructions are not ready yet.

## Usage

Usage instructions of the IMU and Lidar nodes are not ready yet.

### RealSense camera QCNode

The RealSense node (`QC_NODE_TYPE_REALSENSE`, [`include/QC/Node/RealSense.hpp`](include/QC/Node/RealSense.hpp)) is the QCNode counterpart of the realsense2_camera ROS 2 node for the Intel RealSense D455 and D405. It opens one camera through librealsense2, by serial number, and delivers one event per sample through the node callback:

| Stream | Index 0 of the event | realsense2_camera topic |
| --- | --- | --- |
| color | `RealSenseImageDescriptor_t`, `rgb8` (or `bgr8`, `rgba8`, `bgra8`, `yuv422_yuy2`) | `color/image_raw` |
| depth | `RealSenseImageDescriptor_t`, `16UC1` (raw Z16; × depth scale = metres) | `depth/image_rect_raw` |
| aligned_depth_to_color | `RealSenseImageDescriptor_t`, `16UC1`, color resolution | `aligned_depth_to_color/image_raw` |
| infra1, infra2 | `RealSenseImageDescriptor_t`, `mono8` (or `mono16`) | `infra1/image_rect_raw`, `infra2/image_rect_raw` |
| pointcloud | qrb_contracts `PointCloudDescriptor_t`, `x y z` (+ `rgb`), ordered or not | `depth/color/points` |
| accel, gyro | qrb_contracts `ImuDescriptor_t` | `accel/sample`, `gyro/sample` |
| imu | qrb_contracts `ImuDescriptor_t`, accel united with gyro (`copy` or `linear_interpolation`) | `imu` |

Index 1 of every event is a `RealSenseFrameInfo_t`: stream, camera serial and logical camera name, librealsense frame number, the original sensor timestamp and its timestamp domain, the metadata sensor timestamp, the host receive time, the depth scale and the `CameraInfo` of the stream (K, D, R, P, distortion model, frame id).

The static information that realsense2_camera publishes on latched topics is queried from the initialized node (`dynamic_cast<RealSense *>` of the `QCNodeIfs`):

| API | realsense2_camera counterpart |
| --- | --- |
| `GetDeviceInfo()`, static `EnumerateDevices()` | device log (model, serial, firmware, USB type, port), `rs-enumerate-devices` |
| `GetCameraInfo(stream)` | `<stream>/camera_info` (infra2 in the infra1 frame with `P[3] = -fx * baseline`) |
| `GetExtrinsics(from, to)` | `extrinsics/depth_to_<stream>` |
| `GetImuInfo(accel/gyro)` | `accel/imu_info`, `gyro/imu_info` |
| `GetStaticTransforms()` (`TransformStamped_t` or a `TFMessageDescriptor_t`) | `/tf_static`: `camera_link` → `<camera>_<stream>_frame` → `<camera>_<stream>_optical_frame` |

Hardware synchronization: `interCamSyncMode` (`depth_module.inter_cam_sync_mode`: 0 default, 1 master, 2 slave) is set before streaming; the nodes of one process with the same `syncGroup` may have only one master. `enableSync` (synchronized framesets of one camera) is a different thing, and is on with `alignDepth` or the pointcloud. Any other sensor option is set with `sensorOptions` (`depth_module.exposure=8500,rgb_camera.enable_auto_exposure=0`); an unknown or out-of-range option fails `Initialize()` with the list of the supported ones, as does an unsupported stream profile (with the supported profiles).

The application gives the node DMA buffers at `Initialize()` (each image buffer tagged with its `stream`) and gets each buffer back through `ProcessFrameDescriptor()`; see the header for the configuration and the buffer contract. The sample application `tests/sample/SampleRealSense.cpp` does this and is the quickest way to run a camera.

#### Requirements

- A RealSense D455 or D405 on a USB 3 port (the D405 has no IMU and no hardware sync connector).
- librealsense2 2.58.4 on the target (`librealsense2` recipe of meta-qcom-robotics-sdk), with its udev rules, so that the node runs as a normal user.
- qcnode built with this node and its sample, see [Build from source](#build-from-source).

```bash
rs-enumerate-devices -s     # model, serial number, firmware
rs-enumerate-devices -c     # supported resolutions and frame rates, Accel/Gyro rates
```

#### Run the sample

Run from `/usr`, where `qcrun` finds `lib/`:

```bash
cd /usr

# color and depth, 640x480 at 30 fps, for 10 seconds
qcrun QCNodeSampleApp -d -T 10 -n rs0 -t RealSense

# the connected cameras, then the calibration and TF of one, as the latched realsense2_camera topics
qcrun QCNodeSampleApp -d -T 1 -n rs0 -t RealSense -k list_devices -v true -k info -v true \
  -k infra1 -v true -k infra2 -v true -k imu -v true

# the counterpart of
#   ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true \
#     enable_infra1:=true enable_infra2:=true pointcloud.enable:=true \
#     enable_gyro:=true enable_accel:=true gyro_fps:=200 accel_fps:=200 unite_imu_method:=2
qcrun QCNodeSampleApp -d -n rs0 -t RealSense -k enable_sync -v true -k align_depth -v true \
  -k infra1 -v true -k infra2 -v true -k pointcloud -v true \
  -k imu -v true -k gyro_fps -v 200 -k accel_fps -v 200 \
  -k unite_imu_method -v linear_interpolation -k hz -v true

# manual exposure on the depth module
qcrun QCNodeSampleApp -d -T 10 -n rs0 -t RealSense \
  -k sensor_options -v depth_module.enable_auto_exposure=0,depth_module.exposure=8500,depth_module.gain=16

# two D405 in one process, told apart by their serial numbers
qcrun QCNodeSampleApp -d \
  -n d405_1 -t RealSense -k serial -v 260422274812 -k camera_name -v d405_1 -k align_depth -v true \
  -n d405_2 -t RealSense -k serial -v 260422271130 -k camera_name -v d405_2 -k align_depth -v true
```

Without `-T` the sample runs until Ctrl+C.

| Attribute | Default | Description (realsense2_camera parameter) |
| --- | --- | --- |
| `serial` | first camera found | Camera serial number (`serial_no`) |
| `camera_name` | `camera` | Logical camera name and frame id prefix, e.g. `cam1` (`camera_name`) |
| `list_devices`, `info` | `false` | Print the connected cameras / the camera, CameraInfo, extrinsics, IMU info and TF at Init |
| `color`, `depth` | `true` | Enable the streams (`enable_color`, `enable_depth`) |
| `color_width`, `color_height`, `color_fps` | `640`, `480`, `30` | Color profile (`rgb_camera.color_profile`) |
| `color_format` | `rgb8` | `rgb8`, `bgr8`, `rgba8`, `bgra8` or `yuv422_yuy2` (`rgb_camera.color_format`) |
| `depth_width`, `depth_height`, `depth_fps` | `640`, `480`, `30` | Depth profile, Z16 (`depth_module.depth_profile`) |
| `infra1`, `infra2` | `false` | Enable the left / right IR stream (`enable_infra1`, `enable_infra2`) |
| `infra_width`, `infra_height`, `infra_fps` | `640`, `480`, `30` | IR profile (`depth_module.infra_profile`) |
| `infra_format` | `mono8` | `mono8` or `mono16` (`depth_module.infra1_format`) |
| `align_depth` | `false` | Also deliver the depth aligned to color (`align_depth.enable`) |
| `pointcloud` | `false` | Deliver the point cloud (`pointcloud.enable`) |
| `pointcloud_texture` | `color` | `color` (xyz + rgb) or `none` (xyz) (`pointcloud.stream_filter`) |
| `ordered_pc`, `allow_no_texture_points` | `false` | (`pointcloud.ordered_pc`, `pointcloud.allow_no_texture_points`) |
| `enable_sync` | `false` | Synchronized framesets of this camera (`enable_sync`) |
| `imu` | `false` | Enable accel and gyro |
| `accel`, `gyro` | the `imu` value | (`enable_accel`, `enable_gyro`) |
| `accel_fps`, `gyro_fps` | `0` (camera default) | IMU rates, as listed by `rs-enumerate-devices -c` (`accel_fps`, `gyro_fps`) |
| `unite_imu_method` | `copy` | `none`, `copy` or `linear_interpolation` (`unite_imu_method` 0, 1, 2) |
| `raw_imu` | `false` | With `copy` / `linear_interpolation`, also deliver the accel and gyro samples |
| `inter_cam_sync_mode` | `-1` (unchanged) | `0` default, `1` master, `2` slave (`depth_module.inter_cam_sync_mode`) |
| `sync_group` | `default` | Hardware sync group: one master per group in the process |
| `sensor_options` | none | `<module>.<option>=<value>,...` (`<module>.<option>`), e.g. `depth_module.exposure=8500` |
| `clock` | `realtime` | Timestamps in `realtime` (the ROS time of realsense2_camera) or `monotonic` |
| `buffer_num` | `4` | DMA buffers per image stream and for the pointcloud |
| `timeout_ms` | `1000` | Time without any sample reported as a timeout |
| `echo` | `true` (`false` with `hz`) | Print each image and pointcloud, and one IMU sample out of 100 |
| `hz` | `false` | Print the rate of each stream once a second, like `ros2 topic hz` |
| `<stream>_topic` | none | DataBroker topic of the descriptors (`std::shared_ptr<QCBufferDescriptorBase_t>`): `color`, `depth`, `aligned_depth`, `infra1`, `infra2`, `pointcloud`, `imu`, `accel`, `gyro` |
| `<stream>_frames_topic` | none | DataBroker topic of `DataFrames_t` for the image samples (Recorder, ...): `color`, `depth`, `aligned_depth`, `infra1`, `infra2`; rgb8 / bgr8 color as a `QC::Memory::ImageDescriptor_t` RGB888 / BGR888, the others as a plain buffer |
| `imu_file` | none | Write every IMU sample to this CSV file |
| `frame_log` | none | Write every sample (camera, serial, stream, frame number, timestamps) to this CSV file |
| `log_level` | `ERROR` | Node log level; `INFO` also prints the camera, depth unit, sensor options and sync role |

#### Output

With `echo`, one line per image and pointcloud, and per 100 IMU samples:

```
rs0: color stamp 1773417212.288688128 frame 1234 camera_color_optical_frame 640x480 rgb8 step 1920, center (24, 36, 38)
rs0: depth stamp 1773417212.288631808 frame 1236 camera_depth_optical_frame 640x480 16UC1 step 1280, center 873
rs0: aligned_depth_to_color stamp 1773417212.288631808 frame 1236 camera_color_optical_frame 640x480 16UC1 step 1280, center 870
rs0: infra1 stamp 1773417212.288631808 frame 1236 camera_infra1_optical_frame 640x480 mono8 step 640, center 112
rs0: pointcloud stamp 1773417212.288631808 camera_depth_optical_frame 251234x1 points 251234 step 16 xyzrgb, middle point (0.012, -0.031, 0.874)
rs0: imu stamp 1773417212.541964288 camera_imu_optical_frame angular_velocity (0.0021, 0.0021, -0.0021) linear_acceleration (-0.2746, -9.7870, 0.1079), sample 201
```

`stamp` is the timestamp in seconds, `frame` the librealsense frame number, `step` the bytes per row and `center` the value of the image center: RGB, raw depth (× the depth unit printed with `info` is metres: 0.001 m on the D455, 0.0001 m by default on the D405; `0` means no depth there) or IR intensity. At rest the linear acceleration is about 9.8 m/s^2 and the angular velocity about 0.

At Stop the sample prints the node diagnostics per stream, then the camera status; `dropped`, `gaps`, `timeouts` and `errors` stay 0 in a healthy run:

```
rs0: color 300 (dropped 0, gaps 0, missed 0, max consecutive missed 0, fps 30.00)
rs0: depth 300 (dropped 0, gaps 0, missed 0, max consecutive missed 0, fps 30.00)
rs0: connected yes, sync mode 0, timeouts 0, errors 0, disconnects 0, release errors 0
```

#### Check the rates

`-k hz -v true` measures the arrival rate of each stream, as `ros2 topic hz` does, once a second and again at Stop:

```
rs0: color 640x480    average rate: 30.002	min: 0.032s max: 0.035s std dev: 0.00045s window: 299
rs0: depth 640x480    average rate: 30.001	min: 0.032s max: 0.035s std dev: 0.00047s window: 299
rs0: imu 200Hz        average rate: 199.980	min: 0.004s max: 0.006s std dev: 0.00031s window: 1995
```

`min`, `max` and `std dev` are taken over the intervals between consecutive samples, and `window` is the number of intervals since Start. A `max` near twice the frame period means a frame was lost. With `linear_interpolation` the IMU samples arrive in batches, so their arrival intervals are not meaningful; check the IMU on its timestamps with `imu_file` instead.

To filter the output down to the sample lines:

```bash
dmesg -n 1     # no kernel messages on the console
qcrun QCNodeSampleApp -d -n rs0 -t RealSense -k align_depth -v true -k imu -v true \
  -k gyro_fps -v 200 -k accel_fps -v 200 -k hz -v true 2>&1 | grep --line-buffered "rs0:"
```

Record the images with the Recorder sample. Each color frame is written to `/tmp/RS_COLOR_<n>_0.raw`, with one line per frame in `/tmp/RS_COLOR.meta` (the frame id is the librealsense frame number, so a gap is a lost frame); depth and IR frames are written as raw buffers:

```bash
qcrun QCNodeSampleApp -d -T 15 \
  -n rs0 -t RealSense -k align_depth -v true -k buffer_num -v 8 \
    -k color_frames_topic -v /rs/color/frames -k aligned_depth_frames_topic -v /rs/depth/frames \
  -n RS_COLOR -t Recorder -k max -v 300 -k topic -v /rs/color/frames \
  -n RS_DEPTH -t Recorder -k max -v 300 -k topic -v /rs/depth/frames

# frame rate and frame number gaps of the color frames
awk '{ id=$3; ts=$5+0; if (NR>1 && id!=pid+1) gap++; if (NR==1) t0=ts; t1=ts; pid=id; n++ }
     END { printf "frames %d  fps %.2f  frameId gaps %d\n", n, (n-1)/((t1-t0)/1e9), gap }' /tmp/RS_COLOR.meta

# view a frame on a host
ffmpeg -f rawvideo -pixel_format rgb24 -video_size 640x480 -i RS_COLOR_0_0.raw color_0.png
ffmpeg -f rawvideo -pixel_format gray16le -video_size 640x480 -i RS_DEPTH_0_0.raw -vf normalize depth_0.png
```

Record the IMU samples to a CSV file (`index,frame_number,timestamp_ns,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,stream`, the frame number of the gyro sample for `imu`) and compute the rate on their timestamps, skipping the first 100 samples while the timestamps settle:

```bash
qcrun QCNodeSampleApp -d -T 15 -n rs0 -t RealSense -k imu -v true \
  -k gyro_fps -v 200 -k accel_fps -v 200 -k unite_imu_method -v linear_interpolation \
  -k imu_file -v /tmp/rs_imu.csv

awk -F, 'NR<=101 { next }
  { fn=$2; ts=$3/1e9
    if (n) { dt=ts-pts; if (n==1 || dt<mn) mn=dt; if (n==1 || dt>mx) mx=dt; s+=dt; ss+=dt*dt
             if (fn!=pfn+1) gap++ }
    else t0=ts
    n++; pts=ts; pfn=fn }
  END { m=s/(n-1); printf "samples %d  rate %.1f Hz  interval min %.4f max %.4f std %.5f s  frame_number gaps %d\n",
        n, (n-1)/(pts-t0), mn, mx, sqrt(ss/(n-1)-m*m), gap }' /tmp/rs_imu.csv
```

#### Multi-camera hardware synchronization

Connect the D455 cameras with the RealSense sync cable, each camera on its own USB 3 link to the host (a powered hub when the ports cannot power them all). Give each camera a logical name and its serial number, one master and the others slaves, and log every sample with `frame_log`:

```bash
qcrun QCNodeSampleApp -d -T 600 \
  -n cam1 -t RealSense -k serial -v <SN1> -k camera_name -v cam1 -k inter_cam_sync_mode -v 1 \
    -k depth -v false -k infra1 -v true -k infra2 -v true -k imu -v true -k gyro_fps -v 200 \
    -k accel_fps -v 200 -k echo -v false -k frame_log -v /tmp/cam1.csv \
  -n cam2 -t RealSense -k serial -v <SN2> -k camera_name -v cam2 -k inter_cam_sync_mode -v 2 \
    -k depth -v false -k infra1 -v true -k infra2 -v true -k echo -v false -k frame_log -v /tmp/cam2.csv \
  -n cam3 -t RealSense -k serial -v <SN3> -k camera_name -v cam3 -k inter_cam_sync_mode -v 2 \
    -k depth -v false -k infra1 -v true -k infra2 -v true -k echo -v false -k frame_log -v /tmp/cam3.csv
```

A second master in the same `sync_group` fails `Initialize()`. On a host, [`tests/tools/realsense_sync_report.py`](tests/tools/realsense_sync_report.py) reports from the logs the rate, frame counter gaps, missing frame rate and maximum consecutive missing frames per camera and stream, the complete multi-camera frameset ratio, and the inter-camera timestamp gap (mean, std, P50, P90, P99, min, max, outliers) against the first camera:

```bash
python3 tests/tools/realsense_sync_report.py cam1.csv cam2.csv cam3.csv --streams color infra1 infra2
```

## Build from source

The nodes are built as part of the QcNode project, which links them into `libQCNode.so`, and their samples into the QCNode sample application.

### RealSense node

The RealSense node needs the librealsense2 headers and library in the build sysroot; it takes them from the librealsense2 CMake package (`realsense2::realsense2`), or else from `usr/include/librealsense2` and `usr/lib/librealsense2.so*` of `CMAKE_SYSROOT`.

- Yocto: the `qcnode_%.bbappend` of [meta-qcom-robotics-sdk](https://github.com/qualcomm-linux/meta-qcom-robotics-sdk) adds this repository to the qcnode build, registers the node directory (`realsense:QCNodeRealSense` in `qrb_qcnodes.cmake`) and adds `DEPENDS += "librealsense2"`. Then:

  ```bash
  bitbake qcnode
  ```

- QcNode tree: add the node directory to the QcNode build and link it into `QCNode`, as for the other nodes (`source/Node/realsense` builds the OBJECT library `QCNodeRealSense` and adds `tests/sample/SampleRealSense.cpp` to `QCNodeSample`). If the sysroot is older than the device image, `-DREALSENSE_ALLOW_SHLIB_UNDEFINED=ON` skips the link-time check of the librealsense2 dependencies.

Check the result: `libQCNode.so` needs `librealsense2.so.2.58`, and `libQCNodeSample.so` registers the `RealSense` sample type.

```bash
readelf -d libQCNode.so | grep realsense
QCNodeSampleApp 2>&1 | grep "register sample type RealSense"
```

## Test

Test instructions are not ready yet.

## Contributing

Please read our [contributing guidelines](CONTRIBUTING.md) before submitting a pull request, and follow our [code of conduct](CODE-OF-CONDUCT.md). Commits must be signed off under the [DCO](http://developercertificate.org/).

To report a potential security vulnerability, see [SECURITY.md](SECURITY.md).

## License

This project is licensed under the BSD-3-Clause-Clear license. See [LICENSE](LICENSE) for the full license text.
