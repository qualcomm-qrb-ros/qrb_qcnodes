# Directory structure

This page describes the directory layout of the `qrb_qcnodes` repository and what
belongs in each directory. The tree is currently a scaffold: directories and
build files are in place, but no sources have landed yet.

Directories with no build file of their own are held by a `.gitkeep`
placeholder, which carries no meaning other than keeping the directory under
version control — delete it once the directory has real content.

```
qrb_qcnodes/
├── CMakeLists.txt               Top-level build entry point
├── docs/                        Markdown documentation
│   └── images/                  Image assets referenced by the docs
├── include/
│   └── QC/                      Public headers, installed with the package
│       ├── Common/              Sensor-independent public types and defines
│       └── Node/                Per-node public headers, node base classes
│                                and the extension-point interfaces
├── source/
│   ├── Common/                  Shared implementation not tied to any node
│   ├── Library/                 Wrappers over sensor and platform interfaces
│   └── Node/
│       ├── Base/                Node base classes and factory registration
│       ├── common/              Helpers shared by all sensor nodes
│       ├── imu/                 IMU QCNode implementation
│       └── lidar/               Lidar QCNode implementation
└── tests/
    ├── sample/                  Sample application showing graph integration
    └── unit_test/
        └── Node/
            ├── common/          Unit tests for the shared node helpers
            ├── imu/             Unit tests for the IMU QCNode
            └── lidar/           Unit tests for the Lidar QCNode
```

## Build files

Every directory that holds buildable content carries its own `CMakeLists.txt`
and is pulled in by an `add_subdirectory()` from its parent. The top-level
`CMakeLists.txt` is the entry point: it always descends into `source/`, and
descends into `tests/` only when `QRB_QCNODES_BUILD_TESTS` is `ON`.

`docs/` and `include/` carry no `CMakeLists.txt`. They hold nothing buildable,
and the public headers reach consumers through the include directories of the
targets that install them.

The leaf build files are placeholders holding only a comment, so the tree
configures and builds cleanly as it stands while producing no targets.

## docs

Markdown documentation, one page per node plus cross-cutting topics. Pages are
linked from `index.md`, which acts as the entry point for the documentation set.

`docs/images/` holds the image assets referenced from those pages. Reference
them with a relative path, for example `![](./images/imu-flow.png)`.

## include/QC

Public headers, installed with the package. Everything here is API that
consumers compile against, so changes are breaking changes. All declarations
live in the `QC` namespace, mirroring the directory path.

- `include/QC/Common/` — types and defines that are not tied to a specific
  sensor: payload and tensor types, error codes, and shared macros.
- `include/QC/Node/` — one public header per node, named after the node
  (`Imu.hpp`, `Lidar.hpp`), declaring its configuration struct and factory
  entry point. The node base classes live here too, as do the Graph Framework
  extension-point interfaces a node implements: the node lifecycle interface,
  the configuration interface, the monitoring interface, the frame descriptor
  interface, and the factory signature the framework uses to instantiate nodes.

Implementation details must not leak into this directory. Private headers
belong next to their `.cpp` file under `source/`.

## source

Implementation. Each directory carries its own `CMakeLists.txt` and is added by
the `CMakeLists.txt` of its parent.

- `source/Common/` — implementation shared across the whole package and
  independent of the node abstraction, such as logging and payload helpers.
  Platform-specific variants are split into separate files suffixed with the
  target OS.
- `source/Library/` — wrappers that isolate the nodes from sensor drivers and
  platform interfaces. One subdirectory per interface. Keeping these separate is
  what lets the nodes be unit tested against mocks instead of real hardware.
- `source/Node/Base/` — the node base classes, frame descriptor plumbing, and
  the factory that maps a node name to its constructor.
- `source/Node/common/` — helpers shared by the sensor nodes themselves:
  configuration parsing, monitoring counters, and buffer utilities.
- `source/Node/imu/`, `source/Node/lidar/` — one directory per node. A node
  directory holds the lifecycle entry points, the implementation class with its
  private header, configuration parsing, and monitoring, each in its own file.

## tests

- `tests/sample/` — a sample application that builds a graph from these nodes
  and runs it. This is the reference for how the nodes are meant to be
  configured and wired together, and doubles as a manual smoke test on target.
- `tests/unit_test/Node/` — GoogleTest unit tests, one directory per node,
  mirroring the layout of `source/Node/`. Test files are named
  `gtest_<unit under test>.cpp`. Mocks and stubs that stand in for the
  `source/Library/` wrappers live in a `Mock/` subdirectory of the test
  directory that uses them, so tests run on the host without hardware.
