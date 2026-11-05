# External Libraries and Tools

This directory contains git submodules referencing external libraries and tools used by Paparazzi. These modules provide essential functionality for communications, flight control, simulation, cryptography, and more.

## Table of Contents
- [Quick Start](#quick-start)
- [Installing All Modules](#installing-all-modules)
- [Installing Individual Modules](#installing-individual-modules)
- [Available Modules](#available-modules)
- [OpenCV Bebop Setup](#opencv-bebop-setup)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

Initialize and build all external modules:

```bash
make -C sw/ext
```

This will checkout and compile all dependencies automatically.

---

## Installing All Modules

To install and build all external modules at once:

```bash
make -C sw/ext
```

This runs the complete build chain for all modules that require compilation (chibios, fatfs, libsbp, TRICAL, hacl-c, key_generator, rustlink, ecl, matrix, mavlink, dronecan, and unifiedmocaprouter).

---

## Installing Individual Modules

To install or update only a specific module, run:

```bash
make -C sw/ext <module-name>
```

**Example:**
```bash
make -C sw/ext chibios
make -C sw/ext mavlink
```

Each module target will sync and update the git submodule, then build if required.

---

## Available Modules

See the main [Makefile in this directory](Makefile) for the complete list of modules and their build targets. Common modules include:

- **chibios** — Operating system for embedded systems
- **fatfs** — FAT filesystem library
- **mavlink** — MAVLink message definitions and code generation
- **pprzlink** — Paparazzi-specific communications protocol
- **libsbp** — Swift Binary Protocol for GNSS/RTK
- **TRICAL** — Magnetometer automatic calibration
- **opencv_bebop** — Computer vision library (see detailed instructions below)
- **dronecan** — DroneCAN protocol support
- **And many more...** — See [Makefile](Makefile) for details

---

## OpenCV Bebop Setup

This module provides advanced computer vision capabilities for Bebop drones.

### Prerequisites

This guide has been tested on **Ubuntu 22.04**. For Ubuntu 20.04 instructions, see: [`conf/modules/cv_opencvdemo.xml`](../../conf/modules/cv_opencvdemo.xml)

### Building OpenCV Bebop

From the paparazzi source directory:

```bash
cd sw/ext
make opencv_bebop
```

### Verify Installation

After a successful build, you should have these directories:

```
sw/ext/opencv_bebop/build_arm/
sw/ext/opencv_bebop/build_pc/
sw/ext/opencv_bebop/install_arm/
sw/ext/opencv_bebop/install_pc/
```

These contain compiled binaries for ARM (autopilot) and PC (simulation) targets.

### Configuring Your Airframe Module

To use OpenCV in your airframe, configure your module XML with build flags for both autopilot (`ap`) and simulation (`nps`) targets.

**Example airframe module configuration:**

```xml
<makefile target="ap">
    <file name="opencv_example.cpp"/>
    <file name="opencv_image_functions.cpp"/>
    <flag name="CXXFLAGS" value="I$(PAPARAZZI_SRC)/sw/ext/opencv_bebop/install_arm/include/opencv4"/>
    
    <flag name="LDFLAGS" value="L$(PAPARAZZI_HOME)/sw/ext/opencv_bebop/install_arm/lib"/>
    <flag name="LDFLAGS" value="lopencv_world"/>
    <flag name="LDFLAGS" value="L$(PAPARAZZI_HOME)/sw/ext/opencv_bebop/install_arm/lib/opencv4/3rdparty"/>
    <flag name="LDFLAGS" value="llibprotobuf"/>
    <flag name="LDFLAGS" value="llibjpeg-turbo"/>
    <flag name="LDFLAGS" value="llibpng"/>
    <flag name="LDFLAGS" value="llibtiff"/>
    <flag name="LDFLAGS" value="llibopenjp2"/>
    <flag name="LDFLAGS" value="lzlib"/>
    <flag name="LDFLAGS" value="lade"/>
    <flag name="LDFLAGS" value="ldl"/>
    <flag name="LDFLAGS" value="lm"/>
    <flag name="LDFLAGS" value="lpthread"/>
    <flag name="LDFLAGS" value="lrt"/>
  </makefile>
  <makefile target="nps">
    <file name="opencv_example.cpp"/>
    <file name="opencv_image_functions.cpp"/>
    
    <flag name="CXXFLAGS" value="I$(PAPARAZZI_SRC)/sw/ext/opencv_bebop/install_pc/include/opencv4"/>
    
    <flag name="LDFLAGS" value="L$(PAPARAZZI_HOME)/sw/ext/opencv_bebop/install_pc/lib"/>
    <flag name="LDFLAGS" value="lopencv_world"/>
    <flag name="LDFLAGS" value="L$(PAPARAZZI_HOME)/sw/ext/opencv_bebop/install_pc/lib/opencv4/3rdparty"/>
    <flag name="LDFLAGS" value="llibprotobuf"/>
    <flag name="LDFLAGS" value="lade"/>
    <flag name="LDFLAGS" value="L/usr/lib/x86_64-linux-gnu"/>
    <flag name="LDFLAGS" value="ljpeg"/>
    <flag name="LDFLAGS" value="lpng"/>
    <flag name="LDFLAGS" value="ltiff"/>
    <flag name="LDFLAGS" value="lopenjp2"/>
    <flag name="LDFLAGS" value="L/usr/lib/x86_64-linux-gnu/hdf5/serial"/>
    <flag name="LDFLAGS" value="lhdf5"/>
    <flag name="LDFLAGS" value="lcrypto"/>
    <flag name="LDFLAGS" value="lcurl"/>
    <flag name="LDFLAGS" value="lpthread"/>
    <flag name="LDFLAGS" value="lsz"/>
    <flag name="LDFLAGS" value="lz"/>
    <flag name="LDFLAGS" value="ldl"/>
    <flag name="LDFLAGS" value="lm"/>
    <flag name="LDFLAGS" value="lfreetype"/>
    <flag name="LDFLAGS" value="lharfbuzz"/>
    <flag name="LDFLAGS" value="lrt"/>

  </makefile>
</module>
```

### Using OpenCV in C++ Code

Include the OpenCV headers in your C++ files:

```cpp
#include "opencv_image_functions.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
```

You can then use provided wrapper functions like `grayscale_opencv_to_yuv422()` in your code:

```cpp
// Convert and process images
Mat image = imread("image.jpg");
Mat gray;
cvtColor(image, gray, COLOR_BGR2GRAY);
// ... your vision processing
```

### Team Consistency

**Important:** Ensure all team members use the same build environment and configuration. Inconsistent builds can lead to compatibility issues during testing and deployment, leading to high dependency on one laptop/person.

---

## Troubleshooting

### Build Failures

- **Module not found:** Ensure you're running `make` from the paparazzi root or using `make -C sw/ext`
- **Dependency errors:** Check that required packages are installed (see individual module documentation)
- **Python build issues:** Some modules (mavlink, dronecan) require Python packages. See [Makefile](Makefile) for details on `MY_MAVLINKTOOLS` and `MY_DRONECANTOOLS`

### Submodule Issues

If a submodule is in a bad state, reinitialize it:

```bash
# Sync and reset a specific module
cd <paparazzi-root>
git submodule sync sw/ext/<module-name>
git submodule update --init --recursive sw/ext/<module-name>

# Or reset all modules
git submodule update --init --recursive
```

---
