# Stream Motion Usage Guide

## Overview

The `StreamMotionConnection` class in the `stream_motion` namespace provides
an interface for communicating with a robot controller over a UDP socket.
It implements the communication protocol described in tables 3.3 "Stream
Motion OPERATOR'S MANUAL".
The packet structures are defined in the header file:
[packets.hpp](stream_motion/include/stream_motion/packets.hpp)

## Requirements

- C++17
- Dependencies:
  - sockpp
  - gtest

### Third-party sources

The third-party libraries built by this package (`sockpp`, `readerwriterqueue`,
`yaml-cpp`, `reflect-cpp`) are no longer vendored as git submodules. They are
declared in the top-level [`dep.repos`](../dep.repos) file and must be imported
with `vcs2l` (the maintained successor to vcstool; `apt install python3-vcs2l`)
*before* building, so no cloning happens during the build:

```bash
# from the root of the fanuc_driver repository
vcs import . < dep.repos
```

When built inside the wider ROS 2 workspace this is handled automatically by the
devcontainer setup.

## Basic Usage

### Add to your CMake project

```cmake
find_package(stream_motion REQUIRED)
# ...
target_link_libraries(stream_motion_test PRIVATE stream_motion)
```

### Include in your source code

```cpp
#include "stream_motion/stream.hpp"

// ...
// Instantiating the connection with the robot controller's IP address and port
stream_motion::StreamMotionConnection connection(ip_address, port)
```

The public interface for the `StreamMotionConnection` class can be found
[stream.hpp](stream_motion/include/stream_motion/stream.hpp).

## Example Code

Build the example code with the following commands:

```bash
cmake -Bbuild -S. -DBUILD_EXAMPLES=ON
cmake --build build --target all -j16
```

Note, you will need Eigen3 installed on your system to compile the example
code.

In order to run [stream_motion_example.cpp](examples/stream_motion_example.cpp),
you will need to have the following TP program running on the robot controller:

```none
IBGN start[1]
IBGN end[1]
```

Optionally, you can may want to add a move command before the start command.
This can be done with the `+Point` button in the editor of the TP program.

```none
J @P[1] 100% FINE
IBGN start[1]
IBGN end[1]
```

In the example, the robot is assumed to be on 192.168.1.100.
Try pinging the address before running the example.
The example with move the robot along a cosine wave trajectory with a
duration of 5 seconds.

Run the example with the following command:

```bash
build/examples/stream_motion_example
```
