# littletools

A collection of C++ experiments, utilities, and benchmarks. The repository
covers concurrent containers, coroutine examples, networking utilities,
memory management, logging, and system-level helpers.

## Contents

- DPDK `RTE_Ring` and a C++17 free-lock queue implementation
- CommandCenter synchronous and asynchronous wrappers
- Ceph-style hash helpers, alignment utilities, and CPU binding
- Memory pools, thread pools, timers, and an LRU cache
- C++20 coroutine, Asio, hook, and RPC examples

## Prerequisites

- A C++20-capable compiler
- CMake 3.8 or later
- Git, including submodule support
- `make` for the bundled `liburing` build
- Python development headers and `pybind11` for the `call_python` example

## Build

Initialize third-party dependencies and configure an out-of-source Release
build:

```sh
git submodule update --init --recursive --depth 1
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 1
```

The top-level build currently includes the tools test suite, io_uring examples
and benchmark, coroutine examples, the C++-to-Python `call_python` example,
yalanting examples, Asio examples, and hook tests. Other directories listed in
the root `CMakeLists.txt` remain disabled.

## Test Suite

`doctooltest` supports filtering by test-case name:

```sh
# Network tests and benchmarks
./build/tools_test/doctooltest --test-case='*network_*'

# Object-pool tests and benchmarks
./build/tools_test/doctooltest --test-case='*ObjectPool_*'

# Reliable UDP tests and benchmarks
./build/tools_test/doctooltest --test-case='*ReliableUDP_*'

# Serialization tests and benchmarks
./build/tools_test/doctooltest --test-case='*serialize_*'

# Raft leader-election tests
./build/tools_test/doctooltest --test-case='*Raft_*'

# Shared-memory component tests
./build/tools_test/doctooltest --test-case='*shm_*'

# Coroutine tests
./build/tools_test/doctooltest --test-case='*coroutine_*'

# timer manager tests
./build/tools_test/doctooltest --test-case='*TimerManager*'
```

## io_uring Logging Benchmark

The top-level CMake build configures and installs the bundled `liburing` into
`build/liburing/install`; no manual in-tree build is required. Build and run
the EasyLog benchmark with:

```sh
cmake --build build --target uring_log_benchmark
./build/uring_log/uring_log_benchmark
```

### RK3588 Reference Result

The following result writes 2,500,000 messages across five threads, with each
message approximately 80 bytes:

```text
spdlog synchronous system write:       2315607969 ns
easylog synchronous system write:      1359331896 ns
easylog asynchronous system write:      337628465 ns
easylog synchronous io_uring write:    1073387513 ns
easylog asynchronous io_uring write:    279847519 ns
```

## Python Interoperability Example

The C++-to-Python example is included in the top-level build. Build and run it
with:

```sh
cmake --build build --target call_python
./build/call_python/call_python
```

To build it independently, use the CMake project in `call_python/` instead.

## v6 Timer Manager

`timermanager::v6::TimerManager` uses a `steady_clock` deadline min-heap. Each
heap entry owns one timer, and the worker waits for the earliest heap deadline
rather than polling at a fixed interval.

```cpp
auto& timer_manager = timermanager::v6::TimerManager::GetInstance();
const auto id = timer_manager.AddTimer(
    std::chrono::milliseconds(10), [] { /* callback */ },
    std::chrono::milliseconds(100));
timer_manager.CancelTimer(id);
```

Set `interval` to zero for a one-shot timer. A repeating callback may cancel
itself with its returned `TimerId`; a cancelled timer is not rescheduled.

## TCMalloc Sample

`tcmalloc_sample` shows how to use tcmalloc and check memory leak.install pprof first
```sh
export GOPROXY=https://goproxy.cn,direct
go install github.com/google/pprof@latest

```

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gperftools_all tcmalloc_sample
MALLOCSTATS=2 ./build/tcmalloc_sample/tcmalloc_sample
HEAPPROFILE=/tmp/leak_test HEAP_PROFILE_TIME_INTERVAL=5 ./build/tcmalloc_sample/tcmalloc_sample
```

if your progress use glibc's ptmalloc, run follow to use tcmalloc:
```sh
sudo apt install -y autoconf automake libtool
git clone https://github.com/gperftools/gperftools.git
cd gperftools
./autogen.sh
mkdir build && cd build
../configure --prefix=${PWD}/../install CXXFLAGS="-O3" CFLAGS="-O3"
make -j4
make install

LD_PRELOAD=/path/to/libtcmalloc.so HEAPPROFILE=/tmp/leak_test HEAP_PROFILE_TIME_INTERVAL=5 ./your_progress
```

and then, check memory leak
```sh
export PATH=$PATH:$(go env GOPATH)/bin
pprof --base=/tmp/leak_test.0002.heap \
  --text \
  ./your_progress \
  /tmp/leak_test.0005.heap

pprof --text ./your_progress /tmp/leak_test.0001.heap
pprof --text ./your_progress /tmp/leak_test.0005.heap
```

## Standalone Applications

These applications require MySQL support and are not currently enabled by the
top-level CMake build.

### Trimule

An HTTP server that receives call information and stores it in a database.
Requires C++17.

```sh
cd Trimule
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target trimule -j 2
cmake --build build --target trimule_copyfile
```

To run with Docker:

```sh
cd Trimule
sh docker_build.sh
sh docker_run.sh
```

To change the port, update `docker_run.sh` and `conf/trimule_config.json`.

### Dialogmanager

An HTTP server for managing dialog sessions and storing them in a database.
Requires C++17.

```sh
cd Dialogmanager
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target dialogmanager -j 1
```

Configuration: `conf/dialog_manager_config.json`.

## Planned Work

- Optimize `timermanager` with a map-and-list design
- Add a work-pool/object-pool path for large-message receiving
- Support multi-message sending with reliable UDP
- Decouple flow control from received-message counts
