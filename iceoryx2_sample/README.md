# iceoryx2 C++ samples

This directory contains two IPC examples built against the vendored
`3rdparty/iceoryx2` source tree:

- publish/subscribe: `iox2_publisher` and `iox2_subscriber`
- request/response: `iox2_client` and `iox2_server`

The examples can be built with the complete littletools project, or on their
own. CMake 3.22 or newer, a C++20 compiler, Rust and Cargo are required.

```sh
cmake -S iceoryx2_sample -B build-iceoryx2 -DCMAKE_BUILD_TYPE=Release
cmake --build build-iceoryx2 --target \
  iox2_publisher iox2_subscriber iox2_client iox2_server -j 4
```

Run each pair in two terminals. Start order does not matter:

```sh
# Publish/subscribe
./build-iceoryx2/iox2_subscriber
./build-iceoryx2/iox2_publisher

# Request/response
./build-iceoryx2/iox2_server
./build-iceoryx2/iox2_client
```

Press Ctrl+C to stop a process cleanly. Set `IOX2_LOG_LEVEL` to change the
iceoryx2 log verbosity.
