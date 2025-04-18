# http_sample

## depends
* c++20
* libcurl

## build
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build/

## run http_client
./build/download_file <URL> </path/to/files> [thread_num], example:  
./build/download_file https://releases.ubuntu.com/22.04/ubuntu-22.04.5-live-server-amd64.iso.torrent tmpfile 4

## check
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.5-live-server-amd64.iso.torrent
md5sum ubuntu-22.04.5-live-server-amd64.iso.torrent
md5sum tmpfile