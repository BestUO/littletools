littletools usage in tooltest
====
* DPDK RTE_Ring
* simple queue wrap
* priority_queue support add or remove
* RdKafka::Producer and RdKafka::Producer
* qicosmos/cinatra
* RWSeparate in cinatra with kafka
* CommandCenter asyncwrap and syncwrap
* CephHashFun JenkinsHashFun public_align32pow2 cpu_bind
* memory pool
* timemanager
* freelockqueue c++17 implement from RTE_Ring
* threadpool
* c++20 coroutines test
* hooktest
* funregister for rpc
* asio example
* Simple LRU_Cache

# ToolExample
git submodule update --init
cmake -B build -DCMAKE_BUILD_TYPE=Debug;  
cmake --build build -j 1 

## network func_test & bench
./build/examples/doctooltest --test-case=*network_*

## object_pool func_test & bench
./build/examples/doctooltest --test-case=*ObjectPool_*

## reliable udp func_test & bench
./build/examples/doctooltest --test-case=*ReliableUDP_*

## serialize tool func_test & bench
./build/examples/doctooltest --test-case=*serialize_*

## timemanager func_test
./build/examples/doctooltest --test-case=*TimerManager_*

## raft_pick_leader func_test
./build/examples/doctooltest --test-case=*Raft_*

## shared memory component func_test
./build/examples/doctooltest --test-case=*shm_*

## coroutine func_test
./build/examples/doctooltest --test-case=*coroutine_*

## to do
list+object_pool
large_msg_recv workpool
optimize timermanager
multi_msg_send with rudp



# Two Projects
need mysql support

## Trimule
Httpserver for receive call info and store in db
### require 
c++17
### Make
mkdir build && cd build  
cmake -DCMAKE_BUILD_TYPE=Debug ..; cmake --build . --target trimule -j 2;make trimule_copyfile
### Run in Docker
//if you want chang port, change docker_run.sh and conf/trimule_config.json  
cd Trimule  
sh docker_build.sh  
sh docker_run.sh  

## Dialogmanager
Httpserver for manage dialog session and store info in db
### require 
c++17
### Make
mkdir build && cd build  
cmake -DCMAKE_BUILD_TYPE=Debug ..; cmake --build . --target dialogmanager -j 1
### Config
conf/dialog_manager_config.json 