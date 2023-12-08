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
cmake -B build -DCMAKE_BUILD_TYPE=Debug; cmake --build build -j 1

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