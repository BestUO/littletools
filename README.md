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

# Trimule
use cinatra+ormpp+freelockqueue for httpserver and dbclient.  
**very fast**

## HowtoRunTrimule
### Make
mkdir build && cd build  
cmake .. && make  
### Run in Docker
//if you want chang port, change docker_run.sh and conf/config.json  
cd Trimule  
sh docker_build.sh  
sh docker_run.sh  
