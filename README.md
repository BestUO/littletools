# littletools
C++17 for Ringqueue，timemanager，threadpool，Rdkafka，rpccore

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
##Trimule Run in docker