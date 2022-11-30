## littletools
C++17 for Ringqueue，timemanager，threadpool，Rdkafka，rpccore

## Dialogmanager
Httpserver for manage dialog session and store info in db
### require 
c++17
### Make
mkdir build && cd build  
cmake -DCMAKE_BUILD_TYPE=Debug ..; cmake --build . --target dialogmanager -j 1
### Config
conf/dialog_manager_config.json 

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