LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib
export LD_LIBRARY_PATH
ulimit -c unlimited
GLOG_minloglevel=0 ./build/trimule
