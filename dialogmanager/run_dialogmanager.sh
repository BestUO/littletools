#!/bin/bash
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./lib
export LD_LIBRARY_PATH
ulimit -c unlimited
./dialogmanager