#ifndef DATACACHE_H
#define DATACACHE_H
#include "CallBack.h"
#include "../../dbstruct/dbstruct.h"
class DataCache:public CallBackManage{

public: 

    void PollingQueue();
    

};

#endif