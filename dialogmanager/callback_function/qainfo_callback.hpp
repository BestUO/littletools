#pragma once
#include "global.h"

class QAInfoCallBackFunction
{
public:
    static void StoreInDB(std::shared_ptr<QAInfo> qainfo)
    {
        //打分、数据落盘
        LOGGER->info("QAInfo落盘 course_id:{} session_id:{} node_id:{}", 
                    qainfo->course_id, qainfo->session_id, qainfo->current_node.lock()->node_id);
    }
};