#ifndef CALL_H
#define CALL_H

#include <iostream>
#include <string>
#include <json/json.h>
#include "../dbstruct/dbstruct.h"
class CallRecord{
    public:
      
 CallInfo GetCallRecord(std::string s,int framework_class);
 
 };

 #endif