#include"thirdso.h"
#include <string.h>
#include <unistd.h>

bool thirdso::mystrcmp(std::string s)
{
    sleep(1);
    return true;
};