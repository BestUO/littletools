#include "global.h"
#include "dboperate/dboperate.hpp"
#include <iostream>

std::vector<std::string> split(const std::string& str, const std::string& delims = " ")
{
    std::vector<std::string> output;
    auto first = std::cbegin(str);

    while (first != std::cend(str))
    {
        const auto second = std::find_first_of(first, std::cend(str), std::cbegin(delims), std::cend(delims));

        if (first != second)
            output.emplace_back(first, second);

        if (second == std::cend(str))
            break;

        first = std::next(second);
    }

    return output;
}

Session::~Session()
{
    int end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    DBOperate::GetInstance()->CoursePratiseUpdate(end_time-start_time, end_time, session_id);
}