#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/easylog/easylog.hpp"

TEST_CASE("easylog_")
{
    std::string filename = "easylog.txt";
    std::filesystem::remove(filename);
    easylog::init_log(
        easylog::Severity::DEBUG, filename, false, true, 5000, 1, true);
    ELOGFMT(INFO, "it is a long string test {} {}", 42, "fmt");
}