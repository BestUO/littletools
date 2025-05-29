#pragma once
#include <iostream>

class SampleServer
{
public:
    SampleServer()  = default;
    ~SampleServer() = default;

    void run()
    {
        std::cout << "SampleServer is running!" << std::endl;
        // Add server logic here
    }
};