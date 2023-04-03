#include <iostream>
#include "coro_rpc/coro_rpc_client.hpp"
#include "testhun.hpp"

async_simple::coro::Lazy<void> core_rpc_client() 
{
    coro_rpc::coro_rpc_client client;
    co_await client.connect("localhost", /*port =*/"9000"); // connect to the server

    auto r = co_await client.call<echo>("hello coro_rpc"); // call remote function echo
    std::cout << r.value() << "\n"; //will print "hello coro_rpc"
}

int main()
{
    syncAwait(core_rpc_client());
    return 0;  
}