#include "coro_rpc/coro_rpc_server.hpp"
#include "coro_http/coro_http_client.h"

async_simple::coro::Lazy<void> get_post(cinatra::coro_http_client &client) {
  std::string uri = "http://www.example.com";
  auto result = co_await client.async_get(uri);
  std::cout << result.status << "\n";
  
  result = co_await client.async_post(uri, "hello", ylt::req_content_type::string);
  std::cout << result.status << "\n";
}

void core_rpc_asynic_client_test()
{
    cinatra::coro_http_client client{};
    async_simple::coro::syncAwait(get_post(client));
}

inline std::string_view echo(std::string_view str) 
{ 
    return str; 
}

void core_rpc_server_test()
{
    coro_rpc::coro_rpc_server server(/*thread_num =*/10, /*port =*/9000);
    server.register_handler<echo>(); // register function echo
    auto err = server.start(); // start the server & block
}

async_simple::coro::Lazy<int> test1(int i,int j)
{
    co_return i+j;
}

async_simple::coro::Lazy<int> test2(int i,int j)
{
    auto result = co_await test1(i,j);
    co_return result;
}