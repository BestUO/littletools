#include <iostream>
#include "struct_pack/struct_pack.hpp"
#include "coro_rpc/coro_rpc_server.hpp"
#include "coro_rpc/coro_rpc_client.hpp"
#include "coro_http/coro_http_client.h"

void struct_pack_test()
{
    struct person 
    {
        int64_t id;
        std::string name;
        int age;
        double salary;
    };
    person person1{.id = 1, .name = "hello struct pack", .age = 20, .salary = 1024.42};
    auto result = struct_pack::serialize<std::string>(person1);
    std::cout << result << std::endl;

    std::string result2="The next line is struct_pack serialize result.\n";
    struct_pack::serialize_to(result2,person1);
    std::cout << result2 << std::endl;

    // auto result3 = struct_pack::serialize(person1.id, person1.name, person1.age, person1.salary);

    auto person2 = struct_pack::deserialize<person>(result);
    person person3;
    auto ec = struct_pack::deserialize_to(person3, result);
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

async_simple::coro::Lazy<void> core_rpc_client() 
{
    coro_rpc::coro_rpc_client client;
    co_await client.connect("localhost", /*port =*/"9000"); // connect to the server

    auto r = co_await client.call<echo>("hello coro_rpc"); // call remote function echo
    std::cout << r.value() << "\n"; //will print "hello coro_rpc"
}

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

int main()
{
    // struct_pack_test();
    core_rpc_server_test();
    // syncAwait(core_rpc_client());
    // core_rpc_asynic_client_test();
    return 0;  
}