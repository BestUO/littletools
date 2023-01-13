#include <iostream>
#include "struct_pack/struct_pack.hpp"

void t()
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

int main()
{
    t();
    return 0;  
}