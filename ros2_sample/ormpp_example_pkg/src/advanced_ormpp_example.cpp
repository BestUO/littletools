#include <iostream>
#include <optional>
#include <ormpp_ros/ormpp/dbng.hpp>
#include <ormpp_ros/ormpp/sqlite.hpp>
#include <vector>

using namespace ormpp;

// 定义 person 结构，参考 README.md
struct person
{
    int id;
    std::string name;
    std::optional<int> age;  // 可以插入 null 值

    // 字段别名示例
    static constexpr auto get_alias_field_names(person*)
    {
        return std::array{ylt::reflection::field_alias_t{"person_id", 0}, ylt::reflection::field_alias_t{"person_name", 1},
                          ylt::reflection::field_alias_t{"person_age", 2}};
    }

    // 表名别名示例
    static constexpr std::string_view get_alias_struct_name(person*)
    {
        return "PERSON_TABLE";
    }
};
REGISTER_AUTO_KEY(person, id)
REGISTER_CONFLICT_KEY(person, name)
YLT_REFL(person, id, name, age)

// 定义 student 结构，演示冲突键
struct student
{
    int code;
    std::string name;
    char sex;
    int age;
    double dm;
    std::string classroom;
};
REGISTER_CONFLICT_KEY(student, code)
YLT_REFL(student, code, name, sex, age, dm, classroom)

// 枚举类型示例
enum class Color
{
    BLUE = 10,
    RED  = 15
};
enum Fruit
{
    APPLE,
    BANANA
};

struct test_enum_t
{
    int id;
    Color color;
    Fruit fruit;
};
REGISTER_AUTO_KEY(test_enum_t, id)
YLT_REFL(test_enum_t, id, color, fruit)

// AOP 切面示例
struct log
{
    template <typename... Args>
    bool before(Args... args)
    {
        std::cout << "[LOG] Before operation" << std::endl;
        return true;
    }

    template <typename T, typename... Args>
    bool after(T t, Args... args)
    {
        std::cout << "[LOG] After operation, result: " << t << std::endl;
        return true;
    }
};

struct validate
{
    template <typename... Args>
    bool before(Args... args)
    {
        std::cout << "[VALIDATE] Before validation" << std::endl;
        return true;
    }

    template <typename T, typename... Args>
    bool after(T t, Args... args)
    {
        std::cout << "[VALIDATE] After validation" << std::endl;
        return true;
    }
};

void demonstrateBasicOperations()
{
    std::cout << "\n=== 基本操作示例 ===" << std::endl;

    dbng<sqlite> db;
    db.connect("advanced_example.db");

    // 创建表
    db.create_datatable<person>(ormpp_auto_key{"id"});

    // 清空表
    db.delete_records_s<person>();

    // 插入数据
    person p1 = {0, "test1", 25};
    person p2 = {0, "test2", 30};
    person p3 = {0, "test3", {}};
    std::vector<person> v{p1, p2, p3};

    db.insert(p1);
    db.insert(v);

    // 查询数据(id=1)
    auto result = db.query_s<person>("id=?", 1);
    std::cout << "Query result size: " << result.size() << std::endl;

    // 获取插入后的自增id
    auto id1 = db.get_insert_id_after_insert<person>(p1);
    auto id2 = db.get_insert_id_after_insert<person>(v);
    std::cout << "Last insert ID for single: " << id1 << ", for batch: " << id2 << std::endl;

    // 更新数据
    if (!result.empty())
    {
        auto person_to_update = result[0];
        person_to_update.age  = 99;
        db.update(person_to_update);
        db.update(person_to_update, "id=1");
    }

    // 替换数据
    person p_replace = {1, "replaced", 88};
    db.replace(p_replace);

    // 查询所有数据
    auto all_persons = db.query_s<person>();
    for (const auto& person : all_persons)
    {
        std::cout << person.id << " " << person.name << " " << (person.age ? std::to_string(*person.age) : "NULL") << std::endl;
    }
}

void demonstrateTransaction()
{
    std::cout << "\n=== 事务处理示例 ===" << std::endl;

    dbng<sqlite> db;
    db.connect("transaction_example.db");
    db.create_datatable<person>(ormpp_auto_key{"id"});

    // 清空表
    db.delete_records_s<person>();

    // 事务示例
    db.begin();
    for (int i = 0; i < 5; ++i)
    {
        person s = {0, "tom_" + std::to_string(i), 19 + i};
        if (!db.insert(s))
        {
            std::cout << "Transaction failed, rolling back" << std::endl;
            db.rollback();
            return;
        }
    }
    db.commit();
    std::cout << "Transaction committed successfully" << std::endl;

    auto count = db.query_s<person>().size();
    std::cout << "Records after transaction: " << count << std::endl;
}

void demonstrateEnumTypes()
{
    std::cout << "\n=== 枚举类型示例 ===" << std::endl;

    dbng<sqlite> db;
    db.connect("enum_example.db");

    db.execute("drop table if exists test_enum_t");
    db.create_datatable<test_enum_t>(ormpp_auto_key{"id"});

    // 插入枚举数据
    test_enum_t enum_data{0, Color::BLUE, APPLE};
    db.insert(enum_data);

    auto vec1 = db.query_s<test_enum_t>();
    if (!vec1.empty())
    {
        std::cout << "Inserted enum data - Color: " << static_cast<int>(vec1.front().color) << ", Fruit: " << vec1.front().fruit << std::endl;

        // 更新枚举值
        vec1.front().color = Color::RED;
        db.update(vec1.front());

        auto vec2 = db.query_s<test_enum_t>();
        std::cout << "Updated enum data - Color: " << static_cast<int>(vec2.front().color) << std::endl;
    }
}

void demonstrateConflictKey()
{
    std::cout << "\n=== 冲突键示例 ===" << std::endl;

    dbng<sqlite> db;
    db.connect("conflict_example.db");

    db.execute("drop table if exists student");
    db.create_datatable<student>();

    // 插入学生数据
    student s1{1001, "Alice", 'F', 20, 85.5, "CS101"};
    student s2{1002, "Bob", 'M', 21, 90.0, "CS102"};

    db.insert(s1);
    db.insert(s2);

    // 使用冲突键更新 - 如果 code 相同则更新
    student s3{1001, "Alice Updated", 'F', 21, 88.0, "CS101"};
    db.replace(s3);  // 会根据 code 进行 replace

    auto students = db.query_s<student>();
    for (const auto& student : students)
    {
        std::cout << "Code: " << student.code << ", Name: " << student.name << ", Age: " << student.age << ", Score: " << student.dm << std::endl;
    }
}

void demonstrateSpecialQueries()
{
    std::cout << "\n=== 特殊查询示例 ===" << std::endl;

    dbng<sqlite> db;
    db.connect("query_example.db");
    db.create_datatable<person>(ormpp_auto_key{"id"});

    // 清空并插入测试数据
    db.delete_records_s<person>();
    for (int i = 1; i <= 5; ++i)
    {
        person p{0, "user" + std::to_string(i), 20 + i};
        db.insert(p);
    }

    // 特定列查询
    auto result = db.query_s<std::tuple<int, std::string>>("select id, name from person");
    std::cout << "Selected columns query result:" << std::endl;
    for (const auto& row : result)
    {
        std::cout << "ID: " << std::get<0>(row) << ", Name: " << std::get<1>(row) << std::endl;
    }

    // 聚合查询
    auto count_result = db.query_s<std::tuple<int>>("select count(1) from person");
    if (!count_result.empty())
    {
        std::cout << "Total count: " << std::get<0>(count_result[0]) << std::endl;
    }

    // 条件查询
    auto filtered = db.query_s<person>("age > ?", 22);
    std::cout << "Persons older than 22: " << filtered.size() << std::endl;
}

void demonstrateAOP()
{
    std::cout << "\n=== AOP 面向切面编程示例 ===" << std::endl;

    dbng<sqlite> db;

    // 简单的连接示例（AOP 功能可能需要特定版本支持）
    try
    {
        db.connect("aop_example.db");
        std::cout << "Connected to database successfully" << std::endl;

        // 创建简单表进行演示
        db.create_datatable<person>(ormpp_auto_key{"id"});

        // 插入一条记录
        person p{0, "aop_test", 25};
        auto result = db.insert(p);

        std::cout << "AOP example - inserted record with result: " << result << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << "AOP example error: " << e.what() << std::endl;
    }
}

int main()
{
    std::cout << "=== ORMPP SQLite Advanced Example ===" << std::endl;

    try
    {
        demonstrateBasicOperations();
        demonstrateTransaction();
        demonstrateEnumTypes();
        demonstrateConflictKey();
        demonstrateSpecialQueries();
        demonstrateAOP();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "\n=== All advanced examples completed successfully ===" << std::endl;
    return 0;
}