#include <iostream>
#include <optional>
#include <ormpp_ros/ormpp/dbng.hpp>
#include <ormpp_ros/ormpp/sqlite.hpp>
#include <vector>

using namespace ormpp;

// 定义 person 结构，也是数据库表结构
struct person
{
    int id;
    std::string name;
    std::optional<int> age;  // 可以插入 null 值
};
REGISTER_AUTO_KEY(person, id)  // 注册自增主键
YLT_REFL(person, id, name, age)

void PrintAllInDB()
{
    std::cout << "######### show all records in db" << std::endl;
    dbng<sqlite> db;
    try
    {
        db.connect("example.db");
        auto persons = db.query_s<person>();
        std::cout << "Total records: " << persons.size() << std::endl;
        for (const auto &person : persons)
        {
            std::cout << "ID: " << person.id << ", Name: " << person.name << ", Age: " << (person.age ? std::to_string(*person.age) : "NULL")
                      << std::endl;
        }
        std::cout << "######################" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void SetWal(dbng<sqlite> &db)
{
    std::cout << "Setting WAL mode for the database" << std::endl;
    if (db.execute("PRAGMA journal_mode=WAL;"))
    {
        std::cout << "WAL mode set successfully" << std::endl;
    }
    else
    {
        std::cerr << "Failed to set WAL mode" << std::endl;
    }
}

void InsertOP(dbng<sqlite> &db)
{
    person p1 = {0, "test1", 25};
    person p2 = {0, "test2", 30};
    person p3 = {0, "test3", {}};
    std::vector<person> v{p1, p2, p3};

    std::cout << "\n--- insert one record like: insert into 'person' (name,age) values('test1',25) ---" << std::endl;
    auto result = db.insert(p1);
    std::cout << "Inserted 1 record, result: " << result << std::endl;
    PrintAllInDB();

    std::cout << "\n--- insert multi records like: loop insert one record ---" << std::endl;
    result = db.insert(v);
    std::cout << "Inserted " << result << " records" << std::endl;
    PrintAllInDB();
}

auto QueryOP(dbng<sqlite> &db)
{
    std::cout << "\n--- query all like: select * from person ---" << std::endl;
    auto persons = db.query_s<person>();
    std::cout << "Total records: " << persons.size() << std::endl;
    for (const auto &person : persons)
    {
        std::cout << "ID: " << person.id << ", Name: " << person.name << ", Age: " << (person.age ? std::to_string(*person.age) : "NULL")
                  << std::endl;
    }

    std::cout << "\n--- query condition like: select * from person where name='test1'---" << std::endl;
    auto result_query = db.query_s<person>("name=?", "test1");
    for (const auto &person : result_query)
    {
        std::cout << "ID: " << person.id << ", Name: " << person.name << ", Age: " << (person.age ? std::to_string(*person.age) : "NULL")
                  << std::endl;
    }

    std::cout << "\n--- query condition like: select id, name from person ---" << std::endl;
    auto result = db.query_s<std::tuple<int, std::string>>("select id, name from person");
    std::cout << "Selected columns query result:" << std::endl;
    for (const auto &row : result)
    {
        std::cout << "ID: " << std::get<0>(row) << ", Name: " << std::get<1>(row) << std::endl;
    }
    return persons;
}

auto UpdateOP(dbng<sqlite> &db, std::vector<person> &persons)
{
    if (!persons.empty())
    {
        auto first_person = persons[0];
        first_person.age  = 99;
        first_person.name = "test4";

        std::cout << "Updated partial fields with id in query response like: update `person` set age=99 where id=" << first_person.id << std::endl;
        db.update_some<&person::age>(first_person);
        PrintAllInDB();

        std::cout << "\n--- Updated all fields with id in query response like: update `person` set id=" << first_person.id
                  << ",name=test4,age=100 where id=" << first_person.id << "---" << std::endl;
        first_person.age = 100;
        db.update(first_person);
        PrintAllInDB();

        std::cout << "\n--- Updated with custom condition like: update `person` set age=12 where name='test4' ---" << std::endl;
        person tmp = {0, "", 12};
        db.update_some<&person::age>(tmp, "name='test4'");
        PrintAllInDB();
    }
}

void DeleteOP(dbng<sqlite> &db)
{
    std::cout << "Deleted records like: delete from `person` where name=test2" << std::endl;
    db.delete_records_s<person>("name=?", "test2");
    PrintAllInDB();
}

void TransactionOP(dbng<sqlite> &db)
{
    std::cout << "\n--- Transaction begin" << std::endl;
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
    PrintAllInDB();
}

void RawSql(dbng<sqlite> &db)
{
    std::cout << "\n--- Raw SQL execution example" << std::endl;
    std::string sql = "INSERT INTO person (name, age) VALUES ('raw_sql', 42)";
    if (db.execute(sql))
    {
        std::cout << "Raw SQL executed successfully" << std::endl;
    }
    else
    {
        std::cerr << "Failed to execute raw SQL" << std::endl;
    }
    PrintAllInDB();
}

int main()
{
    std::cout << "=== ORMPP SQLite Basic Example ===" << std::endl;
    dbng<sqlite> db;

    try
    {
        db.connect("example.db");
        std::cout << "Connected to SQLite database: example.db" << std::endl;

        db.create_datatable<person>(ormpp_auto_key{"id"});
        std::cout << "Created table 'person' and set id as PRIMARY KEY AUTOINCREMENT" << std::endl;
        db.delete_records_s<person>();

        SetWal(db);
        InsertOP(db);
        auto persons = QueryOP(db);
        UpdateOP(db, persons);
        DeleteOP(db);
        TransactionOP(db);
        RawSql(db);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}