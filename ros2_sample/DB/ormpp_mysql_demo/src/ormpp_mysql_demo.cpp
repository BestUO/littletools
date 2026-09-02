#include "ormpp_ros/ormpp/dbng.hpp"
#include "ormpp_ros/ormpp/mysql.hpp"
#include <iostream>
#include <optional>

using namespace ormpp;

struct person {
    int id;
    std::string name;
    std::optional<int> age;
};
REGISTER_AUTO_KEY(person, id)
YLT_REFL(person, id, name, age)

int main() {
    try {
        std::cout << "=== ORMPP MySQL Example ===" << std::endl;
        dbng<mysql> db;

        // Try different connection methods
        bool connected = false;

        // Method 2: Try connecting with password
        std::cout << "Trying to connect with password..." << std::endl;
        if (db.connect("localhost", "root", "password", "mysql", std::nullopt, 3306)) {
            std::cout << "Connected with password!" << std::endl;
            connected = true;
        }

        if (!connected) {
            std::cerr << "Failed to connect to MySQL database with any method" << std::endl;
            return -1;
        }

        std::cout << "Connected to MySQL database" << std::endl;

        // Create database if not exists (use mysql database for this demo)
        db.execute("CREATE DATABASE IF NOT EXISTS demo_db");
        db.execute("USE demo_db");

        // Create table
        db.create_datatable<person>(ormpp_auto_key{"id"});
        std::cout << "Created table 'person'" << std::endl;

        // Clear existing data
        db.delete_records_s<person>();

        // Insert some data
        person p1 = {0, "Alice", 30};
        person p2 = {0, "Bob", 25};
        person p3 = {0, "Charlie", {}};

        db.insert(p1);
        db.insert(p2);
        db.insert(p3);

        std::cout << "Inserted 3 records" << std::endl;

        // Query all records
        auto persons = db.query_s<person>();
        std::cout << "Total records: " << persons.size() << std::endl;

        for (const auto &person : persons) {
            std::cout << "ID: " << person.id << ", Name: " << person.name
                      << ", Age: " << (person.age ? std::to_string(*person.age) : "NULL") << std::endl;
        }

    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
