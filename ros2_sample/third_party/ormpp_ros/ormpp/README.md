<p align="center">
<a href="https://github.com/qicosmos/ormpp/actions/workflows/ci-sqlite.yml">
<img alt="ci-sqlite" src="https://github.com/qicosmos/ormpp/actions/workflows/ci-sqlite.yml/badge.svg?branch=master">
</a>
<a href="https://github.com/qicosmos/ormpp/actions/workflows/ci-mysql.yml">
<img alt="ci-mysql" src="https://github.com/qicosmos/ormpp/actions/workflows/ci-mysql.yml/badge.svg?branch=master">
</a>
<a href="https://github.com/qicosmos/ormpp/actions/workflows/ci-pgsql.yml">
<img alt="ci-pgsql" src="https://github.com/qicosmos/ormpp/actions/workflows/ci-pgsql.yml/badge.svg?branch=master">
</a>
<a href="https://codecov.io/gh/qicosmos/ormpp">
<img alt="codecov" src="https://codecov.io/gh/qicosmos/ormpp/branch/master/graph/badge.svg">
</a>
<img alt="language" src="https://img.shields.io/github/languages/top/qicosmos/ormpp?style=flat-square">
<img alt="last commit" src="https://img.shields.io/github/last-commit/qicosmos/ormpp?style=flat-square">
</p>

<p align="center">
  <a href="https://github.com/qicosmos/ormpp/tree/master/lang/english/README.md">English</a> | <span>中文</span>
</p>

# 一个很酷的Modern C++ ORM库----ormpp

iguana版本1.0.9
https://github.com/qicosmos/iguana.git

[谁在用ormpp](https://github.com/qicosmos/ormpp/wiki), 也希望ormpp用户帮助编辑用户列表，也是为了让更多用户把ormpp用起来，也是对ormpp 最大的支持，用户列表的用户问题会优先处理。

## 目录

* [ormpp的目标](#ormpp的目标)
* [ormpp的特点](#ormpp的特点)
* [快速示例](#快速示例)
* [如何编译](#如何编译)
* [接口介绍](#接口介绍)
* [roadmap](#roadmap)
* [联系方式](#联系方式)

## ormpp的目标
ormpp最重要的目标就是让c++中的数据库编程变得简单，为用户提供统一的接口，支持多种数据库，降低用户使用数据库的难度。

## ormpp的特点
ormpp是modern c++(c++11/14/17)开发的ORM库，目前支持了三种数据库：mysql, postgresql和sqlite，ormpp主要有以下几个特点：

1. header only
1. cross platform
1. unified interface
1. easy to use
1. easy to change database

你通过ormpp可以很容易地实现数据库的各种操作了，大部情况下甚至都不需要写sql语句。ormpp是基于编译期反射的，会帮你实现自动化的实体映射，你再也不用写对象到数据表相互赋值的繁琐易出错的代码了，更酷的是你可以很方便地切换数据库，如果需要从mysql切换到postgresql或sqlite只需要修改一下数据库类型就可以了，无需修改其他代码。

## 自增主键

使用REGISTER_AUTO_KEY注册自增主键

```C++
struct person {
  std::string name;
  int age;
  int id;
};
REGISTER_AUTO_KEY(person, id)
YLT_REFL(person, id, name, age)
```

## 冲突主键

使用REGISTER_CONFLICT_KEY注册冲突主键来进行update，如果未注册冲突主键则会采用自增主键

```C++
struct student {
  int code;
  std::string name;
  char sex;
  int age;
  double dm;
  std::string classroom;
};
REGISTER_CONFLICT_KEY(student, code)
YLT_REFL(student, code, name, sex, age, dm, classroom)
```

## 快速示例

这个例子展示如何使用ormpp实现数据库的增删改查之类的操作，无需写sql语句。

```C++
#include "dbng.hpp"
#include "mysql.hpp"//注意，使用什么数据库时就需要include对应的hpp文件，里面是对相关函数的反射封装
//#include "sqlite.hpp" //例如使用sqlite时，则包含sqlite.hpp
using namespace ormpp;

struct person {
  std::optional<int> age; // 可以插入null值
  std::string name;
  int id;
  static constexpr auto get_alias_field_names(alias *) {
    return std::array{ylt::reflection::field_alias_t{"person_id", 0},
                      ylt::reflection::field_alias_t{"person_name", 1},
                      ylt::reflection::field_alias_t{"person_age", 2}}; // 注意: 这里需与YLT_REFL的注册顺序一致
  }
  static constexpr std::string_view get_alias_struct_name(student *) {
    return "CUSTOM_TABLE_NAME"; // 表名默认结构体名字(person), 这里可以修改表名
  }
};
REGISTER_AUTO_KEY(person, id)
REGISTER_CONFLICT_KEY(person, name)
// REGISTER_CONFLICT_KEY(person, name, age) // 如果是多个
YLT_REFL(person, id, name, age)

int main() {
  person p = {"test1", 2};
  person p1 = {"test2", 3};
  person p2 = {"test3", 4};
  std::vector<person> v{p1, p2};

  dbng<mysql> mysql;
  mysql.connect("127.0.0.1", "dbuser", "yourpwd", "testdb");
  mysql.create_datatable<person>(ormpp_auto_key{"id"});

  // 插入数据
  mysql.insert(p);
  mysql.insert(v);

  // 查询数据(id=1)
  auto result = mysql.query_s<person>("id=?", 1);

  // 获取插入后的自增id
  auto id1 = mysql.get_insert_id_after_insert<person>(p);
  auto id2 = mysql.get_insert_id_after_insert<person>(v);

  // 更新数据
  mysql.update(p);
  mysql.update(v);
  mysql.update(p, "id=1");

  // 替换数据
  mysql.replace(p);
  mysql.replace(v);

  // 更新指定字段
  // mysql.update_some<&person::name, &person::age>(p);
  // mysql.update_some<&person::name, &person::age>(v);

  auto result = mysql.query_s<person>();
  for (auto &person : result) {
    std::cout << person.id << " " << person.name << " " << person.age
              << std::endl;
  }

  mysql.delete_records<person>();

  // transaction
  mysql.begin();
  for (int i = 0; i < 10; ++i) {
    person s = {"tom", 19};
    if (!mysql.insert(s)) {
      mysql.rollback();
      return -1;
    }
  }
  mysql.commit();
  return 0;
}
```

```C++
enum class Color { BLUE = 10, RED = 15 };
enum Fruit { APPLE, BANANA };

struct test_enum_t {
  Color color;
  Fruit fruit;
  int id;
};
REGISTER_AUTO_KEY(test_enum_t, id)
YLT_REFL(test_enum_t, id, color, fruit)

int main() {
  dbng<sqlite> sqlite;
  sqlite.connect(db);//或者开启sqcipher后sqlite.connect(db， password);
  sqlite.execute("drop table if exists test_enum_t");
  sqlite.create_datatable<test_enum_t>(ormpp_auto_key{"id"});
  sqlite.insert<test_enum_t>({Color::BLUE});
  auto vec1 = sqlite.query<test_enum_t>();
  vec1.front().color = Color::RED;
  sqlite.update(vec1.front());
  auto vec2 = sqlite.query<test_enum_t>();
  sqlite.update<test_enum_t>({Color::BLUE, BANANA, 1}, "id=1");
  auto vec3 = sqlite.query<test_enum_t>();
  vec3.front().color = Color::RED;
  sqlite.replace(vec3.front());
  auto vec4 = sqlite.query<test_enum_t>();
  sqlite.delete_records<test_enum_t>();
  auto vec5 = sqlite.query<test_enum_t>();
  return 0;
}
```

## 如何编译

支持的选项如下:
	1.	ENABLE_SQLITE3
	2.	ENABLE_MYSQL
	3.	ENABLE_PG

cmake -B build -DENABLE_SQLITE3=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug

## 作为第三方库引入

mysql
```cmake
set(ENABLE_MYSQL ON)
add_definitions(-DORMPP_ENABLE_MYSQL)
add_subdirectory(ormpp)
```
或者
```cmake
set(ENABLE_MYSQL ON)
add_definitions(-DORMPP_ENABLE_MYSQL)
add_library(ormpp INTERFACE)
include(ormpp/cmake/mysql.cmake)
target_link_libraries(ormpp INTERFACE ${MYSQL_LIBRARY})
target_include_directories(ormpp INTERFACE ormpp ormpp/ormpp ${MYSQL_INCLUDE_DIR})
```

sqlite
```cmake
set(ENABLE_SQLITE3 ON)
add_definitions(-DORMPP_ENABLE_SQLITE3)
add_subdirectory(ormpp)
```
或者
```cmake
set(ENABLE_SQLITE3 ON)
add_definitions(-DORMPP_ENABLE_SQLITE3)
add_subdirectory(ormpp/thirdparty)
add_library(ormpp INTERFACE)
target_link_libraries(ormpp INTERFACE sqlite3)
target_include_directories(ormpp INTERFACE ormpp ormpp/ormpp ormpp/thirdparty/sqlite3)
```

如果需要开启sqlcipher

因为sqlcipher需要链接到OpenSSL库，需要在本地先安装OpenSSL，然后添加下述cmake语句
```cmake
set(ENABLE_SQLITE3 ON)
set(ENABLE_SQLITE3_CODEC)#开启sqlcipher需要同时开启ENABLE_SQLITE3和ENABLE_SQLITE3_CODEC
add_definitions(
  -DORMPP_ENABLE_SQLITE3 
  -DSQLITE_HAS_CODEC
)
add_subdirectory(ormpp)
```
或者
```cmake
set(ENABLE_SQLITE3 ON)
set(ENABLE_SQLITE3_CODEC)#开启sqlcipher需要同时开启ENABLE_SQLITE3和ENABLE_SQLITE3_CODEC
add_definitions(
  -DORMPP_ENABLE_SQLITE3 
  -DSQLITE_HAS_CODEC
)
add_subdirectory(ormpp/thirdparty)
add_library(ormpp INTERFACE)
target_link_libraries(ormpp INTERFACE sqlite3)
target_include_directories(ormpp INTERFACE ormpp ormpp/ormpp ormpp/thirdparty/sqlite3)
```
注意：最终发布的应用程序依赖于 OpenSSL 的动态库，
- Windows 下通常为 `libssl-3-x64.dll`和 `libcrypto-3-x64.dll`；  
  请将这两个 `.dll` 拷贝到可执行文件所在目录。  
- Linux下通常已安装在 `/usr/lib` 或 `/usr/local/lib`，不需要拷贝；  
  如果你使用自编译的 OpenSSL 或者希望随同可执行文件一起分发，也可以将 `libssl.so` 与 `libcrypto.so` 放在执行目录，并通过 `LD_LIBRARY_PATH` 或者 `-Wl,-rpath` 指定查找路径。
- macOS 下通常为 `libssl.dylib` 和 `libcrypto.dylib`；
  请将这两个 `.dylib` 拷贝到可执行文件所在目录，或通过 `DYLD_LIBRARY_PATH`、Mach-O `@rpath` 设置查找路径。  

pg
```cmake
set(ENABLE_PG ON)
add_definitions(-DORMPP_ENABLE_PG)
add_subdirectory(ormpp)
```
或者
```cmake
set(ENABLE_PG ON)
add_definitions(-DORMPP_ENABLE_PG)
add_library(ormpp INTERFACE)
include(ormpp/cmake/pgsql.cmake)
target_link_libraries(ormpp INTERFACE ${PGSQL_LIBRARY})
target_include_directories(ormpp INTERFACE ormpp ormpp/ormpp ${PGSQL_INCLUDE_DIR})
```

### 编译器支持

需要支持C++17的编译器, 要求的编译器版本：linux gcc7.2, clang4.0; windows >vs2017 update5

### 数据库的安装

因为ormpp支持mysql和postgresql，所以需要安装mysql，postgresql，postgresql官方提供的libpq，安装之后，在CMakeLists.txt配置目录和库路径(默认安装不需要)。

## 接口介绍
ormpp屏蔽了不同数据库操作接口的差异，提供了统一简单的数据库操作接口，具体提供了数据库连接、断开连接、创建数据表、插入数据、更新数据、删除数据、查询数据和事务相关的接口。

### 接口概览

```c++
//连接数据库
template <typename... Args>
bool connect(Args&&... args);

//断开数据库连接
bool disconnect();

//创建数据表
template<typename T, typename... Args>
bool create_datatable(Args&&... args);

//插入单条数据
template<typename T, typename... Args>
int insert(const T& t, Args&&... args);

//插入多条数据
template<typename T, typename... Args>
int insert(const std::vector<T>& t, Args&&... args);

//替换单条数据
template <typename T, typename... Args>
int replace(const T &t, Args &&...args);

//替换多条数据
template <typename T, typename... Args>
int replace(const std::vector<T> &v, Args &&...args);

//更新单条数据
template<typename T, typename... Args>
int update(const T& t, Args&&... args);

//更新多条数据
template<typename T, typename... Args>
int update(const std::vector<T>& t, Args&&... args);

//更新单条数据(指定字段)
template <auto... members, typename T, typename... Args>
int update_some(const T &t, Args &&...args);

//更新多条数据(指定字段)
template <auto... members, typename T, typename... Args>
int update_some(const std::vector<T> &v, Args &&...args);

//获取插入后的自增id
template <typename T, typename... Args>
uint64_t get_insert_id_after_insert(const T &t, Args &&...args);

//删除数据(带预处理)
template <typename T, typename... Args>
int delete_records_s(const std::string &str = "", Args &&...args);

//查询数据，包括单表查询和多表查询(带预处理)
template <typename T, typename... Args>
std::vector<T> query_s(const std::string &str = "", Args &&...args);

//删除数据(不带预处理)
template <typename T, typename... Args>
[[deprecated]] bool delete_records(Args &&...where_condition)

//查询数据，包括单表查询和多表查询(不带预处理)
template <typename T, typename... Args>
[[deprecated]] std::vector<T> query(Args &&...args);

//执行原生的sql语句
int execute(const std::string& sql);

//开始事务
bool begin();

//提交事务
bool commit();

//回滚
bool rollback();
```

### 具体的接口使用介绍
先在entity.hpp中定义业务实体（和数据库的表对应），接着定义数据库对象：

```C++
#include "dbng.hpp"
//#include "mysql.hpp"...等等，别忘记了
using namespace ormpp;

struct person {
  int id;
  std::string name;
  std::optional<int> age; // 插入null值
  static constexpr std::string_view get_alias_struct_name(student *) {
    return "CUSTOM_TABLE_NAME";
  }
};
REGISTER_AUTO_KEY(person, id)
YLT_REFL(person, id, name, age)

int main(){
	dbng<mysql> mysql;
  dbng<sqlite> sqlite;
  dbng<postgresql> postgres;
	//......
}
```

1. 连接数据库
```cpp
	template <typename... Args>
	bool connect(Args&&... args);
```

connect exmple:

```C++
// mysql.connect(host, dbuser, pwd, dbname);
mysql.connect("127.0.0.1", "root", "12345", "testdb");
// mysql.connect(host, dbuser, pwd, dbname, timeout, port);
mysql.connect("127.0.0.1", "root", "12345", "testdb", 5, 3306);
  
postgres.connect("127.0.0.1", "root", "12345", "testdb");

sqlite.connect("127.0.0.1", "testdb");//或直接sqlite.connect("testdb")；

//开启sqlcipher后
sqlite.connect("127.0.0.1", "root", "12345", "testdb");//或者直接sqlite.connect("testdb", "123456");
```

返回值：bool，成功返回true，失败返回false.

2. 断开数据库连接
```cpp	
	bool disconnect();
```

disconnect exmple:

```c++
mysql.disconnect();

postgres.disconnect();

sqlite.disconnect();
```

注意：用户可以不用显式调用，在数据库对象析构时会自动调用disconnect接口。

返回值：bool，成功返回true，失败返回false.

3. 创建数据表

```C++
template<typename T, typename... Args>
bool create_datatable(Args&&... args);
```

create_datatable example:

```C++
//创建不含主键的表
mysql.create_datatable<student>();

postgres.create_datatable<student>();

sqlite.create_datatable<student>();

//创建含主键和not null属性的表
ormpp_key key1{"id"};
ormpp_not_null not_null{{"id", "age"}};

person p = {1, "test1", 2};
person p1 = {2, "test2", 3};
person p2 = {3, "test3", 4};

mysql.create_datatable<person>(key1, not_null);
postgres.create_datatable<person>(key1, not_null);
sqlite.create_datatable<person>(key1);
```

注意：目前只支持了key、unique和not null属性。
```
mysql.create_datatable<person>(ormpp_unique{{"name"}});
当在mysql中使用由unique声明的std::string成员创建表时，
由于"BLOB/TEXT column 'NAME' used in key specification without a key length", 
故在创建表时，如果是由unique声明的std::string成员对应的数据类型则为VARCHAR(512)，否则则为TEXT
```

返回值：bool，成功返回true，失败返回false.

4. 插入单条数据

```C++
template<typename T, typename... Args>
int insert(const T& t, Args&&... args);
```

insert example:

```C++
person p = {1, "test1", 2};
TEST_CHECK(mysql.insert(p)==1);
TEST_CHECK(postgres.insert(p)==1);
TEST_CHECK(sqlite.insert(p)==1);

// age为null
person p = {1, "test1", {}};
TEST_CHECK(mysql.insert(p)==1);
TEST_CHECK(postgres.insert(p)==1);
TEST_CHECK(sqlite.insert(p)==1);
```

返回值：int，成功返回插入数据的条数1，失败返回INT_MIN.

5. 插入多条数据

```C++
template<typename T, typename... Args>
int insert(const std::vector<T>& t, Args&&... args);
```

multiple insert example:

```C++
person p = {1, "test1", 2};
person p1 = {2, "test2", 3};
person p2 = {3, "test3", 4};
std::vector<person> v1{p, p1, p2};

TEST_CHECK(mysql.insert(v1)==3);
TEST_CHECK(postgres.insert(v1)==3);
TEST_CHECK(sqlite.insert(v1)==3);
```

返回值：int，成功返回插入数据的条数N，失败返回INT_MIN.

6. 更新单条数据


```C++
template<typename T, typename... Args>
int update(const T& t, Args&&... args);
```

update example:

```C++
person p = {1, "test1", 2};
TEST_CHECK(mysql.update(p)==1);
TEST_CHECK(postgres.update(p)==1);
TEST_CHECK(sqlite.update(p)==1);
```

注意：更新会根据表的key字段去更新，如果表没有key字段的时候，需要指定一个更新依据字段名，比如
```C++
TEST_CHECK(mysql.update(p, "age")==1);
TEST_CHECK(postgres.update(p, "age")==1);
TEST_CHECK(sqlite.update(p, "age")==1);
```

返回值：int，成功返回更新数据的条数1，失败返回INT_MIN.

7. 更新多条数据

```C++
template<typename T, typename... Args>
int update(const std::vector<T>& t, Args&&... args);
```

multiple insert example:

```C++
person p = {1, "test1", 2};
person p1 = {2, "test2", 3};
person p2 = {3, "test3", 4};
std::vector<person> v1{p, p1, p2};

TEST_CHECK(mysql.update(v1)==3);
TEST_CHECK(postgres.update(v1)==3);
TEST_CHECK(sqlite.update(v1)==3);
```

注意：更新会根据表的key字段去更新，如果表没有key字段的时候，需要指定一个更新依据字段名，用法同上。

返回值：int，成功返回更新数据的条数N，失败返回INT_MIN.

8. 删除数据
```cpp
template<typename T, typename... Args>
int delete_records_s(const std::string &str = "", Args &&...args);
```

delete_records_s example:

```C++
//删除所有数据
TEST_REQUIRE(mysql.delete_records_s<person>());
TEST_REQUIRE(postgres.delete_records_s<person>());
TEST_REQUIRE(sqlite.delete_records_s<person>());

//根据条件删除数据
TEST_REQUIRE(mysql.delete_records_s<person>("id=?", 1));
TEST_REQUIRE(postgres.delete_records_s<person>("id=$1", 1));
TEST_REQUIRE(sqlite.delete_records_s<person>("id=?", 1));
```

返回值：bool，成功返回true，失败返回false.

9. 查询数据

```C++
template<typename T, typename... Args>
std::vector<T> query_s(const std::string &str = "", Args &&...args);
```

query_s example:

```C++
auto result = mysql.query_s<person>();
TEST_CHECK(result.size()==3);

auto result1 = postgres.query_s<person>();
TEST_CHECK(result1.size()==3);

auto result2 = sqlite.query_s<person>();
TEST_CHECK(result2.size()==3);

//可以根据条件查询
auto result3 = mysql.query_s<person>("id=?", 1);
TEST_CHECK(result3.size()==1);

auto result4 = postgres.query_s<person>("id=$1", 2);
TEST_CHECK(result4.size()==1);

auto result5 = sqlite.query_s<person>("id=?", 3);
```

返回值：std::vector<T>，成功vector不为空，失败则为空.

10. 特定列查询

```C++
template<typename T, typename... Args>
std::vector<T> query_s(const std::string &str = "", Args &&...args);
```

some fields query_s example:

```C++
auto result = mysql.query_s<std::tuple<int, std::string, int>>("select code, name, dm from person");
TEST_CHECK(result.size()==3);

auto result1 = postgres.query_s<std::tuple<int, std::string, int>>("select code, name, dm from person");
TEST_CHECK(result1.size()==3);

auto result2 = sqlite.query_s<std::tuple<int, std::string, int>>("select code, name, dm from person");
TEST_CHECK(result2.size()==3);

auto result3 = mysql.query_s<std::tuple<int>>("select count(1) from person");
TEST_CHECK(result3.size()==1);
TEST_CHECK(std::get<0>(result3[0])==3);

auto result4 = postgres.query_s<std::tuple<int>>("select count(1) from person");
TEST_CHECK(result4.size()==1);
TEST_CHECK(std::get<0>(result4[0])==3);

auto result5 = sqlite.query_s<std::tuple<int>>("select count(1) from person");
TEST_CHECK(result5.size()==1);
TEST_CHECK(std::get<0>(result5[0])==3);
```

返回值：std::vector<std::tuple<T>>，成功vector不为空，失败则为空.

11. 执行原生sql语句

```C++
int execute(const std::string& sql);
```

execute example:

```C++
r = mysql.execute("drop table if exists person");
TEST_REQUIRE(r);

r = postgres.execute("drop table if exists person");
TEST_REQUIRE(r);

r = sqlite.execute("drop table if exists person");
TEST_REQUIRE(r);
```

注意：execute接口支持的原生sql语句是不带占位符的，是一条完整的sql语句。

返回值：int，成功返回更新数据的条数1，失败返回INT_MIN.

12. 事务接口

开始事务，提交事务，回滚

```C++
//transaction
mysql.begin();
for (int i = 0; i < 10; ++i) {
  person s = {i, "tom", 19};
      if(!mysql.insert(s)){
          mysql.rollback();
          return -1;
      }
}
mysql.commit();
```
返回值：bool，成功返回true，失败返回false.

13. 面向切面编程AOP

定义切面：

```C++
struct log{
	//args...是业务逻辑函数的入参
    template<typename... Args>
    bool before(Args... args){
        std::cout<<"log before"<<std::endl;
        return true;
    }

	//T的类型是业务逻辑返回值，后面的参数则是业务逻辑函数的入参
    template<typename T, typename... Args>
    bool after(T t, Args... args){
        std::cout<<"log after"<<std::endl;
        return true;
    }
};

struct validate{
	//args...是业务逻辑函数的入参
    template<typename... Args>
    bool before(Args... args){
        std::cout<<"validate before"<<std::endl;
        return true;
    }

	//T的类型是业务逻辑返回值，后面的参数则是业务逻辑函数的入参
    template<typename T, typename... Args>
    bool after(T t, Args... args){
        std::cout<<"validate after"<<std::endl;
        return true;
    }
};
```

注意：切面的定义中，允许你只定义before或after，或者二者都定义。

```C++
//增加日志和校验的切面
dbng<mysql> mysql;
auto r = mysql.warper_connect<log, validate>("127.0.0.1", "root", "12345", "testdb");
TEST_REQUIRE(r);
```

## roadmap

1. 支持组合键。
1. 多表查询时增加一些诸如where, group, oder by, join, limit等常用的谓词，避免直接写sql语句。
2. 增加日志
3. 增加获取错误消息的接口
4. 支持更多的数据库
5. 增加数据库链接池


## 联系方式

purecpp@163.com

qq群: 492859173

[http://purecpp.cn/](http://purecpp.cn/ "purecpp")

[https://github.com/qicosmos/ormpp](https://github.com/qicosmos/ormpp "ormpp")