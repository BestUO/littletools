#pragma once
#include "spdlog/spdlog.h"
#include "tools/jsonwrap.hpp"
#include "ormpp/dbng.hpp"
#include "ormpp/mysql.hpp"

#define SPDLOG_FILENAME "log/TrimuleLogger.log"
#define SPDLOGGERNAME "TrimuleLogger"
#define LOGGER spdlog::get(SPDLOGGERNAME)

template<class ...Args>
std::string TupleToString(std::tuple<Args...> args)
{
    return std::apply([](auto&&... args)
    {
        std::string s = (std::string(args) + "," + ...);
        s.pop_back();
        return s;
    },args);
}

template<typename T>
struct Promoted {
    using type = T;
};

template<>
struct Promoted<const char *> {
    using type = std::string;
};

template<typename T>
using Promoted_t = typename Promoted<T>::type;

template<typename... ArgsT>
auto MakeMyTuple(ArgsT&&... args) {
    return std::make_tuple<Promoted_t<std::decay_t<ArgsT>>...>(std::forward<ArgsT>(args)...);
}

#define GETMYSQLCLIENT \
ormpp::dbng<ormpp::mysql> mysqlclient;\
auto config = JsonSimpleWrap::GetPaser("conf/trimule_config.json");\
mysqlclient.connect(config.value()["mysql_setting"]["mysql_host"].GetString(), \
                    config.value()["mysql_setting"]["mysql_user"].GetString(), \
                    config.value()["mysql_setting"]["mysql_password"].GetString(), \
                    config.value()["mysql_setting"]["mysql_db"].GetString(), \
                    config.value()["mysql_setting"]["mysql_timeout"].GetInt(), \
                    config.value()["mysql_setting"]["mysql_port"].GetInt());

template <typename T> inline constexpr auto to_str(T &&t) {
  if constexpr (std::is_arithmetic_v<std::decay_t<T>>) {
    return std::to_string(std::forward<T>(t));
  } else {
    return std::string("'") + t + std::string("'");
  }
}

inline void get_str(std::string &sql, const std::string &s) {
  auto pos = sql.find_first_of('?');
  sql.replace(pos, 1, " ");
  sql.insert(pos, s);
}

template <typename... Args>
inline std::string get_sql(const std::string &o, Args &&...args) {
  auto sql = o;
  std::string s = "";
  (get_str(sql, to_str(args)), ...);

  return sql;
}