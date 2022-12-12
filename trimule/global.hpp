#pragma once

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