#include "common_utils/string_utils/split_string.h"

namespace common_utils {
    std::vector<std::string> SplitString(const std::string &str, const char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(str);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }
}
