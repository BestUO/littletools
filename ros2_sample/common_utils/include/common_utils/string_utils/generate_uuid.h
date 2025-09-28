/**
 * ===============================================================
 *  @copyright Copyright (c) yijiahe Co. Ltd. All Rights Reserved.
 *  @file generate_uuid.h
 *  @author fanlong
 *  @date 2025-08-21 
 *  @Description TODO 描述该文件的功能  
 * ===============================================================
 */

#ifndef BUILD_GENERATE_UUID_H
#define BUILD_GENERATE_UUID_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>

namespace common_utils {
    inline std::string GenerateUuid() {
        boost::uuids::random_generator generator;
        boost::uuids::uuid uuid = generator();
        return boost::uuids::to_string(uuid);
    }
}
#endif //BUILD_GENERATE_UUID_H