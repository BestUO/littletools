project(CallBack)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
add_library(${PROJECT_NAME} CallBack.cpp CallBack.h CmDataCache.h CmDataCache.cpp ActiveGetCallRecord.h ActiveGetCallRecord.cpp)

set(CallBackProject CallBack)
target_link_libraries(${CallBackProject}   redisclient dbstruct )