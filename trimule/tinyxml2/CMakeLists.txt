# project(tinyxml)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
# add_library(${PROJECT_NAME} tinyxml2.cpp tinyxml2.h )
# target_link_libraries(tinyxml ${PROJECT_NAME}  pthread)


cmake_minimum_required(VERSION 2.8)

project(TinyXMLLIB)

include_directories(./)

aux_source_directory(./ LIB_SRCS)

#add_definitions(-DTIXML_USE_STL) #如果需要使用TIXML_USE_STL宏进行编译，则将这一句开启

add_library(tinyxml STATIC ${LIB_SRCS})