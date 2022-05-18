
ADD_SUBDIRECTORY(run_trimule)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/build)
# set(CONF_FILE ${SOURCE_DIR}/conf)
# set(DBSTRUCT ${SOURCE_DIR}/dbstruct)
# set(KAFKA ${SOURCE_DIR}/kafka)
# set(LIB ${SOURCE_DIR}/lib)
# set(LOG ${SOURCE_DIR}/log)
# set(QUEUE ${SOURCE_DIR}/queue)
# set(SETTINGPARSER ${SOURCE_DIR}/settingParser)
ADD_CUSTOM_TARGET(
    COMMAND
    ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/trimule
)


set(OUTPUT_PATH ${PROJECT_SOURCE_DIR}/trimule)

file(COPY ${SOURCE_DIR}/trimule DESTINATION ${OUTPUT_PATH} FOLLOW_SYMLINK_CHAIN)
FILE(GLOB SOURCE_DIRS 
${SOURCE_DIR}/conf/*
${SOURCE_DIR}/dbstruct/*
${SOURCE_DIR}/kafka/*
${SOURCE_DIR}/lib/*
${SOURCE_DIR}/log/*
${SOURCE_DIR}/queue/*
${SOURCE_DIR}/settingParser/*

)

ADD_CUSTOM_TARGET(copy_file ALL
# COMMAND 
# mv ${SOURCE_DIR}/trimule ${OUTPUT_PATH}/trimule
COMMAND
${CMAKE_COMMAND} -E copy  ${SOURCE_DIRS} ${OUTPUT_PATH}
)
