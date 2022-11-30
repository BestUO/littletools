project(speechservice)

MESSAGE(${THIRD_PARTY_PATH}/lib/alinls)
include_directories(${THIRD_PARTY_PATH}/include/alinls)
link_directories(${THIRD_PARTY_PATH}/lib/alinls)

add_library(${PROJECT_NAME} SHARED speechSynthesizer.cpp speechTranscriber.cpp SpeecService.cpp)
# find_library(alinls NAMES alibabacloud-idst-speech PATHS ${THIRD_PARTY_PATH}/lib/alinls)
target_link_libraries(${PROJECT_NAME} alibabacloud-idst-speech z pthread)
