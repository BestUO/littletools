project(speechservice)

include_directories(${THIRD_PARTY_PATH}/include/alinls)
link_directories(${THIRD_PARTY_PATH}/lib/alinls)

add_library(${PROJECT_NAME} SHARED speechSynthesizer.cpp speechTranscriber.cpp SpeecService.cpp speechRestfulAsr.cpp speechRestfulAsr.h)
target_link_libraries(${PROJECT_NAME} alibabacloud-idst-speech z pthread curl)
