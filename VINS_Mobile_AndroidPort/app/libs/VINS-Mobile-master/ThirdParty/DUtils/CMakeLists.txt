
#[[ All Headers that are included else where
# Included in dbow & dvision
    # DUtils.h
    ../ThirdParty/DUtils/DUtils.h
    ../ThirdParty/DUtils/DException.h
    ../ThirdParty/DUtils/Timestamp.h
    ../ThirdParty/DUtils/Random.h
]]

set(dutils_SRCS  
    Timestamp.cpp
    Random.cpp    
)

set(TARGET_NAME DUtils)
add_library(${TARGET_NAME} STATIC ${dutils_SRCS})

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})