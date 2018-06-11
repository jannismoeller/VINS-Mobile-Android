
#[[ All Headers that are included else where
    # Included in loop & dbow
    # DVision.h
    ../ThirdParty/DVision/DVision.h
    ../ThirdParty/DVision/BRIEF.h
        
    #DUtils.h
]]

set(dvision_SRCS
    BRIEF.cpp   
)

set(TARGET_NAME DVision)
add_library(${TARGET_NAME} STATIC ${dvision_SRCS})

target_link_libraries(${TARGET_NAME} opencv_core)
target_link_libraries(${TARGET_NAME} DUtils)
target_link_libraries(${TARGET_NAME} boost_LIB)

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})