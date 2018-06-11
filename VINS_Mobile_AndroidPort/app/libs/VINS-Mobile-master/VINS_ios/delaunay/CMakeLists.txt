
#[[ All Headers that are included else where
    delaunay.h
    vector2.h
    triangle.h
    
    edge.h
]]

set(delauny_SRCS
    delaunay.cpp
    triangle.cpp
)

set(TARGET_NAME Delaunay)
add_library(${TARGET_NAME} STATIC ${delauny_SRCS})

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})