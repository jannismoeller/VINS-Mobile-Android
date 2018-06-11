cmake_minimum_required(VERSION 3.4.1)

#[[ All Headers that are included else where
    # Includes from ViewController.h (Objective C)
    #include "keyframe.h"
    #include "loop_closure.h"
    #include "keyfame_database.h"
    
    keyframe.h
    loop_closure.h
    keyfame_database.h
    
    DLoopDetector.h
    demoDetector.h
    
    TemplatedLoopDetector.h
    #DBoW2.h
    #DVision.h
    #FBrief.h
    #TemplatedVocabulary.h
    #TemplatedDatabase.h
    #QueryResults.h
    #BowVector.h
        
    # Inlcuded in dbow
    # VocabularyBinary.hpp
    VocabularyBinary.hpp
]]

set(loop_SRCS
    keyframe.cpp
    loop_closure.cpp
    keyfame_database.cpp
    VocabularyBinary.cpp
)

set(TARGET_NAME Loop)
add_library(${TARGET_NAME} STATIC ${loop_SRCS})

target_link_libraries(${TARGET_NAME} Delaunay)
target_link_libraries(${TARGET_NAME} DBoW)
target_link_libraries(${TARGET_NAME} DVision)
target_link_libraries(${TARGET_NAME} eigen_LIB)
target_link_libraries(${TARGET_NAME} VINSMobile)

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})