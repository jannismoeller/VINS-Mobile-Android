cmake_minimum_required(VERSION 3.4.1)

#[[ All Headers that are included else where
    # Included in loop
    # DBoW2.h
    # FBrief.h
    # TemplatedVocabulary.h
    # TemplatedDatabase.h
    # QueryResults.h
    # BowVector.h
    
    ../ThirdParty/DBoW/DBoW2.h
    ../ThirdParty/DBoW/FBrief.h
    ../ThirdParty/DBoW/TemplatedVocabulary.h
    ../ThirdParty/DBoW/TemplatedDatabase.h
    ../ThirdParty/DBoW/QueryResults.h
    ../ThirdParty/DBoW/BowVector.h
    
    ../ThirdParty/DBoW/FeatureVector.h
    ../ThirdParty/DBoW/FClass.h
    #DVision.h
    ../ThirdParty/DBoW/ScoringObject.h
    #DUtils.h
    #VocabularyBinary.hpp    
]]

set(dbow_SRCS
    FBrief.cpp
    QueryResults.cpp
    BowVector.cpp
    FeatureVector.cpp
    ScoringObject.cpp
)

set(TARGET_NAME DBoW)
add_library(${TARGET_NAME} STATIC ${dbow_SRCS})

target_link_libraries(${TARGET_NAME} Loop)
target_link_libraries(${TARGET_NAME} DUtils)
target_link_libraries(${TARGET_NAME} DVision)

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})