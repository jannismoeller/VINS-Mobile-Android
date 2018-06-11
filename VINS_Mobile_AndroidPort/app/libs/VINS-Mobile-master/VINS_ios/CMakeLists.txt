cmake_minimum_required(VERSION 3.4.1)

#[[ All Headers that are included else where
    VINS.hpp
    feature_manager.hpp
    utility.hpp
    projection_facor.hpp
    pose_local_parameterization.hpp
    global_param.hpp
    marginalization_factor.hpp
    imu_factor.h
    draw_result.hpp
    inital_sfm.hpp
    initial_aligment.hpp
    motion_estimator.hpp
    
    feature_tracker.hpp
    integration_base.h
    vins_pnp.hpp
    imu_factor_pnp.h
    perspective_factor.hpp
]]

set(vinsmobile_SRCS
    VINS.cpp
    feature_manager.cpp
    utility.cpp
    projection_facor.cpp
    pose_local_parameterization.cpp
    global_param.cpp
    marginalization_factor.cpp
    draw_result.cpp
    inital_sfm.cpp
    initial_aligment.cpp
    motion_estimator.cpp
    
    feature_tracker.cpp
    vins_pnp.cpp
    imu_factor_pnp.h
    perspective_factor.cpp
)

add_subdirectory(delaunay)
add_subdirectory(loop)


set(TARGET_NAME VINSMobile)
add_library(
    ${TARGET_NAME}
    STATIC
    ${vinsmobile_SRCS}
)

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${TARGET_NAME} Delaunay)
target_link_libraries(${TARGET_NAME} Loop)

target_link_libraries(
    ${TARGET_NAME}
    ceres_LIB
    boost_LIB
    eigen_LIB
    ${OpenCV_LIBS}
)
