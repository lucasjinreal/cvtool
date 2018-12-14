cmake_minimum_required(VERSION 3.8)

project(a)



find_package(OpenCV)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "not find opencv.")
endif()

link_directories(${CMAKE_INSTALL_PREFIX}/lib)



#set(BUILD_SHARED_LIBS OFF)

add_executable(calib_local calib_local.cc)
target_link_libraries(calib_local ${OpenCV_LIBS} thor)


add_executable(calib_online calib_online.cc)
target_link_libraries(calib_online ${OpenCV_LIBS} thor)

add_executable(calib_online_realsense calib_online_realsense.cc)
target_link_libraries(calib_online_realsense ${OpenCV_LIBS} thor realsense2)
