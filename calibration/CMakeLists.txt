cmake_minimum_required(VERSION 3.8)

project(a)



find_package(OpenCV)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "not find opencv.")
endif()


find_library(thor_LIBRARY
        NAME thor
        HINTS "${CMAKE_INSTALL_PREFIX}/lib"
        )
if (NOT thor_FOUND)
    message(FATAL_ERROR "Thor lib not found, you can clone from https://github.com/jinfagang/Thor to build and install.")
endif ()


#set(BUILD_SHARED_LIBS OFF)

add_executable(calib_local calib_local.cc)
target_link_libraries(calib_local ${OpenCV_LBS} ${thor_LIBRARY})

