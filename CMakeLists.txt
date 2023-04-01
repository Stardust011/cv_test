cmake_minimum_required(VERSION 3.10)
project(cv_test)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

#opencv
if (WIN32)
    set(OpenCV_DIR "H:\\opencv\\build\\install\\x64\\vc17\\lib")
else ()
endif ()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

#CUDA
#find_package(CUDA)
#if (${CUDA_FOUND})
#    set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
#    set(CUDA_SEPARABLE_COMPILATION ON)
#    include_directories(${CUDA_INCLUDE_DIRS})
#    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
#    set(CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20;-G;-g)#根据具体GPU更改参数
#    #arch=compute_30,code=sm_30
#    #arch=compute_35,code=sm_35
#    #arch=compute_50,code=[sm_50,compute_50]
#    #arch=compute_52,code=[sm_52,compute_52]
#    #SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_61,code=sm_61;-std=c++11;-O3;-G;-g)
#    link_directories($ENV{CUDA_PATH}/lib/x64)
#    CUDA_ADD_EXECUTABLE(cv_2022 ${CURRENT_HEADERS} ${CURRENT_SOURCES})
#else (${CUDA_FOUND})
#    MESSAGE(STATUS "Skip CUDA!")
#endif (${CUDA_FOUND})

if (WIN32)
    file(GLOB CURRENT_HEADERS Inc/*.h Inc/*.hpp Inc/*.cuh)
    file(GLOB CURRENT_SOURCES Src/*.cpp Src/*.cu)
    include_directories(win-unix/)
    file(GLOB WIN_UNIX win-unix/*.h win-unix/*.hpp)
else()
    file(GLOB CURRENT_HEADERS Inc/*.h Inc/*.hpp Inc/*.cuh)
    file(GLOB CURRENT_SOURCES Src/*.cpp Src/*.cu)
endif()
source_group("Include" FILES ${CURRENT_HEADERS})
source_group("Source" FILES ${CURRENT_SOURCES})

include_directories(./)
include_directories(Inc/)
include_directories(Src/)

aux_source_directory(Lib/libv4l2_opencv_mat/src libv4l2_SRC_FILES)
add_library(libv4l2 STATIC ${libv4l2_SRC_FILES})
target_include_directories(libv4l2 PUBLIC "Lib/libv4l2_opencv_mat/inc")

add_library(WzSerialPortPlus STATIC Lib/WzSerialPortPlus/WzSerialPortPlus.cpp )
target_include_directories(WzSerialPortPlus PUBLIC "Lib/WzSerialPortPlus")


add_executable(cv_run ${CURRENT_HEADERS} ${CURRENT_SOURCES} ${WIN_UNIX})
target_link_libraries(cv_run libv4l2)
target_link_libraries(cv_run WzSerialPortPlus)

#---------------------TEST---------------------
#include_directories(./)
#include_directories(Inc/)
#include_directories(Src/)
#
#aux_source_directory(Lib/libv4l2_opencv_mat/src libv4l2_SRC_FILES)
#add_library(libv4l2 STATIC ${libv4l2_SRC_FILES})
#target_include_directories(libv4l2 PUBLIC "Lib/libv4l2_opencv_mat/inc")
#
#add_library(WzSerialPortPlus STATIC Lib/WzSerialPortPlus/WzSerialPortPlus.cpp )
#target_include_directories(WzSerialPortPlus PUBLIC "Lib/WzSerialPortPlus")
#
#set(SRC Src/armour.cpp  Src/led.cpp Src/kalman.cpp )
#set(INC Inc/armour.h   Inc/led.h Inc/kalman.h )
##set(SRC Src/armour.cpp Src/led.cpp Src/kalman.cpp Src/serial.cpp)
##set(INC Inc/armour.h Inc/led.h Inc/kalman.h Inc/serial.h)
#add_executable(cv_test ${SRC} ${INC} Src/serial.cpp Inc/serial.h)
#target_link_libraries(cv_test libv4l2 WzSerialPortPlus)
#
##target_link_libraries(cv_test WzSerialPortPlus)
#---------------------END----------------------

#!
add_executable(test_lib main.cpp Src/serial.cpp Inc/serial.h)
target_link_libraries(test_lib WzSerialPortPlus)