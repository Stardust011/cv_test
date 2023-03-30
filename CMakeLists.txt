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


add_executable(cv_test ${CURRENT_HEADERS} ${CURRENT_SOURCES} ${WIN_UNIX})
target_link_libraries(cv_test libv4l2)
target_link_libraries(cv_test WzSerialPortPlus)

#!TEST_LIB
add_executable(test_lib main.cpp Src/serial.cpp Inc/serial.h)
target_link_libraries(test_lib WzSerialPortPlus)