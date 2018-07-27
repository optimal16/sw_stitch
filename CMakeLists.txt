cmake_minimum_required(VERSION 2.9)
project(sw_stitch)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fopenmp")

execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/${ARCHITECTURE})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${ARCHITECTURE})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${ARCHITECTURE})

set(3RD_PARTY_INCLUDE_DIRS /usr/local/mvp/include) #including Eigen gflags glog ceres json11
set(3RD_PARTY_LIB_DIRS /usr/local/mvp/lib) 

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${3RD_PARTY_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include )

                #set(SOURCE_FILES source/chessboard/Chessboard.cpp 
                #		 source/chessboard/ChessboardCorner.h 
                #		 source/chessboard/ChessboardQuad.h 
                #		 source/chessboard/Spline.h)
                #
                #add_library(chessboard SHARED ${SOURCE_FILES})
                #target_link_libraries(chessboard PUBLIC ${OpenCV_LIBS})

set(STITCH_FILES source/Composition.cpp source/ParkingMapStitch.cpp source/IOConfig.cpp)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp)

add_library(sw_calib SHARED ${SOURCES})
add_library(sw_stitch SHARED ${STITCH_FILES})
link_directories(${3RD_PARTY_LIB_DIRS} ${LIBRARY_OUTPUT_PATH})

target_link_libraries(sw_stitch PUBLIC ${OpenCV_LIBS})
target_link_libraries(sw_calib PUBLIC ${OpenCV_LIBS} ${3RD_PARTY_LIB_DIRS}/libceres.so ${3RD_PARTY_LIB_DIRS}/libglog.so)

set_target_properties(sw_stitch PROPERTIES VERSION 1.0) # libsw_stitch.so => libsw_stitch.so.1.0
set_target_properties(sw_calib PROPERTIES VERSION 1.0)

add_subdirectory(sample)

install(TARGETS sw_stitch 
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/sdk/${CMAKE_SYS_ARCH}/lib/
        ARCHIVE DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/sdk/${CMAKE_SYS_ARCH}/lib/)
install(FILES include/ParkingMapStitch.h
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include)









