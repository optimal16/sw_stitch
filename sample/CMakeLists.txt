cmake_minimum_required(VERSION 2.8)
project(sv_sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -O3 -fopenmp")

execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include
		            /usr/local/mvp/include)


                #add_executable(sv sv.cpp)
                #link_directories(/usr/local/mvp/lib ${PROJECT_SOURCE_DIR}/lib/${ARCHITECTURE})
                #target_link_libraries(sv sw_stitch 
                #                         ${OpenCV_LIBS}
                #		                 glog
                #                         ceres)
                #
                #add_executable(testRun testRun.cpp)
                #target_link_libraries(testRun sw_stitch 
                #                              ${OpenCV_LIBS}
                #		                      gflags
                #		                      glog
                #                              ceres)
                #
                #add_executable(testIOConfig testIOConfig.cpp)
                #target_link_libraries(testIOConfig sw_stitch 
                #                              ${OpenCV_LIBS})
                #
                #add_executable(testComposition testComposition.cpp)
                #target_link_libraries(testComposition sw_stitch 
                #${OpenCV_LIBS})

add_executable(testUndistort testUndistort.cpp)
target_link_libraries(testUndistort sw_calib ${OpenCV_LIBS} gflags)

add_executable(testCalib testCalib.cpp)
target_link_libraries(testCalib sw_calib ${OpenCV_LIBS} gflags)

add_executable(testOnlineCalib testOnlineCalib.cpp)
target_link_libraries(testOnlineCalib sw_calib ${OpenCV_LIBS} gflags)

add_executable(testFindPoints testFindPoints.cpp)
target_link_libraries(testFindPoints sw_calib ${OpenCV_LIBS} gflags)

add_executable(testStitch testStitch.cpp)
target_link_libraries(testStitch sw_stitch ${OpenCV_LIBS} gflags)

add_executable(genChessboard genChessboard.cpp)
target_link_libraries(genChessboard ${OpenCV_LIBS} gflags)

add_executable(genExtrinsicPlot genExtrinsicPlot.cpp)
target_link_libraries(genExtrinsicPlot ${OpenCV_LIBS} gflags)

add_executable(testIOConfig testIOConfig.cpp)
target_link_libraries(testIOConfig sw_calib ${OpenCV_LIBS} gflags)
#add_executable(generateCameraPos generateCameraPos.cpp)
#target_link_libraries(generateCameraPos SurroundView ${OpenCV_LIBS})

#add_executable(createMask createMask.cpp)
#target_link_libraries(createMask ${OpenCV_LIBS}) 

add_executable(getMousePosition getMousePosition.cpp)
target_link_libraries(getMousePosition ${OpenCV_LIBS}) 

add_executable(testFindCircle testFindCircle.cpp)
target_link_libraries(testFindCircle ${OpenCV_LIBS}) 

