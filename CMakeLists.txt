cmake_minimum_required(VERSION 3.1) 
project(yolo-trt VERSION 1.0)            
enable_language(CUDA)

set(CMAKE_CXX_COMPILIER "/usr/bin/g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -Wno-write-strings ")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,$ORIGIN")





find_package(OpenCV REQUIRED)






######END SORT ############

file(GLOB_RECURSE sources3  ${CMAKE_CURRENT_SOURCE_DIR}/include/objectTracker.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/objectTracker.cpp
)

add_executable(ros2Mot  main.cpp)	
target_include_directories(ros2Mot PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/src/ 
${CMAKE_CURRENT_SOURCE_DIR}/include/
${CUDA_INCLUDE_DIRS}
)

target_link_libraries(ros2Mot  "stdc++fs" ${OpenCV_LIBS} ${Boost_LIBRARIES})


