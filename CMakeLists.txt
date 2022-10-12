cmake_minimum_required(VERSION 2.8)
project( canbus-tcp-master )
#find_package( OpenCV REQUIRED )
#find_package( CUDA REQUIRED )

set(ENV{PKG_CONFIG_PATH} /usr/lib/pkgconfig)
find_package(PkgConfig)
pkg_search_module(LCM REQUIRED lcm)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#add_definitions(-std=c++11)

#INCLUDE_DIRECTORIES(/usr/local/opencv4)
#INCLUDE_DIRECTORIES(/usr/local/include/opencv4)
INCLUDE_DIRECTORIES(${LCM_INCLUDE_DIRS})

#LIST(APPEND CUDA_NVCC_FLAGS -std=c++11;-O2)
#LIST(APPEND CUDA_NVCC_FLAGS -Xcompiler;-fPIC)

#cuda_add_executable(canbus-tcp-master canbus-tcp-master.cpp read-line.c tcp_can.cpp can_node.cpp csv_parser.c atox.c)
#cuda_add_executable(canbus-master canbus-master.cpp read-line.c socket_can.cpp can_node.cpp csv_parser.c atox.c)

add_executable(canbus-tcp-master canbus-tcp-master.cpp read-line.c tcp_can.cpp can_node.cpp csv_parser.c atox.c)
add_executable(canbus-master canbus-master.cpp read-line.c socket_can.cpp can_node.cpp csv_parser.c atox.c)

#set(canbus-tcp-master canbus-tcp-master PROPERTY CUDA_STANDARD 11)
#set(canbus-master canbus-master PROPERTY CUDA_STANDARD 11)

#target_link_libraries(canbus-tcp-master ${OpenCV_LIBS} ${LCM_LIBRARIES} Threads::Threads)
#target_link_libraries(canbus-master ${OpenCV_LIBS} ${LCM_LIBRARIES} Threads::Threads)

target_link_libraries(canbus-tcp-master ${LCM_LIBRARIES} Threads::Threads)
target_link_libraries(canbus-master ${LCM_LIBRARIES} Threads::Threads)

