cmake_minimum_required (VERSION 3.5)

project(mpc)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")
set(sources src/MPC.cpp src/main.cpp)

include_directories(/usr/local/include)
include_directories(src/Eigen-3.3)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	include_directories(/usr/local/opt/openssl/include)
	link_directories(/usr/local/opt/openssl/lib)
	link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} ipopt z ssl uv uWS)

