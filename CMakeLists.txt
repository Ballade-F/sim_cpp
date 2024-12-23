cmake_minimum_required(VERSION 3.15)
project(sim)
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

if (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    message(NOTICE "Debug mode")
else()
    message(NOTICE "Release mode")
endif()

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
set(Torch_DIR /home/zj/Desktop/wzr/sim_cpp/src/thirdparty/libtorch/share/cmake/Torch) 
find_package(Torch REQUIRED)

include_directories(
    ./src/utils
    ./src/robot
    ./src/thirdparty/include
    ./src/thirdparty/include/osqp
    ./src/thirdparty/include/qdldl
    ./src/thirdparty/include/json
    /usr/local/include/eigen3
    /usr/include/eigen3
    ${TORCH_INCLUDE_DIRS}
)

file(GLOB ROBOT_SOURCEFILE  ./src/utils/*.cpp ./src/robot/*.cpp )
add_library(ROBOT_LIB STATIC ${ROBOT_SOURCEFILE})

# add_executable(pnc_test ./src/test/pnc_test.cc)
# target_link_directories(pnc_test PRIVATE /home/zj/Desktop/wzr/sim_cpp/src/thirdparty/lib)
# target_link_libraries(pnc_test SIM_LIB osqp qdldl)

# add_executable(ring_test ./src/test/test_ring.cc)


# add_executable(torch_test ./src/test/test.cc)
# target_link_libraries(torch_test ${TORCH_LIBRARIES})

add_executable(test_sim ./src/test/test_sim.cc)
target_link_directories(test_sim PRIVATE /home/zj/Desktop/wzr/sim_cpp/src/thirdparty/lib)
target_link_libraries(test_sim ROBOT_LIB osqp qdldl jsoncpp ${TORCH_LIBRARIES})
