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
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
set(Torch_DIR /home/zj/Desktop/wzr/sim_cpp/src/thirdparty/libtorch/share/cmake/Torch) 
find_package(Torch REQUIRED)

include_directories(
    ./src/map 
    ./src/pnc
    ./src/utils
    ./src/thirdparty/include
    ./src/thirdparty/include/osqp
    ./src/thirdparty/include/qdldl
    /usr/local/include/eigen3
    /usr/include/eigen3
    /usr/include/python3.8
    /home/ballade/anaconda3/envs/drl/include/python3.8
    /home/ballade/anaconda3/envs/drl/lib/python3.8/site-packages/numpy/core/include)

file(GLOB SIM_SOURCEFILE  ./src/map/*.cpp ./src/pnc/*.cpp)
add_library(SIM_LIB STATIC ${SIM_SOURCEFILE})

add_executable(pnc_test ./src/pnc/pnc_test.cc)
# target_include_directories(pnc_test PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} lib /home/zj/Desktop/wzr/sim_cpp/src/thirdparty/lib)
target_link_directories(pnc_test PRIVATE /home/zj/Desktop/wzr/sim_cpp/src/thirdparty/lib)
target_link_libraries(pnc_test Python3::Python Python3::NumPy SIM_LIB osqp qdldl)

add_executable(ring_test ./src/utils/test_ring.cc)


add_executable(torch_test ./src/test.cc)
target_link_libraries(torch_test ${TORCH_LIBRARIES})

# add_executable(test ./src/test.cc)
# target_include_directories(test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
# target_link_libraries(test Python3::Python Python3::NumPy SIM_LIB)

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS}) 
# add_executable(search_test ./src/planner/search_test.cc)
# target_link_libraries(search_test ${OpenCV_LIBS} SIM_LIB)



# add_executable(vector_test ./src/ring_vector/vector_test.cc)
# target_link_libraries(vector_test SIM_LIB)

# add_executable(task_test ./src/task/task_test.cc)
# target_link_libraries(task_test SIM_LIB)

# add_executable(kalman_test ./src/kalman/kalman_test.cc)
# target_include_directories(kalman_test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
# target_link_libraries(kalman_test Python3::Python Python3::NumPy SIM_LIB)
# target_link_libraries(kalman_test SIM_LIB)

# add_executable(test_2_1 ./src/test_2_1.cc)
# target_include_directories(test_2_1 PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS} lib)
# target_link_libraries(test_2_1 Python3::Python Python3::NumPy SIM_LIB)
# target_link_libraries(test_2_1 SIM_LIB)