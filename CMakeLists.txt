cmake_minimum_required(VERSION 3.27)
project(BOW)

set(CMAKE_CXX_STANDARD 23)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(limbo REQUIRED)
find_package(FCL REQUIRED)

include_directories(
        include
        ${limbo_INCLUDE_DIRS}
        ${fcl_INCLUDE_DIRS}
        /usr/include/python3.10/
)
link_directories(${CMAKE_BINARY_DIR}/install/lib)
add_executable(BOW main.cpp
        include/robot_model.h
        include/collision_checker.h
        include/bo_param.h
        include/bow_planner.h

)
target_link_libraries(BOW ${limbo_LIBRARIES} ${fcl_LIBRARIES} -lpython3.10 -pthread)
