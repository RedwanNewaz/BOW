cmake_minimum_required(VERSION 3.27)
project(MBOW)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -g)
endif()
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
find_package(Boost REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

include_directories(
        include
        include/limbo
        ${Boost_INCLUDE_DIRS})

add_executable(MBOW main.cpp
        src/BOW.cpp
        include/MinkowskiSumComputer.h
        include/CollisionChecker.h
        src/CollisionChecker.cpp
        include/param_manager.h)
target_precompile_headers(MBOW PUBLIC include/limbo/limbo/limbo.hpp)

target_link_libraries(MBOW
        yaml-cpp::yaml-cpp
        Eigen3::Eigen
        -lnlopt_cxx
        Boost::boost)