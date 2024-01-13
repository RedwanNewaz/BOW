cmake_minimum_required(VERSION 3.22)
project(BOW)

set(CMAKE_CXX_STANDARD 23)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem program_options REQUIRED)


include(ExternalProject)
include(FetchContent)

ExternalProject_Add(
    "libccd"
    URL "https://github.com/danfis/libccd/archive/refs/tags/v2.1.zip"
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}/install
)
set(LIBCCD_ROOT ${CMAKE_BINARY_DIR}/libccd-prefix/src/libccd/src)
include_directories( include ${LIBCCD_ROOT} /usr/include)

ExternalProject_Add(
    "fcl"
    URL "https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.zip"
    DEPENDS libccd
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
        -DCMAKE_INSTALL_INCLUDEDIR=${LIBCCD_ROOT}
        -DBUILD_TESTING=OFF
)

ExternalProject_Add(
        "nlopt"
        URL "https://github.com/stevengj/nlopt/archive/refs/tags/v2.7.1.zip"
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
)

ExternalProject_Add(
        "libcmaes"
        URL "https://github.com/CMA-ES/libcmaes/archive/refs/tags/v0.10.zip"
        CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/install
)

FetchContent_Declare(limbo
        URL "https://github.com/resibots/limbo/archive/refs/tags/v2.1.0.zip"
        # DEPENDS nlopt libcmaes
)
FetchContent_MakeAvailable(limbo)

link_directories(${CMAKE_BINARY_DIR}/install/lib)

set(fcl_INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}/install/include
)


set(limbo_INCLUDE_DIRS
        ${Eigen3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_BINARY_DIR}/install/include
        ${CMAKE_BINARY_DIR}/_deps/limbo-src/src)

set(limbo_LIBRARIES
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        -lnlopt -lm
        libcmaes.so
)



set(fcl_LIBRARIES
    Eigen3::Eigen
    libccd.so
    libfcl.so
)

include_directories(
        include
        ${limbo_INCLUDE_DIRS}
        ${fcl_INCLUDE_DIRS}
        /usr/include/python3.10/
)

add_executable(BOW main.cpp
        include/robot_model.h
        include/collision_checker.h
        include/bo_param.h
        include/bow_planner.h

)
# Add dependencies on ExternalProjects
add_dependencies(BOW libccd fcl nlopt libcmaes)
target_include_directories(BOW PRIVATE ${LIBCCD_ROOT})

target_link_libraries(BOW ${limbo_LIBRARIES} ${fcl_LIBRARIES} -lpython3.10 -pthread)
