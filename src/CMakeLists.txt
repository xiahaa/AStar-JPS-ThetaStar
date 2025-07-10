cmake_minimum_required(VERSION 2.8)
project(AStar-ThetaStar)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(SOURCE_FILES
        asearch.cpp
        config.h
        environmentoptions.cpp
        mission.cpp
        searchresult.h
        astar.cpp
        environmentoptions.h
        isearch.cpp
        map.cpp
        mission.h
        theta.cpp
        astar.h
        config.cpp
        gl_const.h
        isearch.h
        map.h
        node.h
        theta.h
        path_smoothing.h
        path_smoothing.cpp
        tinyxml2.h
        tinyxml2.cpp
        )
add_executable(AStar-ThetaStar ${SOURCE_FILES})