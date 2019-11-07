cmake_minimum_required(VERSION 2.8)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS "-std=c++11")


include_directories(
                /usr/include/eigen3/
                backend
                )


add_executable(pose_graph
            test/main.cpp
            #test/test.cpp
            backend/vertex.h
            backend/vertex.hpp
            backend/vertex_pose.h
            backend/vertex_point.h
            backend/base_edge.h
            backend/base_binary_edge.h
            backend/base_binary_edge.hpp
            backend/edge_SE3ProjectXYZ.cpp
            backend/edge_SE3ProjectXYZ.h
            backend/edge_pose2pose.h
            backend/edge_pose2pose.cpp
            backend/problem.cpp
            backend/problem.h
            )

