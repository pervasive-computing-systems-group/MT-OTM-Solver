#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=optimized
#  > cmake --build build 
#

cmake_minimum_required(VERSION 3.10)

project(mt-otm-solver)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "debug")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
endif()

find_package(GUROBI REQUIRED)

include_directories(
	${GUROBI_INCLUDE_DIRS}
	${CMAKE_CURRENT_LIST_DIR}/inc
)

## Build the MT-OTM-Solver 
add_executable(mt-otm-solver
	src/main.cpp
	src/AEdge.cpp
	src/DEdge.cpp
	src/Edge.cpp
	src/EmptyGraph_Planner.cpp
	src/FMdMtHPP_NLP.cpp
	src/Generic_Partitioner.cpp
	src/Graph_Theory_Algorithms.cpp
	src/Hybrid_Planner.cpp
	src/KMeans_Partitioner.cpp
	src/Partitioner.cpp
	src/Path_Planner.cpp
	src/PathTSP_MIP_PathPlanner.cpp
	src/Random_PathPlanner.cpp
	src/Roots.cpp
	src/Short_Horizons.cpp
	src/Solution.cpp
	src/Solution_Multi.cpp
	src/Solver.cpp
	src/UDEdge.cpp
	src/UGVTrajectory.cpp
	src/Vertex.cpp
	src/VertexOnEdge.cpp
)

## Build baseline solver
add_executable(tree-solve
	src/main_tree.cpp
	src/AEdge.cpp
	src/DEdge.cpp
	src/Edge.cpp
	src/EmptyGraph_Planner.cpp
	src/Generic_Partitioner.cpp
	src/Graph_Theory_Algorithms.cpp
	src/Hybrid_Planner.cpp
	src/Partitioner.cpp
	src/Path_Planner.cpp
	src/Roots.cpp
	src/Random_PathPlanner.cpp
	src/ShortCutTree.cpp
	src/Solver.cpp
	src/Solution.cpp
	src/Short_Horizons.cpp
	src/TotalPath_Follower.cpp
	src/Tree_Cutting.cpp
	src/Tree_Cutting_LKH.cpp
	src/Tree_Solver.cpp
	src/UDEdge.cpp
	src/UGVTrajectory.cpp
	src/Vertex.cpp
	src/VertexOnEdge.cpp
)

## Build advanced solver
add_executable(moving-solver
	src/main_moving.cpp
	src/AEdge.cpp
	src/DEdge.cpp
	src/Edge.cpp
	src/EmptyGraph_Planner.cpp
	src/Generic_Partitioner.cpp
	src/Graph_Theory_Algorithms.cpp
	src/Hybrid_Planner.cpp
	src/KMeans_Partitioner.cpp
	src/MMdMtHPP_NLP.cpp
	src/Partitioner.cpp
	src/Path_Planner.cpp
	src/Random_PathPlanner.cpp
	src/Roots.cpp
	src/Solution.cpp
	src/Solution_Multi.cpp
	src/Solver.cpp
	src/UDEdge.cpp
	src/UGVTrajectory.cpp
	src/Vertex.cpp
	src/VertexOnEdge.cpp
)

if(CXX)
	set(CMAKE_CXX_STANDARD 11)
	target_link_libraries(mt-otm-solver optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
	target_link_libraries(moving-solver optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

target_link_libraries(mt-otm-solver ${GUROBI_LIBRARY})
target_link_libraries(moving-solver ${GUROBI_LIBRARY})

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
    include(FeatureSummary)
    feature_summary(WHAT ALL)
endif()
