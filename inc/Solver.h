/*
 * Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 20, 2021
 *
 * Description:
 */

#pragma once

#include <vector>
#include <deque>
#include <limits>
#include <stack>
#include <stdlib.h>
#include <time.h>
#include <list>
#include <queue>

#include "Partitioner.h"
#include "Path_Planner.h"
#include "Hybrid_Planner.h"
#include "Generic_Partitioner.h"
#include "Random_PathPlanner.h"
#include "graph_defines.h"
#include "defines.h"
#include "Edge.h"
#include "DEdge.h"
#include "AEdge.h"
#include "Vertex.h"
#include "VertexOnEdge.h"
#include "Graph_Theory_Algorithms.h"
#include "EmptyGraph_Planner.h"


class Solver {
public:
	Solver(Hybrid_Planner* pHybridPlanner);
	Solver(EmptyGraph_Planner* pEmptyGraphPlanner);
	Solver(Partitioner* pPartitioner, Path_Planner* pPath_Planner);
	~Solver();

	/*
	 * This function solves the given path in the gaph_path file using m partitions.
	 * It uses the Partitioner and Path_Planner object that were given to the solver
	 * in its constructor.
	 */
	void SolvePathPlanning(Solution* solution);

private:
	// Partitioner pointer, used to partition up the search space
	Partitioner* m_pPartitioner;
	// Path Planner pointer, used to plan out paths in each partition of the given solution
	Path_Planner* m_pPath_Planner;
	// Hybrid Planner pointer, used to jointly perform both partitioning and path-planning
	Hybrid_Planner* m_pHybridPlanner;
	// Empty-Graph Planner pointer, does path planning where m is determined during planning
	EmptyGraph_Planner* m_pEmptyGraphPlanner;
};
