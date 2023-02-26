/*
 * Path_Planner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 30, 2021
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

#include "graph_defines.h"
#include "defines.h"
#include "Edge.h"
#include "DEdge.h"
#include "AEdge.h"
#include "Vertex.h"
#include "VertexOnEdge.h"
#include "Graph_Theory_Algorithms.h"
#include "Generic_Partitioner.h"

class Path_Planner {
public:
	Path_Planner();
	virtual ~Path_Planner();

	/*
	 * Runs the path planning algorithm
	 */
	void RunPathPlanningAlgorithm(Solution* pathSolution);
protected:
	/*
	 * Runs the partitioning algorithm that is defined in the child class
	 */
	virtual void RunAlgorithm(Solution* pathSolution) = 0;

private:
};
