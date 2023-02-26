/*
 * Partitioner.h
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

#include "defines.h"
#include "graph_defines.h"
#include "Edge.h"
#include "DEdge.h"
#include "AEdge.h"
#include "Vertex.h"
#include "VertexOnEdge.h"
#include "Solution.h"
#include "Graph_Theory_Algorithms.h"

class Partitioner {
public:
	Partitioner();
	virtual ~Partitioner();

	/*
	 * Runs the partitioning algorithm
	 */
	void RunPartitionAlgorithm(Solution* pathSolution);

protected:
	/*
	 * Runs the partitioning algorithm that is defined in the child class
	 */
	virtual void RunAlgorithm(Solution* pathSolution) = 0;

	bool m_bCreatesTree;
private:
};
