/*
 * TotalPath_Follower.h
 *
 * Created by:	Jonathan Diller
 * On: 			May 17, 2022
 *
 * Description: Creates a path that traverse entire area, then "follows"
 * this path, adding way-points to sub-tours as it goes along.
 */

#pragma once

#include <stdlib.h>
#include <math.h>
#include <stack>
#include <vector>
#include <list>

#include "defines.h"
#include "Solution.h"
#include "EmptyGraph_Planner.h"
#include "Tree_Solver.h"
#include "Graph_Theory_Algorithms.h"
#include "UDEdge.h"
#include "ShortCutTree.h"

#define TPF_DEBUG		1 || DEBUG
#define SPEED_SCHEULING	1

class TotalPath_Follower: public EmptyGraph_Planner, public Tree_Solver {
public:
	TotalPath_Follower();
	virtual ~TotalPath_Follower();

protected:
	/*
	 * Runs tree-following algorithm
	 */
	virtual void RunAlgorithm(Solution* pathSolution);

private:
};
