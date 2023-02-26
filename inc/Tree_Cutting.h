/*
 * Tree_Cutting.h
 *
 * Created by:	Jonathan Diller
 * On: 			June 08, 2022
 *
 * Description: Creates a path that traverse entire area, then "cuts" that path into m
 * equal parts until it finds a good value for m.
 */

#pragma once

#include <stdlib.h>
#include <math.h>
#include <stack>
#include <vector>
#include <list>
#include <limits>

#include "defines.h"
#include "Solution.h"
#include "EmptyGraph_Planner.h"
#include "Tree_Solver.h"
#include "Graph_Theory_Algorithms.h"
#include "UDEdge.h"
#include "ShortCutTree.h"

#define TCUT_DEBUG		0 || DEBUG
#define PRINT_TREE		1
#define SPEED_SCHEULING	1
#define ROUND_ERROR		0.1

class Tree_Cutting: public EmptyGraph_Planner, public Tree_Solver {
public:
	Tree_Cutting();
	virtual ~Tree_Cutting();

protected:
	/*
	 * Runs tree-following algorithm
	 */
	virtual void RunAlgorithm(Solution* pathSolution);

private:
};
