/*
 * Tree_Cutting_LKH.h
 *
 * Created by:	Jonathan Diller
 * On: 			Aug 22, 2022
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

#define TCUT_LKH_DEBUG	0 || DEBUG
#define PRINT_LKH_PATH	0
#define SPEED_SCHEULING	1
#define ROUND_ERROR		0.1
#define DIST_TOLERANCE	1

class Tree_Cutting_LKH: public EmptyGraph_Planner, public Tree_Solver {
public:
	Tree_Cutting_LKH();
	virtual ~Tree_Cutting_LKH();

protected:
	/*
	 * Runs tree-following algorithm
	 */
	virtual void RunAlgorithm(Solution* pathSolution);

private:
};
