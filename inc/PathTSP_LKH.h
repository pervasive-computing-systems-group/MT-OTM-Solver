/*
 * PathTSP_LKH.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov. 5, 2025
 *
 * Description: Solves a Path-TSP in each partition using the LKH solver
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <vector>

#include "Path_Planner.h"
#include "gurobi_c++.h"

#define DEBUG_PTSP_LKH	1 || DEBUG


class PathTSP_LKH : public Path_Planner {
public:
	PathTSP_LKH();
	virtual ~PathTSP_LKH();
protected:

	/*
	 * Runs the Path-TSP MIP solver for each partition in solution
	 */
	void RunAlgorithm(Solution* solution);

private:

	/*
	 * Solves a Path-TSP problem in partition p using a MIP and Gurobi
	 */
	void PathTSP(Solution* solution, int p);
	// Run LKH TSP solver on the give cluster of vertices
	void runLKH_TSP(Solution* solution, std::vector<Vertex*>* cluster, std::vector<Vertex*>* sub_tour);
};
