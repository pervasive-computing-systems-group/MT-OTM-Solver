/*
 * PathTSP_MIP_PathPlanner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Feb 14, 2022
 *
 * Description: Solves a Path-TSP in each partition using a MIP on Gurobi with lazy
 * constraints for sub-tour elimination.
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <vector>

#include "Path_Planner.h"
#include "gurobi_c++.h"

#define DEBUG_MIPTSP		0 || DEBUG

class Subtour {
public:
	Subtour();
	virtual ~Subtour();

	void findsubtour(int n, double** sol, int* tourlenP, int* tour);
};

class SubtourElim : public GRBCallback {
public:
	SubtourElim(GRBVar** xvars, int xn);
	virtual ~SubtourElim();

	GRBVar** vars;
	int n;

protected:
	Subtour sb;

	void callback();
};


class PathTSP_MIP_PathPlanner : public Path_Planner {
public:
	PathTSP_MIP_PathPlanner();
	virtual ~PathTSP_MIP_PathPlanner();
protected:

	/*
	 * Runs the Path-TSP MIP solver for each partition in solution
	 */
	void RunAlgorithm(Solution* solution);

private:

	/*
	 * Solves a Path-TSP problem in partition p using a MIP and Gurobi
	 */
	void PathTSPMIP(Solution* solution, int p);
	// Takes in an integer and returns a string of said integer
	std::string itos(int i);
};
