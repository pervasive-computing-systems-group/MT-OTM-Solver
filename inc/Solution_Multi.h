/*
 * Solution_Multi.h
 *
 * Created by:	Jonathan Diller
 * On: 			Jun 22, 2022
 *
 * Description:
 */

#pragma once

#include <map>
#include <vector>
#include <sstream>
#include <fstream>
#include <limits>
#include <list>
#include <algorithm>

#include "Solution.h"

// Allows for solution specific debugging
#define DEBUG_MULT_SOLUTION		0 || DEBUG
#define PRINT_MULT_SOLUTION		0 || DEBUG


class Solution_Multi: public Solution {
public:
	Solution_Multi(std::string graph_path, int m, int numUAVs, double ct_guess);
	Solution_Multi(std::string graph_path, int m, int numUAVs, double* A);
	virtual ~Solution_Multi();

	/*
	 * Determine the amount of time required for a UAV to run this solution.
	 *
	 * Setting bLagrangianRelaxation to true will cause the algorithm to use Lagrangian relaxation
	 * to relax the max distance constraint. In this case, any partition leg longer than DIST_MAX
	 * will be run at the optimal velocity (V_OPT) with an additional time penalty based on the
	 * distance that the individual leg over-shot DIST_MAX.
	 */
//	virtual float TimeToRunSolution(E_VelocityFlag fixedVelocityFlag = E_VelocityFlag::e_NotFixed, bool bLagrangianRelaxation = false);


	/*
	 * Returns the time required to run this multi-UAV solution. The result is only valid if the solution is
	 * consistent, which isn't guaranteed.
	 */
	double TimeToRunMultiSolution();
	/*
	 * Returns the actual time to run subtour m based on the distance of the subtour and the given velocity flag
	 */
	double GetSubtourTime(int m, E_VelocityFlag fixedVelocityFlag = E_VelocityFlag::e_NotFixed);

	/*
	 * Number of UAVs to deploy. NOTE: This is currently not actually used and most solvers assume the number of UAVs is 1.
	 */
	int m_nNumUAVs;

protected:

private:
	// Build solution
	void classConstructor(std::string graph_path, int m, int numUAVs, double* A);
	// Place depot/terminals
	void place_depotsAndTerminals(double* A);
};
