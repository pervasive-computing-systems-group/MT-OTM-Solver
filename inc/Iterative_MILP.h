/*
 * Iterative_MILP.h
 *
 * Created by:	Jonathan Diller
 * On: 			Feb 03, 2022
 *
 * Description: The Iterative_MIP class iteratively runs a MIP designed to solve the
 * FMDMTHPP on the Gurboi solver. The MIP model tries to minimize the total distance
 * of the tours while also minimizing a penalty for letting any single tour become much
 * larger than the average tour size.
 */

#pragma once

#include <stdlib.h>
#include <map>
#include <queue>
#include <list>
#include <stack>

#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"
#include "MILP_FMDMTHPP.h"


#define DEBUG_IT_MILP			0 || DEBUG

class Iterative_MILP : public Hybrid_Planner {
public:
	Iterative_MILP();
	virtual ~Iterative_MILP();

protected:
	/*
	 * Runs a hybrid iterative approach with MIP
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
};
