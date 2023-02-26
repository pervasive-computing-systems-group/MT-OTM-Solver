/*
 * Hybrid_Planner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 19, 2021
 *
 * Description:
 */

#pragma once

#include <stdlib.h>

#include "defines.h"
#include "Solution.h"

class Hybrid_Planner {
public:
	Hybrid_Planner();
	virtual ~Hybrid_Planner();

	/*
	 * Runs the path planning algorithm
	 */
	void RunHybridAlgorithm(Solution* pathSolution);
protected:
	/*
	 * Runs the partitioning algorithm that is defined in the child class
	 */
	virtual void RunAlgorithm(Solution* pathSolution) = 0;

private:
};
