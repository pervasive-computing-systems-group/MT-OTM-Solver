/*
 * EmptyGraph_Planner.h
 *
 * Created by:	Jonathan Diller
 * On: 			May 17, 2022
 *
 * Description: This type of planner is used for solving the MT-OTM problem on
 * graphs don't that don't have a value for 'm', a.k.a. "empty".
 */

#pragma once

#include <stdlib.h>

#include "defines.h"
#include "Solution.h"

class EmptyGraph_Planner {
public:
	EmptyGraph_Planner();
	virtual ~EmptyGraph_Planner();

	/*
	 * Runs the path planning algorithm
	 */
	void RunEGAlgorithm(Solution* pathSolution);
protected:
	/*
	 * Runs the empty-graph type algorithm that is defined in the child class
	 */
	virtual void RunAlgorithm(Solution* pathSolution) = 0;

private:
};
