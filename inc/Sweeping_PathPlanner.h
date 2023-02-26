/*
 * Sweeping_PathPlanner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 13, 2021
 *
 * Description:
 */

#pragma once

#include <math.h>
#include <stack>

#include "Path_Planner.h"
#include "Sweeping_Approach.h"

#define DEBUG_SWEEP_PP	0 && DEBUG


class Sweeping_PathPlanner : public Path_Planner, public Sweeping_Approach {
public:
	Sweeping_PathPlanner();
	virtual ~Sweeping_PathPlanner();
protected:

	/*
	 * Uses a random ordering algorithm to "solve" the Path TSP on each partition in solution
	 */
	void RunAlgorithm(Solution* solution);

private:

	/*
	 * Randomly orders the vertices in the given partition p and saves the resulting path in solution
	 */
	void RandomAlgorithm(Solution* solution, int p);
	/*
	 * Randomly orders the vertices in the given partition p and saves the resulting path in solution
	 */
	void solveGivenDirection(Solution* solution, int p, std::list<Vertex*> &routeList);
};
