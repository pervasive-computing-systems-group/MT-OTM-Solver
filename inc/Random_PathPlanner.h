/*
 * Random_PathPlanner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 12, 2021
 *
 * Description: This path-planner creates a set of paths in each partition of the given
 * solution by randomly ordering the vertices to be visited. This was created to be used
 * as a sort of quick path-planning solution for debugging other areas of the code.
 */

#pragma once

#include "Path_Planner.h"

#define DEBUG_RANDOM		0 && DEBUG


class Random_PathPlanner : public Path_Planner {
public:
	Random_PathPlanner();
	virtual ~Random_PathPlanner();
protected:

	/*
	 * Uses a random ordering algorithm to "solve" the Path TSP on each partition in solution
	 */
	void RunAlgorithm(Solution* solution);

private:

	/*
	 * Runs a simple "Random" algorithm that hastily puts the vertices into a path based simply on the
	 * order that they appear in the partition.
	 */
	void RandomAlgorithm(Solution* solution, int p);
};
