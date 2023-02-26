/*
 * GA_PathPlanner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 01, 2021
 *
 * Description:
 */

#pragma once

#include "Path_Planner.h"
#include "GA_defines.h"

#define DEBUG_GA			0 && DEBUG


class GA_PathPlanner : public Path_Planner {
public:
	GA_PathPlanner();
	virtual ~GA_PathPlanner();
protected:

	/*
	 * Uses a genetic algorithm to solve the Path TSP on each partition in solution
	 */
	void RunAlgorithm(Solution* solution);

private:

	/*
	 * Runs a simple genetic algorithm to find a Path TSP route in partition p of the given solution.
	 */
	void GeneticAlgorithm(Solution* solution, int p, int nPopSize, int num_gen, int elite_pick);
	// Fitness function used by GeneticAlgorithm(), where index is the depot/terminal number
	float ga_fitness(Solution* solution, T_GASolution* temp_solution, int index);
	// Check to see if n is in list ls
	bool inList(std::list<int> &ls, int n);
};
