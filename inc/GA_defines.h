/*
 * GA_defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 25, 2021
 *
 * Description: Header file for general genetic algorithm defines
 */

#pragma once


#include <vector>
#include <limits>


#define POPULATION_SIZE		1000
#define NUM_GENERATIONS		10000
#define ELITE_PICKS			200
#define PERCENT_DIVERSITY	0.25
#define PERCENT_BEST_FIRST	0.2
#define PERCENT_BEST_LAST	0.2


struct T_GASolution{
	std::vector<int> mRoute;
	float fitness;
	float fit_sum_lower, fit_sum_upper;

	T_GASolution() {
		fitness = std::numeric_limits<float>::max();
		fit_sum_lower = -1;
		fit_sum_upper = -1;
	}

	T_GASolution(const T_GASolution &gaSol) {
		fitness = gaSol.fitness;
		fit_sum_lower = gaSol.fit_sum_lower;
		fit_sum_upper = gaSol.fit_sum_upper;
		for(int i : gaSol.mRoute) {
			mRoute.push_back(i);
		}
	}
};
