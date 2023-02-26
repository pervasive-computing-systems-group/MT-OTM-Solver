/*
 * GA_ISE_PathTSP.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 25, 2021
 *
 * Description: This class performs a basic Genetic Algorithm with Iterative Search-space Expansion (GA-ISE)
 */

#pragma once

#include <queue>

#include "GA_defines.h"
#include "Vertex.h"
#include "Solution.h"

#define DEBUG_GA_ISE			0 || DEBUG

// GA-ISE parameters
#define GAISE_PERCNT_ELITISM		0.25	// % of top solutions that get picked in crosses
#define GAISE_PERCNT_EPOCH_CROSS	0.5		// % of solutions that cross into new epoch
#define GAISE_PERCNT_GEN_CROSS		0.3		// % of solutions that cross into new generation
#define GAISE_POPULATION_SIZE		50		// total population size
#define GAISE_NUM_GEN_FACTOR		10		// factor that determines number of generations to run (= factor * search-space size)
#define GAISE_PERCNT_CROSSOVER		0.35	// % of population to cross-over for new generation

struct T_GAISE_node {
	Vertex* pV;
	float fDepotDist;

	T_GAISE_node() {
		pV = NULL;
		fDepotDist = std::numeric_limits<float>::max();
	}

	T_GAISE_node(Vertex* v) {
		pV = v;
		fDepotDist = std::numeric_limits<float>::max();
	}

	T_GAISE_node(const T_GAISE_node &gaiseNode) {
		pV = gaiseNode.pV;
		fDepotDist = gaiseNode.fDepotDist;
	}
};

struct T_GAISESolution{
	// Route for this solution. NOTE: does not include depot or terminal vertices
	std::vector<Vertex*> mRoute;
	float fitness;
	float fit_sum_lower, fit_sum_upper;

	T_GAISESolution() {
		fitness = std::numeric_limits<float>::max();
		fit_sum_lower = -1;
		fit_sum_upper = -1;
	}

	T_GAISESolution(const T_GAISESolution &gaSol) {
		fitness = gaSol.fitness;
		fit_sum_lower = gaSol.fit_sum_lower;
		fit_sum_upper = gaSol.fit_sum_upper;
		for(Vertex* i : gaSol.mRoute) {
			mRoute.push_back(i);
		}
	}
};


class GA_ISE_PathTSP {
public:
	GA_ISE_PathTSP();
	virtual ~GA_ISE_PathTSP();
	/*
	 * Runs a genetic algorithm with iterative search-space expansion
	 */
	void GeneticAlgorithm_ISE(Solution* solution, int p, int nInitialSSpaceSize, T_GAISESolution &bestSol);

protected:
private:
	// Fitness function used by GeneticAlgorithm(), where index is the depot/terminal number
	float ga_fitness(Solution* pathSolution, T_GAISESolution &temp_solution, int index);
	// Check to see if n is in list ls
	bool inList(std::list<Vertex*> &ls, Vertex* v);
	// Selects a set of parents and stores them in new_gen
	void selectParents(std::priority_queue<T_GAISESolution> population, std::vector<T_GAISESolution> &new_gen, int num_parents);
	// Creates a new population using selected_group that is augmented with the given vertex v
	void createNewAugmentedPop(Solution* solution, std::vector<T_GAISESolution> &selected_group,
			std::priority_queue<T_GAISESolution> &population, Vertex*, int p);
	// Create child solution using random cross-over between parent 1 and 2
	void performCrossOver(T_GAISESolution* parent1, T_GAISESolution* parent2, T_GAISESolution* child);
	// Create child solution using a random mutation of parent
	void performMutation(T_GAISESolution* parent, T_GAISESolution* child);
};
