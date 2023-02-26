#include "GA_PathPlanner.h"

GA_PathPlanner::GA_PathPlanner() {
	// Seed rand()
	srand(time(NULL));
}

GA_PathPlanner::~GA_PathPlanner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Uses a genetic algorithm to solve the Path TSP on each partition in solution
 */
void GA_PathPlanner::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\nRunning Genetic Algorithm for Path-Planning\n");
	for(int i = 0; i < solution->m_nM; i++) {
		// Run genetic algorithm
		GeneticAlgorithm(solution, i, POPULATION_SIZE, NUM_GENERATIONS, ELITE_PICKS);
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************

// operator overload to compare two T_GASolution
bool operator<(const T_GASolution &a, const T_GASolution &b) {
	return a.fitness < b.fitness;
}

// Debug function
bool hasDuplicates(std::vector<int>& route) {
	for(long unsigned int i = 0; i < route.size(); i++) {
		for(long unsigned int j = (i + 1); j < route.size(); j++) {
			if(route[i] == route[j]) {
				printf(" found dup: %d, %d", route[i], route[j]);
				return true;
			}
		}
	}

	return false;
}

// Runs a simple genetic algorithm that solves Path TSP in each partitioning in pathSolution
void GA_PathPlanner::GeneticAlgorithm(Solution* solution, int p, int nPopSize, int num_gen, int elite_pick) {
	std::priority_queue<T_GASolution> population;

	// Track which vertices might be a "best place to start" and "best place to end"
	int best_first = 0;
	int best_second = 0;
	float bf_dist = std::numeric_limits<float>::max();
	int best_last = 0;
	int best_penultimate = 0;
	float bl_dist = std::numeric_limits<float>::max();

	Vertex* depot_v = (solution->m_pVertexData + solution->m_nN + solution->m_nM + p);
	Vertex* terminal_v = (solution->m_pVertexData + solution->m_nN + p);

	// Add every way-point from this partition of solution to vList
	std::vector<Vertex*> vList;	// List of indexes into the partition map
	for(long unsigned int j = 0, l = 0; j < solution->m_mPartitions.at(p).size(); j++) {
		if(solution->m_mPartitions.at(p).at(j)->eVType == E_VertexType::e_Destination) {
			vList.push_back(solution->m_mPartitions.at(p).at(j));
			// Check if this vertex is a better "Best First" than the previous found best
			if(solution->m_mPartitions.at(p).at(j)->GetDistanceTo(depot_v) < bf_dist) {
				// Make this vertex the best first
				best_second = best_first;
				best_first = l;
				bf_dist = solution->m_mPartitions.at(p).at(j)->GetDistanceTo(depot_v);
			}
			// Check if better than previous best last place pick
			if(solution->m_mPartitions.at(p).at(j)->GetDistanceTo(terminal_v) < bl_dist) {
				// Make this vertex the best last
				best_penultimate = best_last;
				best_last = l;
				bl_dist = solution->m_mPartitions.at(p).at(j)->GetDistanceTo(terminal_v);
			}
			l++;
		}
	}

	// Verify that we don't have the same best-first and best-last
	if(best_first == best_last) {
		if(best_first != best_penultimate) {
			best_last = best_penultimate;
		}
		else if(best_last != best_second) {
			best_first = best_second;
		}
		else {
			// There are no good solutions to this dilemma, just make something up...
			best_first = 0;
			best_last = vList.size() - 1;
		}
	}

	int partition_dest_size = vList.size();

	// Sanity print
	if(SANITY_PRINT)
		printf("Genetic Algorithm: partition %d, number of destinations = %d\n", p, partition_dest_size);

	// Create initial population
	for(int i = 0; i < nPopSize; i++) {
		// Initial solution (empty route)
		T_GASolution temp_solution;
		// List of vertices to put into solution
		std::list<int> tempList;
		// Random number to give this specimen the Best-First
		float chanceBestFirst = ((float) rand()/RAND_MAX);
		// Random number to give this specimen the Best-Last
		float chanceBestLast = ((float) rand()/RAND_MAX);
		// Fill tempList based on above probabilities
		if((chanceBestFirst < PERCENT_BEST_FIRST) && (chanceBestLast < PERCENT_BEST_LAST)) {
			// Start this specimen with the best first and last pick
			for(int j = 0; j < partition_dest_size; j++) {
				if((j != best_first) && (j != best_last)) {
					tempList.push_back(j);
				}
			}
			// Give this specimen the best-first pick
			temp_solution.mRoute.push_back(best_first);
		}
		else if(chanceBestFirst < PERCENT_BEST_FIRST) {
			// Start this specimen with the best first
			for(int j = 0; j < partition_dest_size; j++) {
				if(j != best_first) {
					tempList.push_back(j);
				}
			}
			// Give this specimen the best-first pick
			temp_solution.mRoute.push_back(best_first);
		}
		else if(chanceBestLast < PERCENT_BEST_LAST) {
			// Start this specimen with the best last
			for(int j = 0; j < partition_dest_size; j++) {
				if(j != best_last) {
					tempList.push_back(j);
				}
			}
		}
		else { // Start this specimen normally (completely random)
			// Add indices of vList into a temporary list
			for(int j = 0; j < partition_dest_size; j++) {
				tempList.push_back(j);
			}
		}

		// Sanity print
		if(DEBUG_GA)
			printf("\nSolution: %d\n", i);

		// Randomly add vertices to the solution
		while(!tempList.empty()) {
			// Pick a random index in vList
			int pick = rand() % tempList.size();
			// Add that vertex to the route and remove it from the list
			std::list<int>::iterator it = tempList.begin();
			advance(it, pick);
			temp_solution.mRoute.push_back(*it);
			// Sanity print
			if(DEBUG_GA)
				printf(" picked: %d (#%d)\n", *it, pick);
			tempList.erase(it);
		}

		if(chanceBestLast < PERCENT_BEST_LAST) {
			// Give specimen best last
			temp_solution.mRoute.push_back(best_last);
		}

		// Sanity print, check that we didn't mess this up...
		if(DEBUG_GA) {
			printf(" ");
			for(int i : temp_solution.mRoute) {
				printf("%d ", i);
			}
			printf("\n");

			if((int)temp_solution.mRoute.size() != partition_dest_size) {
				printf("[ERROR] : Bad initial solution size (1)!\n Give Best-Frist: %d\n Give Best-Last: %d\n",
						(chanceBestFirst < PERCENT_BEST_FIRST), (chanceBestLast < PERCENT_BEST_LAST));
				exit(0);
			}
			else if(hasDuplicates(temp_solution.mRoute)) {
				printf("[ERROR] : Initial solution has duplicates (1)!\n Give Best-Frist: %d\n Give Best-Last: %d\n",
						(chanceBestFirst < PERCENT_BEST_FIRST), (chanceBestLast < PERCENT_BEST_LAST));
				exit(0);
			}
		}

		temp_solution.fitness = ga_fitness(solution, &temp_solution, p);

		// Add this solution to the initial population
		population.push(temp_solution);
	}

	if(DEBUG_GA)
		printf("\n");


	// Save results to file
	FILE * pProgressFile;
	pProgressFile = fopen("GA_Data/progress_output.dat", "w");
	FILE * pEliteFile;
	pEliteFile = fopen("GA_Data/elite_output.dat", "w");

	// Run generations
	if(SANITY_PRINT)
		printf("Run generations\n");
	for(int i = 0; i < num_gen; i++) {
		// Select population to keep (including elite picks)
		std::vector<T_GASolution> new_gen;
		// Select elite picks to keep
		for(int j = 0; (j < elite_pick) && (!population.empty()); j++) {
			new_gen.push_back(population.top());
			// Sanity print
			if(DEBUG_GA)
				printf(" pushed: %f\n", population.top().fitness);
			population.pop();
		}

		// Select the rest using Roulette Wheel Selection (set-up)
		std::list<T_GASolution> fitness_picks;
		float total_fitness = 0;
		int num_fit_picks = 0;
		while(!population.empty()) {
			T_GASolution top = population.top();
			top.fit_sum_lower = total_fitness;
			total_fitness += top.fitness;
			top.fit_sum_upper = total_fitness;
			// Sanity print
			if(DEBUG_GA)
				printf(" added: %f, [%f, %f]\n", top.fitness, top.fit_sum_lower, top.fit_sum_upper);

			fitness_picks.push_back(top);
			population.pop();
			num_fit_picks++;
		}

		int sanity_count = 0;

		// Run Roulette Wheel Selection
		while(new_gen.size() < (long unsigned int)(3*nPopSize)/5) {
			// Randomly add new specimens into the population with some probability
			float addNewSpec = ((float) rand()/RAND_MAX);
			float probOfNewSpec = PERCENT_DIVERSITY*((float)NUM_GENERATIONS - i)/NUM_GENERATIONS;

			// Check if we should create a new specimen
			if(addNewSpec <= probOfNewSpec) {
				// Initial solution (empty route)
				T_GASolution temp_solution;
				// Add a new specimen
				std::list<int> tempList;

				// Random number to give this specimen the Best-First
				float chanceBestFirst = ((float) rand()/RAND_MAX);
				// Random number to give this specimen the Best-Last
				float chanceBestLast = ((float) rand()/RAND_MAX);
				// Fill tempList based on above probabilities
				if((chanceBestFirst < PERCENT_BEST_FIRST) && (chanceBestLast < PERCENT_BEST_LAST)) {
					// Start this specimen with the best first and last pick
					for(int j = 0; j < partition_dest_size; j++) {
						if((j != best_first) && (j != best_last)) {
							tempList.push_back(j);
						}
					}
					// Give this specimen the best-first pick
					temp_solution.mRoute.push_back(best_first);
				}
				else if(chanceBestFirst < PERCENT_BEST_FIRST) {
					// Start this specimen with the best first
					for(int j = 0; j < partition_dest_size; j++) {
						if(j != best_first) {
							tempList.push_back(j);
						}
					}
					// Give this specimen the best-first pick
					temp_solution.mRoute.push_back(best_first);
				}
				else if(chanceBestLast < PERCENT_BEST_LAST) {
					// Start this specimen with the best last
					for(int j = 0; j < partition_dest_size; j++) {
						if(j != best_last) {
							tempList.push_back(j);
						}
					}
				}
				else { // Start this specimen normally (completely random)
					// Add indices of vList into a temporary list
					for(int j = 0; j < partition_dest_size; j++) {
						tempList.push_back(j);
					}
				}

				// Randomly add vertices to the solution
				while(!tempList.empty()) {
					// Pick a random index in vList
					int pick = rand() % tempList.size();
					// Add that vertex to the route and remove it from the list
					std::list<int>::iterator it = tempList.begin();
					advance(it, pick);
					temp_solution.mRoute.push_back(*it);
					// Sanity print
					if(DEBUG_GA)
						printf(" picked: %d (#%d)\n", *it, pick);
					tempList.erase(it);
				}

				if(chanceBestLast < PERCENT_BEST_LAST) {
					// Give specimen best last
					temp_solution.mRoute.push_back(best_last);
				}

				// Sanity print, check that we didn't mess this up...
				if(1) { // TODO: here!!
//					printf(" ");
//					for(int i : temp_solution.mRoute) {
//						printf("%d ", i);
//					}
//					printf("\n");

					if((int)temp_solution.mRoute.size() != partition_dest_size) {
						printf("[ERROR] : Bad initial solution size (2)!\n Give Best-Frist: %d\n Give Best-Last: %d\n",
								(chanceBestFirst < PERCENT_BEST_FIRST), (chanceBestLast < PERCENT_BEST_LAST));
						exit(0);
					}
					else if(hasDuplicates(temp_solution.mRoute)) {
						printf("[ERROR] : Initial solution has duplicates (2)!\n Give Best-Frist: %d\n Give Best-Last: %d\n",
								(chanceBestFirst < PERCENT_BEST_FIRST), (chanceBestLast < PERCENT_BEST_LAST));
						exit(0);
					}
				}

				temp_solution.fitness = ga_fitness(solution, &temp_solution, p);

				// Add this solution to new generation
				new_gen.push_back(temp_solution);
				sanity_count++;
			}
			else { // Select from current population
				// Random fitness to search for
				float pick = ((float) rand()/RAND_MAX) * total_fitness;
				// Sanity print
				if(DEBUG_GA)
					printf(" check: %f\n", pick);
				// Search for the solution that covers this fitness range
				std::list<T_GASolution>::iterator itSol = fitness_picks.begin();
				while(itSol != fitness_picks.end()) {
					if((itSol->fit_sum_lower < pick) && (itSol->fit_sum_upper > pick)) {
						// Found our match, remove from fitness_picks and add to new generation
						new_gen.push_back(*itSol);
						// Sanity print
						if(DEBUG_GA)
							printf("  pushed: %f\n", itSol->fitness);
						fitness_picks.erase(itSol);
						break;
					}
					if(itSol->fit_sum_lower > pick) {
						// Must have removed the desired solution already, stop searching
						break;
					}
					itSol++;
				}
			}
		}

		// Sanity print
		if(DEBUG_GA)
			printf("Iteration: %d, Added %d new specimens\n", i, sanity_count);

		int top_picks_size = new_gen.size();

		// Perform cross-over (add old gen + children)
		for(int j = 0; j < nPopSize/4; j++) {
			// Pick two candidates to cross
			int pick1 = rand() % top_picks_size;
			int pick2 = rand() % top_picks_size;
			while(pick1 == pick2) {
				pick2 = rand() % top_picks_size;
			}

			// Pick a range of the gene to cross over in new child
			int seqStart = rand() % partition_dest_size;
			int seqEnd = rand() % partition_dest_size;

			// Sanity print
			if(DEBUG_GA) {
				printf("\nSwapping %d -> %d from\n pick1: ", seqStart, seqEnd);
				for(int i : new_gen[pick1].mRoute) {
					printf("%d ", i);
				}
				printf("\n pick2: ");
				for(int i : new_gen[pick2].mRoute) {
					printf("%d ", i);
				}
				printf("\n");

			}

			// Create new child
			T_GASolution child;
			if(seqEnd < seqStart) {
				int temp = (seqEnd + partition_dest_size);

				std::list<int> wp_cross;
				for(int k = seqStart; k <= temp; k++) {
					wp_cross.push_back(new_gen[pick1].mRoute[k % partition_dest_size]);
				}

				for(int k = 0; k <= seqEnd; k++) {
					child.mRoute.push_back(new_gen[pick1].mRoute[k]);
				}
				for(int k = 0, l = 0; ((long unsigned int)k < new_gen[pick1].mRoute.size()) && (l < (seqStart - seqEnd + 1)); k++) {
					if(!inList(wp_cross, new_gen[pick2].mRoute[k])) {
						child.mRoute.push_back(new_gen[pick2].mRoute[k]);
						l++;
					}
				}
				for(int k = seqStart; (long unsigned int)k < new_gen[pick1].mRoute.size(); k++) {
					child.mRoute.push_back(new_gen[pick1].mRoute[k]);
				}
			}
			else if(seqEnd > seqStart) {
				std::list<int> wp_cross;
				for(int k = seqStart; k <= seqEnd; k++) {
					wp_cross.push_back(new_gen[pick1].mRoute[k % partition_dest_size]);
				}

				int l = 0;
				// If seqStart is at the beginning, start copying over now
				if(l == seqStart) {
					for(int m = seqStart; m <= seqEnd; m++) {
						child.mRoute.push_back(new_gen[pick1].mRoute[m]);
						l++;
					}
				}
				for(int k = 0; (long unsigned int)k < new_gen[pick1].mRoute.size(); k++) {
					if(!inList(wp_cross, new_gen[pick2].mRoute[k])) {
						child.mRoute.push_back(new_gen[pick2].mRoute[k]);
						l++;
					}
					// Insert the crossed-over line from pick 1
					if(l == seqStart) {
						for(int m = seqStart; m <= seqEnd; m++) {
							child.mRoute.push_back(new_gen[pick1].mRoute[m]);
							l++;
						}
					}
				}


//				for(int k = seqStart; k <= seqEnd; k++) {
//					child.mRoute.push_back(new_gen[pick1].mRoute[k]);
//				}
//				for(int k = 0, l = (seqEnd + 1); ((long unsigned int)k < new_gen[pick1].mRoute.size()) && ((long unsigned int)l < new_gen[pick1].mRoute.size()); k++) {
//					if(!inList(wp_cross, new_gen[pick2].mRoute[k])) {
//						child.mRoute.push_back(new_gen[pick2].mRoute[k]);
//						l++;
//					}
//				}

			}
			else { // seqEnd == seqStart
				int wp_cross = new_gen[pick1].mRoute[seqStart];

				int l = 0;
				if(l == seqStart) {
					child.mRoute.push_back(wp_cross);
					l++;
				}
				for(int k = 0; (long unsigned int)k < new_gen[pick2].mRoute.size(); k++) {
					if(new_gen[pick2].mRoute[k] != wp_cross) {
						child.mRoute.push_back(new_gen[pick2].mRoute[k]);
						l++;
					}

					if(l == seqStart) {
						child.mRoute.push_back(wp_cross);
						l++;
					}
				}
			}

			if(DEBUG_GA) {
				printf(" child: ");
				for(int i : child.mRoute) {
					printf("%d ", i);
				}
				printf("\n");

				if((int)child.mRoute.size() != partition_dest_size) {
					printf("[ERROR] : (1) Bad child size!\n");
					exit(0);
				}
				else if(hasDuplicates(child.mRoute)) {
					printf("[ERROR] : (1) Child has duplicates!\n");
					exit(0);
				}
			}

			// Add child to new generation
			child.fitness = ga_fitness(solution, &child, p);
			new_gen.push_back(child);
		}

		if(DEBUG_GA) {
			// Sanity print
			printf("\nNew Generation size: %ld\n", new_gen.size());
		}

		// Mutate (duplicate some from new population)
		while(new_gen.size() < (long unsigned int)nPopSize) {
			// Pick random solution to mutate
			int parent = rand() % new_gen.size();

			// Create mutant new child
			T_GASolution child;
			for(int n : new_gen[parent].mRoute) {
				child.mRoute.push_back(n);
			}

			if(DEBUG_GA) {
				printf("Mutant child: ");
				for(int i : child.mRoute) {
					printf("%d ", i);
				}
				printf("\n");
			}

			int gene1 = rand() % child.mRoute.size();
			int gene2 = rand() % child.mRoute.size();

			int temp = child.mRoute[gene1];
			child.mRoute[gene1] = child.mRoute[gene2];
			child.mRoute[gene2] = temp;

			if(DEBUG_GA) {
				printf(" ");
				for(int i : child.mRoute) {
					printf("%d ", i);
				}
				printf("\n");

				if((int)child.mRoute.size() != partition_dest_size) {
					printf("[ERROR] : (2) Bad child size!\n");
					exit(0);
				}
				else if(hasDuplicates(child.mRoute)) {
					printf("[ERROR] : (2) Child has duplicates!\n");
					exit(0);
				}
			}

			// Add child to new generation
			child.fitness = ga_fitness(solution, &child, p);
			new_gen.push_back(child);
		}

		float tot_fitness = 0;
		float gen_size = 0;

		// Push back into population
		for(T_GASolution sol : new_gen) {
			population.push(sol);

			tot_fitness += sol.fitness;
			gen_size += 1.0;
		}

		fprintf(pProgressFile, "%d %f\n", i, tot_fitness/gen_size);
		fprintf(pEliteFile, "%d %f\n", i, population.top().fitness);
	}

	// Close path file
	fclose(pProgressFile);

	// Finished running genetic algorithm, pull off best solution (at the top of the queue)
	T_GASolution bestSol = population.top();

	if(SANITY_PRINT) {
		// Sanity print
		printf("Best Fitness: %f (%f)\n\n", bestSol.fitness, 1.0/bestSol.fitness);
	}

	// Add this solution to pathSolution
	int terminal = solution->m_nN + p;
	int depot = solution->m_nN + solution->m_nM + p;
	int v = vList.at(bestSol.mRoute[0])->nID;

	// Add depot -> v
	solution->m_pAdjMatrix[v][depot] = true;

	// Cycle through route
	for(long unsigned int i = 1; i < bestSol.mRoute.size(); i++) {
		int u = v;
		v = vList.at(bestSol.mRoute[i])->nID;
		// Add u -> v
		if(u < v) {
			solution->m_pAdjMatrix[u][v] = true;
		}
		else {
			solution->m_pAdjMatrix[v][u] = true;
		}
	}

	// Add v -> terminal
	solution->m_pAdjMatrix[v][terminal] = true;
}

// Fitness function used by GeneticAlgorithm()
float GA_PathPlanner::ga_fitness(Solution* pathSolution, T_GASolution* temp_solution, int index) {
	float rt_length = 0;

	int terminal = pathSolution->m_nN + index;
	int depot = pathSolution->m_nN + pathSolution->m_nM + index;
	int v = temp_solution->mRoute[0];

	// Add depot -> v0
	rt_length += sqrt(pow((pathSolution->m_pVertexData[depot].fX - pathSolution->m_mPartitions.at(index).at(v)->fX), 2)
			+ pow((pathSolution->m_pVertexData[depot].fY - pathSolution->m_mPartitions.at(index).at(v)->fY), 2));

	// Cycle through route
	for(long unsigned int i = 1; i < temp_solution->mRoute.size(); i++) {
		int u = v;
		v = temp_solution->mRoute[i];
		// Update route length
		rt_length += sqrt(pow((pathSolution->m_mPartitions.at(index).at(u)->fX - pathSolution->m_mPartitions.at(index).at(v)->fX), 2)
				+ pow((pathSolution->m_mPartitions.at(index).at(u)->fY - pathSolution->m_mPartitions.at(index).at(v)->fY), 2));
	}

	// Add the distance from the final vertex, v, to the terminal
	rt_length += sqrt(pow((pathSolution->m_pVertexData[terminal].fX - pathSolution->m_mPartitions.at(index).at(v)->fX), 2)
			+ pow((pathSolution->m_pVertexData[terminal].fY - pathSolution->m_mPartitions.at(index).at(v)->fY), 2));

	if(DEBUG_GA) {
		// Sanity print
		printf(" fitness: %f\n",rt_length);
	}

	if(rt_length == 0) {
		// Something went wrong...
		return 0;
	}
	else {
		return 1/rt_length;
	}
}

// Check to see if n is in list ls
bool GA_PathPlanner::inList(std::list<int> &ls, int n) {
	for(int i : ls) {
		if(i == n) {
			return true;
		}
	}

	return false;
}
