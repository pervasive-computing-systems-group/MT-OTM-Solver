#include "GA_ISE_PathTSP.h"

GA_ISE_PathTSP::GA_ISE_PathTSP() {
	// Seed rand()
	srand(time(NULL));
}

GA_ISE_PathTSP::~GA_ISE_PathTSP() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

// operator overload to compare two T_GAISE_node
bool operator<(const T_GAISE_node &a, const T_GAISE_node &b) {
	return a.fDepotDist > b.fDepotDist;
}

// operator overload to compare two T_GAISE_node
// If the overriden op< function bothers you too much, use:
//  std::priority_queue<Node, std::vector<Node>, bool (*)(Node, Node)> openSet(Compare)
bool operator<(const T_GAISESolution &a, const T_GAISESolution &b) {
	return a.fitness < b.fitness;
}

// Debug function
bool hasDuplicates(std::vector<Vertex*>& route) {
	for(long unsigned int i = 0; i < route.size(); i++) {
		for(long unsigned int j = (i + 1); j < route.size(); j++) {
			if(route[i] == route[j]) {
				printf(" found dup: %d, %d", route[i]->nID, route[j]->nID);
				return true;
			}
		}
	}

	return false;
}

// Runs a genetic algorithm with iterative search-space expansion
void GA_ISE_PathTSP::GeneticAlgorithm_ISE(Solution* solution, int p, int nInitialSSpaceSize, T_GAISESolution &bestSol) {
	if(SANITY_PRINT)
		printf("** Planning partition %d **\n", p);

	// Priority queue used to feed vertices into GAISE algorithm
	std::priority_queue<T_GAISE_node> qSSpaceCompliment;

	// Prioritize all vertices in this partition based on distance from the depot
	for(Vertex* v : solution->m_mPartitions.at(p)) {
		if(v->eVType == E_VertexType::e_Destination) {
			// Find distance to depot and add to priority queue
			T_GAISE_node temp(v);
			temp.fDepotDist = solution->GetDepotOfPartion(p)->GetDistanceTo(v);
			qSSpaceCompliment.push(temp);
		}
	}

	// List of initial vertices to put into solutions
	std::list<Vertex*> initSSpaceList;

	// Add indices of vList into a temporary list
	if((long unsigned int)nInitialSSpaceSize > qSSpaceCompliment.size()) {
		nInitialSSpaceSize = (int)qSSpaceCompliment.size();
	}
	for(int j = 0; j < nInitialSSpaceSize; j++) {
		initSSpaceList.push_back(qSSpaceCompliment.top().pV);
		qSSpaceCompliment.pop();
	}

	// Create initial population
	std::priority_queue<T_GAISESolution> population;
	for(int i = 0; i < GAISE_POPULATION_SIZE; i++) {
		// Initial solution (empty route)
		T_GAISESolution temp_solution;
		// List of vertices to put into solution
		std::list<Vertex*> tempList(initSSpaceList);

		// Sanity print
		if(DEBUG_GA_ISE)
			printf("\nSolution: %d\n", i);

		// Randomly add vertices to the solution
		while(!tempList.empty()) {
			// Pick a random index in vList
			int pick = rand() % tempList.size();
			// Add that vertex to the route and remove it from the list
			std::list<Vertex*>::iterator it = tempList.begin();
			advance(it, pick);
			temp_solution.mRoute.push_back(*it);
			// Sanity print
			if(DEBUG_GA_ISE)
				printf(" picked: %d (#%d)\n", (*it)->nID, pick);
			tempList.erase(it);
		}

		// Sanity print, check that we didn't mess this up...
		if(DEBUG_GA_ISE) {
			printf(" ");
			for(Vertex* i : temp_solution.mRoute) {
				printf("%d ", i->nID);
			}
			printf("\n");

			if((int)temp_solution.mRoute.size() != nInitialSSpaceSize) {
				printf("[ERROR] : Bad initial solution size (1)!\n");
				exit(0);
			}
			else if(hasDuplicates(temp_solution.mRoute)) {
				printf("[ERROR] : Initial solution has duplicates (1)!");
				exit(0);
			}
		}

		temp_solution.fitness = ga_fitness(solution, temp_solution, p);

		// Add this solution to the initial population
		population.push(temp_solution);
	}

	// Save results to file
//	FILE * pProgressFile;
//	pProgressFile = fopen("GA_Data/GAISE_progress_output.dat", "w");
//	FILE * pEliteFile;
//	pEliteFile = fopen("GA_Data/GAISE_elite_output.dat", "w");

	// Sanity print
	if(DEBUG_GA_ISE)
		printf("\nStart generation/Expanding search space\n");

	// Start iteratively expanding the search space
	while(!qSSpaceCompliment.empty()) {
		// Select population to keep (including elite picks)
		std::vector<T_GAISESolution> epoch_cross_picks;
		selectParents(population, epoch_cross_picks, (GAISE_POPULATION_SIZE * GAISE_PERCNT_EPOCH_CROSS));

		// Wipe-out previous population
		while(!population.empty()) {
			population.pop();
		}

		// Add a new way-point to the solution space
		Vertex* v = qSSpaceCompliment.top().pV;
		qSSpaceCompliment.pop();
		int new_gen_rt_size = epoch_cross_picks.at(0).mRoute.size() + 1;

		// Sanity print
		if(DEBUG_GA_ISE)
			printf("Expanding Search Space to %d\n |search-space compliment| = %ld\n Augmenting with %d\n", new_gen_rt_size,
					qSSpaceCompliment.size(), v->nID);

		// Create a new augmented population using v
		createNewAugmentedPop(solution, epoch_cross_picks, population, v, p);

		// Sanity print
		if(DEBUG_GA_ISE) {
			printf("Size of new Epoch population: %ld\n\n", population.size());
		}

		// Run generations
		if(DEBUG_GA_ISE)
			printf("Run generations\n");
		for(int i = 0; i < (GAISE_NUM_GEN_FACTOR * new_gen_rt_size); i++) {
			// Select population to cross-over
			std::vector<T_GAISESolution> new_gen;
			selectParents(population, new_gen, (GAISE_POPULATION_SIZE * GAISE_PERCNT_GEN_CROSS));

			// Wipe-out previous population
			while(!population.empty()) {
				population.pop();
			}

			// Perform cross-over (add old gen + children)
			int top_picks_size = new_gen.size();
			for(int j = 0; j < (GAISE_POPULATION_SIZE * GAISE_PERCNT_CROSSOVER); j++) {
				// Pick two candidates to cross
				int pick1 = rand() % top_picks_size;
				int pick2 = rand() % top_picks_size;
				while(pick1 == pick2) {
					pick2 = rand() % top_picks_size;
				}

				// Create new child
				T_GAISESolution child;

				// Perform cross-over to create child
				performCrossOver(&new_gen[pick1], &new_gen[pick2], &child);

				// Add child to new generation
				child.fitness = ga_fitness(solution, child, p);
				new_gen.push_back(child);
			}

			// Mutate (duplicate some from new population)
			while(new_gen.size() < GAISE_POPULATION_SIZE) {
				// Pick random solution to mutate
				int parent = rand() % new_gen.size();

				// Create new mutant child
				T_GAISESolution mutant;

				// Mutate child
				performMutation(&new_gen[parent], &mutant);

				// Add child to new generation
				mutant.fitness = ga_fitness(solution, mutant, p);
				new_gen.push_back(mutant);
			}


			float tot_fitness = 0;
			float gen_size = 0;

			// Push back into population
			for(T_GAISESolution sol : new_gen) {
				population.push(sol);

				tot_fitness += sol.fitness;
				gen_size += 1.0;
			}

//			fprintf(pProgressFile, "%d %f\n", i, tot_fitness/gen_size);
//			fprintf(pEliteFile, "%d %f\n", i, population.top().fitness);

			// Sanity print
			if(DEBUG_GA_ISE)
				printf(" Finished generation, best solution: %f\n\n", population.top().fitness);
		}
	}

	// Close files
//	fclose(pProgressFile);
//	fclose(pEliteFile);

	// Finished running genetic algorithm, pull off best solution (at the top of the queue)
	bestSol = population.top();

	// Sanity print
	if(SANITY_PRINT) {
		printf("Best Fitness: %f (%f)\n\n", bestSol.fitness, 1.0/bestSol.fitness);
	}
}


//***********************************************************
// Protected Member Functions
//***********************************************************


//***********************************************************
// Private Member Functions
//***********************************************************


// Fitness function used by GeneticAlgorithm()
float GA_ISE_PathTSP::ga_fitness(Solution* pathSolution, T_GAISESolution &temp_solution, int index) {
	float rt_length = 0;

	// Start with depot
	Vertex* v = pathSolution->GetDepotOfPartion(index);

	// Cycle through route, adding up each u -> v edge
	for(long unsigned int i = 0; i < temp_solution.mRoute.size(); i++) {
		Vertex* u = v;
		v = temp_solution.mRoute[i];
		// Add this edge
		rt_length += u->GetDistanceTo(v);
	}

	// Add the distance from the final vertex, v, to the terminal
	rt_length += pathSolution->GetTerminalOfPartion(index)->GetDistanceTo(v);

	if(DEBUG_GA_ISE) {
		// Sanity print
		printf(" route dist: %f\n",rt_length);
	}

	if(rt_length == 0) {
		// Something went wrong...
		printf("** Fitness is 0! **\n");
		return 0;
	}
	else {
		rt_length = 1/rt_length;
	}

	if(DEBUG_GA_ISE) {
		// Sanity print
		printf(" fitness: %f\n",rt_length);
	}

	return rt_length;
}

// Check to see if n is in list ls
bool GA_ISE_PathTSP::inList(std::list<Vertex*> &ls, Vertex* v) {
	for(Vertex* i : ls) {
		if(i == v) {
			return true;
		}
	}

	return false;
}

// Selects a set of parents and stores them in new_gen
void GA_ISE_PathTSP::selectParents(std::priority_queue<T_GAISESolution> population, std::vector<T_GAISESolution> &new_gen, int num_parents) {
	// Select elite picks to keep
	int num_elite = (int)(population.size() * GAISE_PERCNT_ELITISM);
	if(DEBUG_GA_ISE)
		printf("Picking Elites\n");
	for(int j = 0; (j < num_elite) && (!population.empty()); j++) {
		new_gen.push_back(population.top());
		// Sanity print
		if(DEBUG_GA_ISE)
			printf(" pushed: %f\n", population.top().fitness);
		population.pop();
	}

	// Select the rest using Roulette Wheel Selection (set-up)
	std::list<T_GAISESolution> fitness_picks;
	float total_fitness = 0;
	int num_fit_picks = 0;
	while(!population.empty()) {
		T_GAISESolution top = population.top();
		top.fit_sum_lower = total_fitness;
		total_fitness += top.fitness;
		top.fit_sum_upper = total_fitness;
		// Sanity print
		if(DEBUG_GA_ISE)
			printf(" added: %f, [%f, %f]\n", top.fitness, top.fit_sum_lower, top.fit_sum_upper);

		fitness_picks.push_back(top);
		population.pop();
		num_fit_picks++;
	}

	// Run Roulette Wheel Selection
	while(new_gen.size() < (long unsigned int)num_parents) {
		// Random fitness to search for
		float pick = ((float) rand()/RAND_MAX) * total_fitness;
		// Sanity print
		if(DEBUG_GA_ISE)
			printf(" check: %f\n", pick);
		// Search for the solution that covers this fitness range
		std::list<T_GAISESolution>::iterator itSol = fitness_picks.begin();
		while(itSol != fitness_picks.end()) {
			if((itSol->fit_sum_lower < pick) && (itSol->fit_sum_upper > pick)) {
				// Found our match, remove from fitness_picks and add to new generation
				new_gen.push_back(*itSol);
				// Sanity print
				if(DEBUG_GA_ISE)
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

// Creates a new population using selected_group that is augmented with the given vertex v
void GA_ISE_PathTSP::createNewAugmentedPop(Solution* solution, std::vector<T_GAISESolution> &selected_group,
		std::priority_queue<T_GAISESolution> &population, Vertex* v, int p) {
	// Determine the length of the route for this new augmented population
	int new_gen_rt_size = (int)selected_group.at(0).mRoute.size() + 1;

	// Pick out random solutions and randomly augment them with the new way-point
	while(population.size() < GAISE_POPULATION_SIZE) {
		int pick = rand() % selected_group.size();
		int insert_location = rand() % new_gen_rt_size;
		T_GAISESolution tempSol;

		// Sanity print
		if(DEBUG_GA_ISE)
			printf(" insert at: %d\n  ", insert_location);

		// Augment solution
		int i = 0;
		for(; i < (int)selected_group.at(pick).mRoute.size(); i++) {
			if(i == insert_location) {
				tempSol.mRoute.push_back(v);
			}
			tempSol.mRoute.push_back(selected_group.at(pick).mRoute.at(i));
		}
		if(i == insert_location) {
			tempSol.mRoute.push_back(v);
		}

		// Sanity print, check that we didn't mess this up...
		if(DEBUG_GA_ISE) {
			printf(" ");
			for(Vertex* i : tempSol.mRoute) {
				printf("%d ", i->nID);
			}
			printf("\n");

			if((int)tempSol.mRoute.size() != new_gen_rt_size) {
				printf("[ERROR] : Bad initial solution size (2)!\n");
				exit(0);
			}
			else if(hasDuplicates(tempSol.mRoute)) {
				printf("[ERROR] : Initial solution has duplicates (2)!");
				exit(0);
			}
		}

		tempSol.fitness = ga_fitness(solution, tempSol, p);
		population.push(tempSol);
	}
}

// Create child solution using random cross-over between parent 1 and 2
void GA_ISE_PathTSP::performCrossOver(T_GAISESolution* parent1, T_GAISESolution* parent2, T_GAISESolution* child) {
	int route_length = (int)parent1->mRoute.size();

	// Pick a range of the gene to cross over in new child
	int seqStart = rand() % route_length;
	int seqEnd = rand() % route_length;

	// Sanity print
	if(DEBUG_GA_ISE) {
		printf("\nSwapping %d -> %d from\n parent 1: ", seqStart, seqEnd);
		for(Vertex* i : parent1->mRoute) {
			printf("%d ", i->nID);
		}
		printf("\n parent 2: ");
		for(Vertex* i : parent2->mRoute) {
			printf("%d ", i->nID);
		}
		printf("\n");

	}

	if(seqEnd < seqStart) {
		int temp = (seqEnd + route_length);

		// Store selected vertices from parent 1 (avoid duplicates)
		std::list<Vertex*> wp_cross;
		for(int k = seqStart; k <= temp; k++) {
			wp_cross.push_back(parent1->mRoute[k % route_length]);
		}

		// Start adding "front-end" of solution from parent 1
		for(int k = 0; k <= seqEnd; k++) {
			child->mRoute.push_back(parent1->mRoute[k]);
		}

		// Add crossed-over section from parent 2 (l tracks vertices added to child, k verifies we don't go too far)
		int num_from_partent2 = seqStart - seqEnd + 1;
		for(int k = 0, l = 0; ((long unsigned int)k < parent2->mRoute.size()) && (l < num_from_partent2); k++) {
			if(!inList(wp_cross, parent2->mRoute[k])) {
				child->mRoute.push_back(parent2->mRoute[k]);
				l++;
			}
		}

		// Add "back-end" of solution from parent 1
		for(int k = seqStart; (long unsigned int)k < parent1->mRoute.size(); k++) {
			child->mRoute.push_back(parent1->mRoute[k]);
		}
	}
	else if(seqEnd > seqStart) {
		// Store selected vertices from parent 1 (avoid duplicates)
		std::list<Vertex*> wp_cross;
		for(int k = seqStart; k <= seqEnd; k++) {
			wp_cross.push_back(parent1->mRoute[k % route_length]);
		}

		// l tracks number of vertices added to child
		int l = 0;

		// If seqStart is at the beginning, start copying over now
		if(l == seqStart) {
			for(int m = seqStart; m <= seqEnd; m++) {
				child->mRoute.push_back(parent1->mRoute[m]);
				l++;
			}
		}

		// Add vertices in tour of parent 2
		for(int k = 0; (long unsigned int)k < parent2->mRoute.size(); k++) {
			if(!inList(wp_cross, parent2->mRoute[k])) {
				child->mRoute.push_back(parent2->mRoute[k]);
				l++;
			}

			// Check if we have reach beginning of cross-over section from parent 1
			if(l == seqStart) {
				for(int m = seqStart; m <= seqEnd; m++) {
					child->mRoute.push_back(parent1->mRoute[m]);
					l++;
				}
			}
		}
	}
	else { // seqEnd == seqStart
		Vertex* wp_cross = parent1->mRoute[seqStart];

		// l tracks how many vertices have been added to child
		int l = 0;

		// Check if first vertex should come from parent 1
		if(l == seqStart) {
			child->mRoute.push_back(wp_cross);
			l++;
		}

		// Add vertices from parent 2 to child
		for(int k = 0; (long unsigned int)k < parent2->mRoute.size(); k++) {
			if(parent2->mRoute[k] != wp_cross) {
				child->mRoute.push_back(parent2->mRoute[k]);
				l++;
			}

			// Check if this vertex should come from parent 1
			if(l == seqStart) {
				child->mRoute.push_back(wp_cross);
				l++;
			}
		}
	}

	if(DEBUG_GA_ISE) {
		printf(" child: ");
		for(Vertex* i : child->mRoute) {
			printf("%d ", i->nID);
		}
		printf("\n");

		if((int)child->mRoute.size() != route_length) {
			printf("[ERROR] : (1) Bad child size!\n");
			exit(0);
		}
		else if(hasDuplicates(child->mRoute)) {
			printf("[ERROR] : (1) Child has duplicates!\n");
			exit(0);
		}
	}
}

// Create child solution using a random mutation of parent
void GA_ISE_PathTSP::performMutation(T_GAISESolution* parent, T_GAISESolution* child) {
	for(Vertex* v : parent->mRoute) {
		child->mRoute.push_back(v);
	}

	if(DEBUG_GA_ISE) {
		printf("Mutant child: ");
		for(Vertex* i : child->mRoute) {
			printf("%d ", i->nID);
		}
		printf("\n");
	}

	// Pick two vertices to swap
	int gene1 = rand() % child->mRoute.size();
	int gene2 = rand() % child->mRoute.size();

	// Swap vertices
	Vertex* temp = child->mRoute[gene1];
	child->mRoute[gene1] = child->mRoute[gene2];
	child->mRoute[gene2] = temp;

	if(DEBUG_GA_ISE) {
		printf(" ");
		for(Vertex* i : child->mRoute) {
			printf("%d ", i->nID);
		}
		printf("\n");

		if(child->mRoute.size() != parent->mRoute.size()) {
			printf("[ERROR] : (2) Bad child size!\n");
			exit(0);
		}
		else if(hasDuplicates(child->mRoute)) {
			printf("[ERROR] : (2) Child has duplicates!\n");
			exit(0);
		}
	}
}
