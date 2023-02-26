#include "Random_PathPlanner.h"

Random_PathPlanner::Random_PathPlanner() {
	// Seed rand()
	srand(time(NULL));
}

Random_PathPlanner::~Random_PathPlanner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Uses a random ordering algorithm to "solve" the Path TSP on each partition in solution
 */
void Random_PathPlanner::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\nRunning Random-Ordering Algorithm for Path-Planning\n");
	for(int i = 0; i < solution->m_nM; i++) {
		// Run genetic algorithm
		RandomAlgorithm(solution, i);
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************

// Runs a simple "Random" algorithm that hastily puts the vertices into a path based simply on the
// order that they appear in the partition.
void Random_PathPlanner::RandomAlgorithm(Solution* solution, int p) {
	// Add rand_solution path to solution adjacency matrix
	int terminal = solution->m_nN + p;
	int depot = solution->m_nN + solution->m_nM + p;
	Vertex* first_v = NULL;
	Vertex* previous = NULL;
	Vertex* last = NULL;

	for(Vertex * V : solution->m_mPartitions.at(p)) {
		if(V->eVType == E_VertexType::e_Destination) {
			if(first_v == NULL) {
				previous = last = first_v = V;
			}
			else {
				int u = previous->nID;
				int v = V->nID;
				// Add u -> v
				if(u < v) {
					solution->m_pAdjMatrix[u][v] = true;
				}
				else {
					solution->m_pAdjMatrix[v][u] = true;
				}

				last = previous = V;
			}
		}
	}

	// Add depot -> first_v
	solution->m_pAdjMatrix[first_v->nID][depot] = true;

	// Add last -> terminal
	solution->m_pAdjMatrix[last->nID][terminal] = true;
}
