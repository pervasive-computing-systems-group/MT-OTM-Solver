#include "Basic_GA_ISE.h"

Basic_GA_ISE::Basic_GA_ISE() {
	// Seed rand()
	srand(time(NULL));
}

Basic_GA_ISE::~Basic_GA_ISE() {
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
void Basic_GA_ISE::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\nRunning Genetic Algorithm with Iterative Search-Space Expansion for Path-Planning\n");
	for(int i = 0; i < solution->m_nM; i++) {
		// Run genetic algorithm
		T_GAISESolution bestSol;
		m_oGAISE_PathTSP.GeneticAlgorithm_ISE(solution, i, 3, bestSol);

		// Add this solution to pathSolution
		int v = solution->GetDepotOfPartion(i)->nID;

		// Cycle through route
		for(long unsigned int i = 0; i < bestSol.mRoute.size(); i++) {
			int u = v;
			v = bestSol.mRoute[i]->nID;
			// Add u -> v
			if(u < v) {
				solution->m_pAdjMatrix[u][v] = true;
			}
			else {
				solution->m_pAdjMatrix[v][u] = true;
			}
		}

		// Add v -> terminal
		solution->m_pAdjMatrix[v][solution->GetTerminalOfPartion(i)->nID] = true;
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************
