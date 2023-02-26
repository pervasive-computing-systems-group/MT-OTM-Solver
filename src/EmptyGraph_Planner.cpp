#include "EmptyGraph_Planner.h"

EmptyGraph_Planner::EmptyGraph_Planner() {
}

EmptyGraph_Planner::~EmptyGraph_Planner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Performs checks and runs the RunAlgorithm of the sub-class
 */
void EmptyGraph_Planner::RunEGAlgorithm(Solution* solution) {
	if(solution->isSetup()) {
		// Not what we expected...
		fprintf(stderr, "[ERROR] EmptyGraph_Planner : Asked to solve solution with setup graph\n");
		solution->m_mPartitions.clear();
		solution->ClearAdjacencyMatrix();
	}

	// Run the sub-class hybrid algorithm
	RunAlgorithm(solution);

	// Graph should be setup now
	if(solution->isSetup()) {
		// Add additional indicators onto the solution here
		solution->m_bPartitioned = true;
		solution->m_bSolved = true;
		if(SANITY_PRINT) {
			printf("Found a solution!\n");
		}
	}
	else {
		if(SANITY_PRINT) {
			printf("Failed to find a solution!\n");
		}
		solution->m_bPartitioned = false;
		solution->m_bSolved = false;
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************
