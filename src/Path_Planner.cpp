#include "Path_Planner.h"

Path_Planner::Path_Planner() {
}

Path_Planner::~Path_Planner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm
 */
void Path_Planner::RunPathPlanningAlgorithm(Solution* solution) {
	// Verify that the solution was partitioned
	if(!solution->m_bPartitioned) {
		printf("[WARNING] : Solution wasn't partitioned!\n");
		Generic_Partitioner genPart;
		genPart.RunPartitionAlgorithm(solution);
	}

	// Run the sub-class path-planner
	RunAlgorithm(solution);

	// Add additional indicators onto the solution here
	solution->m_bSolved = true;
}


//***********************************************************
// Private Member Functions
//***********************************************************
