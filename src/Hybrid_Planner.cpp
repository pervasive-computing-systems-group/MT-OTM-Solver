#include "Hybrid_Planner.h"

Hybrid_Planner::Hybrid_Planner() {
}

Hybrid_Planner::~Hybrid_Planner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm
 */
void Hybrid_Planner::RunHybridAlgorithm(Solution* solution) {
	// Verify that there aren't partitions or already a path in this solution
	if(solution->m_mPartitions.size() > 0) {
		solution->m_mPartitions.clear();
	}
	solution->ClearAdjacencyMatrix();

	// Run the sub-class hybrid algorithm
	RunAlgorithm(solution);

	// Add additional indicators onto the solution here
	solution->m_bPartitioned = true;
	solution->m_bSolved = true;
}


//***********************************************************
// Private Member Functions
//***********************************************************
