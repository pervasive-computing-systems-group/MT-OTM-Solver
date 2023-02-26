#include "Partitioner.h"

Partitioner::Partitioner() {
	m_bCreatesTree = false;
}

Partitioner::~Partitioner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm
 */
void Partitioner::RunPartitionAlgorithm(Solution* solution) {
	RunAlgorithm(solution);

	// Add additional indicators onto the solution, such as if it contains a tree
	solution->m_bHasTree = m_bCreatesTree;
	// Mark the solution as partitioned
	solution->m_bPartitioned = true;
}

//***********************************************************
// Private Member Functions
//***********************************************************
