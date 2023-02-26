#include "Generic_Partitioner.h"

Generic_Partitioner::Generic_Partitioner() {
	m_bCreatesTree = false;
}

Generic_Partitioner::~Generic_Partitioner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm
 */
void Generic_Partitioner::RunAlgorithm(Solution* solution) {
	// Ensure that there isn't already a partition
	solution->m_mPartitions.clear();
	int verticesInPartition = solution->m_nN / solution->m_nM;
	// Add single to solution
	for(int i = 0; i < solution->m_nM; i++) {
		std::vector<Vertex*> partList;
		for(int j = (i * verticesInPartition); j < ((i + 1) * verticesInPartition); j++) {
			// Add to partition
			partList.push_back(solution->m_pVertexData + j);
		}

		// Add depot and terminal
		partList.push_back(solution->m_pVertexData + solution->m_nN + i);
		partList.push_back(solution->m_pVertexData + solution->m_nN + i + solution->m_nM);

		if((i == solution->m_nM - 1) && (solution->m_nN % solution->m_nM)) {
			// Add whatever is left into last partition
			for(int j = ((i + 1) * verticesInPartition); j < solution->m_nN; j++) {
				// Add to partition
				partList.push_back(solution->m_pVertexData + j);
			}
		}

		// Add this partition to the partitioning map
		solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(i, partList));
	}

	// Sanity print
	for(auto vect: solution->m_mPartitions) {
		printf("Partition: %d\n ", vect.first);
		for(Vertex* v : vect.second) {
			printf("%d ", v->nID);
		}
		printf("\n");
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************
