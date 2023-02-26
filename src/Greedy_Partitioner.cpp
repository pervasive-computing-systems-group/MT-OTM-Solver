#include "Greedy_Partitioner.h"

Greedy_Partitioner::Greedy_Partitioner() {
	m_bCreatesTree = false;
}

Greedy_Partitioner::~Greedy_Partitioner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm
 */
void Greedy_Partitioner::RunAlgorithm(Solution* solution) {
	printf("Creating Greedy Partition\n");

	// Ensure that there isn't already a partition
	solution->m_mPartitions.clear();

	// Verify that there is something to be partitioned
	if(solution->m_nM == 1) {
		// No partitioning needed, just add all the vertices
		solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(0, std::vector<Vertex*>()));
		for(int i = 0; i < solution->m_nNumVertices; i++) {
			solution->m_mPartitions.at(0).push_back(solution->m_pVertexData + i);
		}

		return;
	}

	// Add each destination vertex to a mask list of destinations
	std::list<Vertex*> availableList;
	for(int i = 0; i < solution->m_nN; i++) {
		availableList.push_back(solution->m_pVertexData + i);
	}

	// Sanity print
	if(DEBUG_GREEDY)
		printf(" |Available List| = %ld, n = %d\n", availableList.size(), solution->m_nN);

	Vertex* depot = solution->m_pVertexData + solution->m_nN + solution->m_nM;

	// Maintain a list of the "base" vertices that the rest of the algorithm is based on
	std::vector<Vertex*> baseVertices;
	// Add the closest vertex to each of the depots
	for(int i = 0; i < solution->m_nM; i++) {
		// Sanity print
		if(DEBUG_GREEDY)
			printf(" First for node: %d, ", depot->nID);
		std::list<Vertex*>::iterator best_it, it = availableList.begin();
		best_it = it;
		// Start with first vertex as best choice
		Vertex* best_v = (*it);
		float best_dist = best_v->GetDistanceTo(depot);
		// Iterate through the rest of the vertices
		for(it++; it != availableList.end(); it++) {
			Vertex* temp = (*it);
			if(temp->GetDistanceTo(depot) < best_dist) {
				// Found a new best option
				best_v = temp;
				best_dist = temp->GetDistanceTo(depot);
				best_it = it;
			}
		}

		// Sanity print
		if(DEBUG_GREEDY)
			printf("%d\n", best_v->nID);

		// Add the best found vertex to this depot's partition and to the base list
		solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(i, std::vector<Vertex*>()));
		solution->m_mPartitions.at(i).push_back(best_v);
		baseVertices.push_back(best_v);
		availableList.erase(best_it);
		depot++;
	}

	// Sanity print
	if(DEBUG_GREEDY) {
		printf("Initial set-up:\n");
		for(auto p : solution->m_mPartitions) {
			printf(" (%d: %d)\n", p.first, p.second.front()->nID);
		}
	}

	// Have each partition take a turn to pull out the closest vertex
	int turn = 0;
	while(!availableList.empty()) {
		// Grab initial vertex
		Vertex* base = baseVertices.at(turn%baseVertices.size());
		// Start with first available vertex as best choice
		std::list<Vertex*>::iterator best_it, it = availableList.begin();
		best_it = it;
		Vertex* best_v = (*it);
		float best_dist = best_v->GetDistanceTo(base);

		// Iterate through the rest of the vertices
		for(it++; it != availableList.end(); it++) {
			Vertex* temp = (*it);
			if(temp->GetDistanceTo(base) < best_dist) {
				// Found a new best option
				best_v = temp;
				best_dist = temp->GetDistanceTo(base);
				best_it = it;
			}
		}

		// Add the best found vertex to this depot's partition
		solution->m_mPartitions.at(turn%baseVertices.size()).push_back(best_v);
		availableList.erase(best_it);
		turn++;
	}

	// Add the base-stations
	for(int i = 0; i < solution->m_nM; i++) {
		solution->m_mPartitions.at(i).push_back(solution->m_pVertexData + solution->m_nN + i);
		solution->m_mPartitions.at(i).push_back(solution->m_pVertexData + solution->m_nN + solution->m_nM + i);
	}

	// Sanity print
	if(DEBUG_GREEDY) {
		printf("Final partition:\n");
		for(auto p : solution->m_mPartitions) {
			printf(" (%d:", p.first);
			for(Vertex* v : p.second) {
				printf(" %d", v->nID);
			}
			printf(")\n");
		}
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************
