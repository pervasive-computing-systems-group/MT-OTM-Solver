#include "Iterative_GAISE.h"

Iterative_GAISE::Iterative_GAISE() {
}

Iterative_GAISE::~Iterative_GAISE() {
}


// operator overload to compare two T_GAISE_node
bool operator<(const T_VERT_NODE &a, const T_VERT_NODE &b) {
	// Check to see if A is clearly further to the right than B
	if(abs(a.pV->fX - b.pV->fX) > 0.5) {
		return a.pV->fX > b.pV->fX;
	}
	else {
		// A and B are very close vertically, split horizontally
		return a.pV->fY < b.pV->fY;
	}
}

// operator overload to compare two T_GAISE_node
bool operator<(const T_VSWEEP_NODE &a, const T_VSWEEP_NODE &b) {
	return a.fTheta < b.fTheta;
}

//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Runs a hybrid iterative sweeping approach
 */
void Iterative_GAISE::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("Running Iterative GA-ISE\n");

	// Set a desired share value and assign initial partition share values
	float const desired_share_value = 1.0 / (float)solution->m_nM;
	float worst_share_error = 0;
	std::vector<float> partitionShares(solution->m_nM);
	std::vector<float> actualShares(solution->m_nM);

	// Set initial partition shares using even split
	for(int i = 0; i < solution->m_nM; i++) {
		// Set initial share portion and "guess" of actual share portion
		partitionShares.at(i) = desired_share_value;
		actualShares.at(i) = desired_share_value;
	}

	// Solution for this setup
	std::vector<std::list<Vertex*>> bestSolution(solution->m_nM);
	float total_dist1 = 0;

	// Iteratively solve partitioning and path-planning
	float epsilon_push = 0.0;
	int loops = 0;
	do {
		// Partition vertices using given partitionShares split
		orderedPartition(solution, &partitionShares);

		// Clear current best solution
		bestSolution.clear();
		bestSolution = std::vector<std::list<Vertex*>>(solution->m_nM);

		// Find best set of routes for this partition
		for(int p = 0; p < solution->m_nM; p++) {
			// Run genetic algorithm
			T_GAISESolution bestSol;
			m_oGAISE_PathTSP.GeneticAlgorithm_ISE(solution, p, 3, bestSol);

			// Add the depot to the route
			bestSolution.at(p).push_back(solution->GetDepotOfPartion(p));
			// Add returned solution route to bestSolution1
			for(long unsigned int i = 0; i < bestSol.mRoute.size(); i++) {
				bestSolution.at(p).push_back(bestSol.mRoute.at(i));
			}
			// Add the terminal to the route
			bestSolution.at(p).push_back(solution->GetTerminalOfPartion(p));
		}

		total_dist1 = 0;
		// Determine the total distance of the solution
		for(std::list<Vertex*> list : bestSolution) {
			total_dist1 += solution->GetTourDist(list);
		}

		// Determine the share of each route
		if(SANITY_PRINT)
			printf("Actual share of work:\n");
		for(long unsigned int i = 0; i < bestSolution.size(); i++) {
			actualShares.at(i) = solution->GetTourDist(bestSolution.at(i))/total_dist1;
			if(SANITY_PRINT)
				printf(" %ld: %f\n", i, actualShares.at(i));
		}

		// Update partition shares and worst share error
		worst_share_error = 0;
		if(SANITY_PRINT)
			printf("Updated share of work:\n");
		for(long unsigned int i = 0; i < bestSolution.size(); i++) {
			float share_error = desired_share_value - actualShares.at(i);
			partitionShares.at(i) = partitionShares.at(i) + share_error;
			if(abs(share_error) > worst_share_error) {
				worst_share_error = abs(share_error);
			}
			if(SANITY_PRINT)
				printf(" %ld: %f\n", i, partitionShares.at(i));
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
		loops++;
		if(loops >= 15) {
			// This isn't working... what you asked for isn't realistic
			fprintf(stderr, "[ERROR] : Iterative_GAISE::RunAlgorithm(), requested partition portions aren't realistic with m = %d\n", solution->m_nM);
			break;
		}
	} while(worst_share_error > (EPSILON_IT_GAISE_ERROR + epsilon_push));

	if(DEBUG_IT_GAISE)
		printf(" solution complete, %f\n", total_dist1);

	// Assign the best found set of routes to solution
	for(std::list<Vertex *> list : bestSolution) {
		// Iterate through the best route list and add it to the adjacency matrix
		Vertex * previous = list.front();
		list.pop_front();
		for(Vertex* v : list) {
			Vertex* next = v;
			int a = previous->nID;
			int b = next->nID;

			if(a < b) {
				solution->m_pAdjMatrix[a][b] = true;
			}
			else {
				solution->m_pAdjMatrix[b][a] = true;
			}

			previous = next;
		}
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************

/*
 * Runs a vertical partitioner based on given parameters
 */
void Iterative_GAISE::orderedPartition(Solution* pSolution, std::vector<float>* pSharesVector) {
	// Clear any existing partition
	pSolution->m_mPartitions.clear();

	std::list<Vertex*> verticalOrderedList;
	findVectorSweepingOrder(pSolution, &verticalOrderedList);

	// Divide up the vertices based on the requested shares
	std::vector<int> nSharesVector;
	for(float share : *pSharesVector) {
		int num_vertices = share * pSolution->m_nN;
		nSharesVector.push_back(std::max(num_vertices, 1));

	}

	// Verify that we have the correct portions in pSharesVector
	bool try_again = true;
	while(try_again) {
		/*
		 * TODO: This process sucks. It would be better to update the expected shares
		 * (and update the base station's position) than waste time trying to force the
		 * system to settle when it just isn't feasible
		 */
		int count = 0;
		for(long unsigned int i = 0; i < nSharesVector.size(); i++) {
			if(nSharesVector.at(i) <= 0) {
				nSharesVector.at(i) = 1;
			}
			count += nSharesVector.at(i);
		}

		if(count < pSolution->m_nN) {
			// Not enough vertices have been accounted for...
			printf(" too few, count = %d ", count);
			int index = -1;
			float largest_diff = 0;
			// Determine which partition has the largest over-shoot
			for(long unsigned int i = 0; i < nSharesVector.size(); i++) {
				float actual_share = nSharesVector.at(i) * 1.0/pSolution->m_nN;
				float share_diff = pSharesVector->at(i) - actual_share;
				if(share_diff > largest_diff) {
					largest_diff = share_diff;
					index = i;
				}
			}

			printf(", index = %d\n", index);
			if(index != -1) {
				nSharesVector.at(index)++;
			}
			else {
				// Something went wrong, hard-fail and report it
				fprintf(stderr, "[ERROR] : Iterative_GAISE::orderedPartition(), under-partitioned but all actuals too low\n");
				exit(1);
			}
		}
		else if(count > pSolution->m_nN) {
			// We have allotted too man vertices
			printf(" too large, count = %d ", count);
			int index = -1;
			float smallest_share = 1;
			int backup_index = -1;
			float largest_partition = 0;
			// Determine which partition has the smallest (non-negative) over-shoot
			for(long unsigned int i = 0; i < nSharesVector.size(); i++) {
				float actual_share = nSharesVector.at(i) * 1.0/pSolution->m_nN;
				float share_diff = pSharesVector->at(i) - actual_share;
				if((share_diff > 0) && (nSharesVector.at(i) > 1) && (share_diff < smallest_share)) {
					smallest_share = share_diff;
					index = i;
				}

				if(nSharesVector.at(i) > largest_partition) {
					largest_partition = nSharesVector.at(i);
					backup_index = i;
				}
			}

			printf(", index = %d,  backup_index = %d\n", index, backup_index);

			if(index != -1) {
				nSharesVector.at(index)--;
			}
			else {
				// Just take away from the largest partition
				nSharesVector.at(backup_index)--;
			}
		}
		else {
			// All partitions have enough shares
			try_again = false;
		}
	}

	int p = 0;
	int p_running_count = 0;
	int DBG_cound = 0;
	std::vector<Vertex*> tempVector;
	pSolution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(p, tempVector));
	if(DEBUG_IT_GAISE)
		printf("  New partition! %d\n Contains: ", p);
	while(!verticalOrderedList.empty()) {
		// Determine if we need to update the current partition
		if((p < (pSolution->m_nM - 1)) && (p_running_count >= nSharesVector.at(p))) {
			// We have completed this partition, update p, running count, and column index
			p++;
			p_running_count = 0;
			std::vector<Vertex*> tempVector;
			pSolution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(p, tempVector));
			// Sanity print
			if(DEBUG_IT_GAISE)
				printf("\n  New partition! %d\n Contains: ", p);
		}

		// Add the top of column to the partition
		pSolution->m_mPartitions.at(p).push_back(verticalOrderedList.front());
		if(DEBUG_IT_GAISE)
			printf("%d ", verticalOrderedList.front()->nID);
		verticalOrderedList.pop_front();
		p_running_count++;
		DBG_cound++;
	}

	// Add depots and terminals
	for(int i = 0; i < pSolution->m_nM; i++) {
		// Add the depot and terminal for this partition
		pSolution->m_mPartitions.at(i).push_back(pSolution->m_pVertexData + (pSolution->m_nN + pSolution->m_nM + i));
		pSolution->m_mPartitions.at(i).push_back(pSolution->m_pVertexData + (pSolution->m_nN + i));
		DBG_cound += 2;
	}

	// Sanity print
	if(DEBUG_IT_GAISE)
		printf("\n total count: %d, expected: %d\n", DBG_cound, pSolution->m_nN + (pSolution->m_nM * 2));
}

/*
 * Finds the columns that each vertex belongs to and puts them into a single list. The order of the list is
 * based off of the same order that findColumns() returns but with the resulting columns pushed (in ascending order)
 * into a single list.
 */
void Iterative_GAISE::findVerticalOrder(Solution* solution, std::list<Vertex*>* verticalOrderedList) {
	// Create a priority queue
	std::priority_queue<T_VERT_NODE> vertQueue;

	// Create vertical nodes and push them onto the queue
	for(int i = 0; i < solution->m_nN; i++) {
		T_VERT_NODE tempNode(solution->m_pVertexData + i);
		vertQueue.push(tempNode);
	}

	// Empty queue into given list
	while(!vertQueue.empty()) {
		verticalOrderedList->push_back(vertQueue.top().pV);
		vertQueue.pop();
	}
}

/*
 * Determines the order to add vertices based on a sweeping vector approach. One can visualize this as dividing up
 * the search space into pizza slices
 */
void Iterative_GAISE::findVectorSweepingOrder(Solution* solution, std::list<Vertex*>* verticalOrderedList) {
	// Create a priority queue
	std::priority_queue<T_VSWEEP_NODE> vectSweepQueue;

	// Find the middle-point of the route
	Vertex* firstDepot = solution->GetDepotOfPartion(0);
	Vertex* lastTerminal = solution->GetTerminalOfPartion(solution->m_nM - 1);
	float bsCentroid = (lastTerminal->fX - firstDepot->fX)/2 + firstDepot->fX;
	// Sanity print
	if(DEBUG_IT_GAISE)
		printf(" bs-centroid = %f\n", bsCentroid);

	// Create vertical nodes and push them onto the queue
	for(int i = 0; i < solution->m_nN; i++) {
		Vertex* v = (solution->m_pVertexData + i);
		// Determine magnitude of centroid-vertex vector. Note: by convention, the bs follows the x-axis, therefore
		// the y component of the centroid is always 0
		float cvMag = sqrt(pow((bsCentroid - v->fX), 2) + pow((0 - v->fY), 2));
		// Determine the angle from the bs trajectory to the centroid-vertex vector
		float cvTheta = acos((v->fX - bsCentroid)/cvMag);
		T_VSWEEP_NODE tempNode(v, cvTheta);
		vectSweepQueue.push(tempNode);
		// Sanity print
		if(DEBUG_IT_GAISE)
			printf("  %d: %f*\n", v->nID, cvTheta);
	}

	// Empty queue into given list
	while(!vectSweepQueue.empty()) {
		verticalOrderedList->push_back(vectSweepQueue.top().pV);
		vectSweepQueue.pop();
	}
}
