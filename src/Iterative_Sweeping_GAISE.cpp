#include "Iterative_Sweeping_GAISE.h"

Iterative_Sweeping_GAISE::Iterative_Sweeping_GAISE() {
}

Iterative_Sweeping_GAISE::~Iterative_Sweeping_GAISE() {
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
void Iterative_Sweeping_GAISE::RunAlgorithm(Solution* solution) {

	// Find a reference triangle (find A, B, C)
	findReferenceTriangle(solution, solution->m_fR);

	// Determine the orientation of our reference triangle
	Vertex* minV = NULL;
	Vertex* midV = NULL;
	Vertex* topV = NULL;
	determineTriangleOrientation(minV, midV, topV);

	// Set a desired share value and assign initial partition share values
	float const desired_share_value = 1.0 / (float)solution->m_nM;
	float worst_share_error = 0;
	std::vector<float> partitionShares(solution->m_nM);
	std::vector<float> actualShares(solution->m_nM);

	// Partition using even split
	for(int i = 0; i < solution->m_nM; i++) {
		// Set initial share portion and "guess" of actual share portion
		partitionShares.at(i) = desired_share_value;
		actualShares.at(i) = desired_share_value;
	}

	/// Run using A := minV, B := topV, C := midV
	m_pA = minV;
	m_pB = topV;
	m_pC = midV;
	printf("\nRun with A := minV, B := topV, C := midV\n");

	// Solution for this setup
	std::vector<std::list<Vertex*>> bestSolution1(solution->m_nM);
	float total_dist1 = 0;

	// Iteratively solve partitioning and path-planning
	float epsilon_push = 0.0;
	do {
		// Partition vertices using given partitionShares split
		sweepingPartition(solution, &partitionShares);

		// Clear current best solution
		bestSolution1.clear();
		bestSolution1 = std::vector<std::list<Vertex*>>(solution->m_nM);

		// Find best set of routes for this partition
		for(int p = 0; p < solution->m_nM; p++) {
			// Run genetic algorithm
			T_GAISESolution bestSol;
			m_oGAISE_PathTSP.GeneticAlgorithm_ISE(solution, p, 3, bestSol);

			// Add the depot to the route
			bestSolution1.at(p).push_back(solution->GetDepotOfPartion(p));
			// Add returned solution route to bestSolution1
			for(long unsigned int i = 0; i < bestSol.mRoute.size(); i++) {
				bestSolution1.at(p).push_back(bestSol.mRoute.at(i));
			}
			// Add the terminal to the route
			bestSolution1.at(p).push_back(solution->GetTerminalOfPartion(p));
		}

		total_dist1 = 0;
		// Determine the total distance of the solution
		for(std::list<Vertex*> list : bestSolution1) {
			total_dist1 += solution->GetTourDist(list);
		}

		// Determine the share of each route
		printf("Actual share of work:\n");
		for(long unsigned int i = 0; i < bestSolution1.size(); i++) {
			actualShares.at(i) = solution->GetTourDist(bestSolution1.at(i))/total_dist1;
			printf(" %ld: %f\n", i, actualShares.at(i));
		}

		// Update partition shares and worst share error
		worst_share_error = 0;
		printf("Updated share of work:\n");
		for(long unsigned int i = 0; i < bestSolution1.size(); i++) {
			float share_error = desired_share_value - actualShares.at(i);
			partitionShares.at(i) = partitionShares.at(i) + share_error;
			if(abs(share_error) > worst_share_error) {
				worst_share_error = abs(share_error);
			}
			printf(" %ld: %f\n", i, partitionShares.at(i));
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
	} while(worst_share_error > (EPSILON_SGAISE_ERROR + epsilon_push));

	printf(" solution 1 complete, %f\n", total_dist1);


	/// Run using A := minV, B := midV, C := topV
	m_pA = minV;
	m_pB = midV;
	m_pC = topV;
	printf("\nRun with A := minV, B := midV, C := topV\n");

	// Solution for this setup
	std::vector<std::list<Vertex*>> bestSolution2(solution->m_nM);
	float total_dist2 = 0;

	// Partition using even split
	for(int i = 0; i < solution->m_nM; i++) {
		// Set initial share portion and "guess" of actual share portion
		partitionShares.at(i) = desired_share_value;
		actualShares.at(i) = desired_share_value;
	}

	// Iteratively solve partitioning and path-planning
	epsilon_push = 0.0;
	do {
		// Clear current best solution
		bestSolution2.clear();
		bestSolution2 = std::vector<std::list<Vertex*>>(solution->m_nM);

		// Partition vertices using given partitionShares split
		sweepingPartition(solution, &partitionShares);

		// Find best set of routes for this partition
		for(int p = 0; p < solution->m_nM; p++) {
			// Run genetic algorithm
			T_GAISESolution bestSol;
			m_oGAISE_PathTSP.GeneticAlgorithm_ISE(solution, p, 3, bestSol);

			// Add the depot to the route
			bestSolution2.at(p).push_back(solution->GetDepotOfPartion(p));
			// Add returned solution route to bestSolution1
			for(long unsigned int i = 0; i < bestSol.mRoute.size(); i++) {
				bestSolution2.at(p).push_back(bestSol.mRoute.at(i));
			}
			// Add the terminal to the route
			bestSolution2.at(p).push_back(solution->GetTerminalOfPartion(p));
		}

		total_dist2 = 0;
		// Determine the total distance of the solution
		for(std::list<Vertex*> list : bestSolution2) {
			total_dist2 += solution->GetTourDist(list);
		}

		// Determine the share of each route
		printf("Actual share of work:\n");
		for(long unsigned int i = 0; i < bestSolution2.size(); i++) {
			actualShares.at(i) = solution->GetTourDist(bestSolution2.at(i))/total_dist2;
			printf(" %ld: %f\n", i, actualShares.at(i));
		}

		// Update partition shares and worst share error
		worst_share_error = 0;
		printf("Updated share of work:\n");
		for(long unsigned int i = 0; i < bestSolution2.size(); i++) {
			float share_error = desired_share_value - actualShares.at(i);
			partitionShares.at(i) = partitionShares.at(i) + share_error;
			if(abs(share_error) > worst_share_error) {
				worst_share_error = abs(share_error);
			}
			printf(" %ld: %f\n", i, partitionShares.at(i));
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
	} while(worst_share_error > (EPSILON_SGAISE_ERROR + epsilon_push));

	printf(" solution 2 complete, %f\n", total_dist2);


	/// Run using A := midV, B := topV, C := minV
	m_pA = midV;
	m_pB = topV;
	m_pC = minV;
	printf("\nRun with A := midV, B := topV, C := minV\n");

	// Solution for this setup
	std::vector<std::list<Vertex*>> bestSolution3(solution->m_nM);
	float total_dist3 = 0;

	// Partition using even split
	for(int i = 0; i < solution->m_nM; i++) {
		// Set initial share portion and "guess" of actual share portion
		partitionShares.at(i) = desired_share_value;
		actualShares.at(i) = desired_share_value;
	}

	// Iteratively solve partitioning and path-planning
	epsilon_push = 0.0;
	do {
		// Clear current best solution
		bestSolution3.clear();
		bestSolution3 = std::vector<std::list<Vertex*>>(solution->m_nM);

		// Partition vertices using given partitionShares split
		sweepingPartition(solution, &partitionShares);

		// Find best set of routes for this partition
		for(int p = 0; p < solution->m_nM; p++) {
			// Run genetic algorithm
			T_GAISESolution bestSol;
			m_oGAISE_PathTSP.GeneticAlgorithm_ISE(solution, p, 3, bestSol);

			// Add the depot to the route
			bestSolution3.at(p).push_back(solution->GetDepotOfPartion(p));
			// Add returned solution route to bestSolution1
			for(long unsigned int i = 0; i < bestSol.mRoute.size(); i++) {
				bestSolution3.at(p).push_back(bestSol.mRoute.at(i));
			}
			// Add the terminal to the route
			bestSolution3.at(p).push_back(solution->GetTerminalOfPartion(p));
		}

		total_dist3 = 0;
		// Determine the total distance of the solution
		for(std::list<Vertex*> list : bestSolution3) {
			total_dist3 += solution->GetTourDist(list);
		}

		// Determine the share of each route
		printf("Actual share of work:\n");
		for(long unsigned int i = 0; i < bestSolution3.size(); i++) {
			actualShares.at(i) = solution->GetTourDist(bestSolution3.at(i))/total_dist3;
			printf(" %ld: %f\n", i, actualShares.at(i));
		}

		// Update partition shares and worst share error
		worst_share_error = 0;
		printf("Updated share of work:\n");
		for(long unsigned int i = 0; i < bestSolution3.size(); i++) {
			float share_error = desired_share_value - actualShares.at(i);
			partitionShares.at(i) = partitionShares.at(i) + share_error;
			if(abs(share_error) > worst_share_error) {
				worst_share_error = abs(share_error);
			}
			printf(" %ld: %f\n", i, partitionShares.at(i));
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
	} while(worst_share_error > (EPSILON_SGAISE_ERROR + epsilon_push));

	printf(" solution 3 complete, %f\n", total_dist3);


	// Determine which orientation gave the best solution
	std::vector<std::list<Vertex*>>* bestSolution;
	if((total_dist1 < total_dist2) && (total_dist1 < total_dist3)) {
		printf("\nBest solution is 1, %f\n", total_dist1);
		bestSolution = &bestSolution1;
	}
	else if((total_dist2 < total_dist1) && (total_dist2 < total_dist3)) {
		printf("\nBest solution is 2, %f\n", total_dist2);
		bestSolution = &bestSolution2;
	}
	else {
		printf("\nBest solution is 3, %f\n", total_dist3);
		bestSolution = &bestSolution3;
	}

	// Assign the best found set of routes to solution
	for(std::list<Vertex *> list : *bestSolution) {
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
 * Runs sweeping partitioning based on given parameters
 */
void Iterative_Sweeping_GAISE::sweepingPartition(Solution* pSolution, std::vector<float>* pSharesVector) {
	// Clear any existing partition
	pSolution->m_mPartitions.clear();

	std::list<Vertex*> columnsOrderedList;
	findColumnList(pSolution, &columnsOrderedList);

	int p = 0;
	int p_running_count = 0;
	int DBG_cound = 0;
	std::vector<Vertex*> tempVector;
	pSolution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(p, tempVector));
	if(DEBUG_IT_SGAISE)
		printf("  New partition! %d\n Contains: ", p);
	while(!columnsOrderedList.empty()) {
		// Determine if we need to update the current partition
		if((p < (pSolution->m_nM - 1)) && (p_running_count >= (pSharesVector->at(p) * (float)pSolution->m_nN))) {
			// We have completed this partition, update p, running count, and column index
			p++;
			p_running_count = 0;
			std::vector<Vertex*> tempVector;
			pSolution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(p, tempVector));
			// Sanity print
			if(DEBUG_IT_SGAISE)
				printf("\n  New partition! %d\n Contains: ", p);
		}

		// Add the top of column to the partition
		pSolution->m_mPartitions.at(p).push_back(columnsOrderedList.front());
		if(DEBUG_IT_SGAISE)
			printf("%d ", columnsOrderedList.front()->nID);
		columnsOrderedList.pop_front();
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
	if(DEBUG_IT_SGAISE)
		printf("\n total count: %d, expected: %d\n", DBG_cound, pSolution->m_nN + (pSolution->m_nM * 2));
}

/*
 * Finds the columns, and stores them in columnsMap, for the given solution. Assumes that m_pA, m_pB,
 * and m_pC have already been assigned.
 */
int Iterative_Sweeping_GAISE::findColumns(Solution* solution, std::map<int, std::priority_queue<T_ColumnV>>* columnsMap) {
	// Minimum column indexes (we are guaranteed to have an index of 0 because this is where A is located)
	int min_col = 0;

	// Start pushing vertices into stacks for each row
	for(int i = 0; i < solution->m_nN; i++) {
		// Put vertex into a container to help track which row/column it belongs to
		Vertex* cV = solution->m_pVertexData + i;

		// Find which column to put this vertex in
		int col = getColumn(solution->m_pVertexData + i, solution->m_fR);
		std::map<int, std::priority_queue<T_ColumnV>>::iterator itC = columnsMap->find(col);
		if(itC != columnsMap->end()) {
			// Add this vertex to the column queue
			itC->second.push(cV);
		}
		else {
			if(DEBUG_IT_SGAISE)
				printf("  new column: %d\n", col);
			columnsMap->insert(std::pair<int, std::priority_queue<T_ColumnV>>(col, std::priority_queue<T_ColumnV>()));
			columnsMap->at(col).push(cV);
		}

		// If necessary, update minimum column index
		if(min_col > col) {
			min_col = col;
		}

		if(DEBUG_IT_SGAISE)
			printf(" %d => R%d\n", (solution->m_pVertexData + i)->nID, col);
	}

	if(DEBUG_IT_SGAISE)
		printf(" min-column: %d\n", min_col);

	return min_col;
}

/*
 * Finds the columns that each vertex belongs to and puts them into a single list. The order of the list is
 * based off of the same order that findColumns() returns but with the resulting columns pushed (in ascending order)
 * into a single list.
 */
void Iterative_Sweeping_GAISE::findColumnList(Solution* solution, std::list<Vertex*>* columnsOrderedList) {
	std::map<int, std::priority_queue<T_ColumnV>> columnsMap;

	// Grab graph columns from findColumns()
	int min_col = findColumns(solution, &columnsMap);

	// Push these columns into a single list, in ascending order
	int offset = 0;
	for(long unsigned int i = 0; i < columnsMap.size(); i++) {
		// Verify that the "next" column exists
		while(!(columnsMap.count(i + min_col + offset) > 0)) {
			// Update offset to find next existing column
			offset++;
		}
		// Empty column into the list
		while(!columnsMap.at(i + min_col + offset).empty()) {
			columnsOrderedList->push_back(columnsMap.at(i + min_col + offset).top().pV);
			columnsMap.at(i + min_col + offset).pop();
		}
	}
}
