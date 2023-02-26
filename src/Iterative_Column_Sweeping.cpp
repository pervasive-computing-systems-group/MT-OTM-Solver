#include "Iterative_Column_Sweeping.h"

Iterative_Column_Sweeping::Iterative_Column_Sweeping() {
}

Iterative_Column_Sweeping::~Iterative_Column_Sweeping() {
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
void Iterative_Column_Sweeping::RunAlgorithm(Solution* solution) {
	// Find the reference triangle
	findReferenceTriangle(solution, solution->m_fR);

	Vertex* minV = NULL;
	Vertex* midV = NULL;
	Vertex* topV = NULL;

	// Determine the orientation of the reference triangle
	determineTriangleOrientation(minV, midV, topV);

	std::vector<float> partitionShares(solution->m_nM);
	std::vector<float> actualShares(solution->m_nM);
	float const desired_share_value = 1.0 / (float)solution->m_nM;
	float worst_share_error = 0;

	/// Run using A := minV, B := topV, C := midV
	m_pA = minV;
	m_pB = topV;
	m_pC = midV;
	printf("\nRun with A := minV, B := topV, C := midV\n");

	// Solution for this setup
	std::vector<std::list<Vertex*>> bestSolution1(solution->m_nM);
	float total_dist1 = 0;

	// Partition using even split
	for(int i = 0; i < solution->m_nM; i++) {
		// Set initial share portion and "guess" of actual share portion
		partitionShares.at(i) = desired_share_value;
		actualShares.at(i) = desired_share_value;
	}

	// Iteratively solve partitioning and path-planning
	float epsilon_push = 0.0;
	do {
		// Clear current best solution
		bestSolution1.clear();
		bestSolution1 = std::vector<std::list<Vertex*>>(solution->m_nM);

		// Partition vertices using given partitionShares split
		sweepingPartition(solution, &partitionShares);

		// Find best set of routes for this partition
		for(int p = 0; p < solution->m_nM; p++) {
			findBestSweepingRoute(solution, p, bestSolution1.at(p));
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
			// Update share error
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
		if(worst_share_error > EPSILON_II_ERROR) {
			printf("Run again\n");
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
	} while(worst_share_error > (EPSILON_II_ERROR + epsilon_push));

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
			findBestSweepingRoute(solution, p, bestSolution2.at(p));
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
			// Update share error
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
		if(worst_share_error > EPSILON_II_ERROR) {
			printf("Run again\n");
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
	} while(worst_share_error > (EPSILON_II_ERROR + epsilon_push));

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

		if(solution->m_mPartitions.size() != (long unsigned int)solution->m_nM) {
			printf("[ERROR 1] : |P| != m\n");
		}

		// Partition vertices using given partitionShares split
		sweepingPartition(solution, &partitionShares);

		// Find best set of routes for this partition
		for(int p = 0; p < solution->m_nM; p++) {

			if(solution->m_mPartitions.size() != (long unsigned int)solution->m_nM) {
				printf("[ERROR 2] : |P| != m\n");
			}

			findBestSweepingRoute(solution, p, bestSolution3.at(p));
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
			// Update share error
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
		if(worst_share_error > EPSILON_II_ERROR) {
			printf("Run again\n");
		}

		// Avoid getting stuck
		epsilon_push += 0.002;
	} while(worst_share_error > (EPSILON_II_ERROR + epsilon_push));

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
void Iterative_Column_Sweeping::sweepingPartition(Solution* pSolution, std::vector<float>* pSharesVector) {
	// Clear any existing partion
	pSolution->m_mPartitions.clear();

	std::map<int, std::priority_queue<T_ColumnV>> columnsMap;

	// Grab graph columns from findColumns()
	int min_col = findColumns(pSolution, &columnsMap);

	// Divide up the columns
	int current_part = 0;
	int running_count = 0;
	int column_offset = 0;
	int DBG_count = 0;

	// Sanity print
	if(SANITY_PRINT)
		printf("Running partitioning\n new part (gets %d): ", (int)(columnsMap.size() * pSharesVector->at(0)));

	for(long unsigned int i = 0; i < columnsMap.size(); i++) {
		// Verify the location of the next column
		while(columnsMap.count(min_col + i + column_offset) == 0) {
			// Updated offset
			column_offset++;
		}

		// Grab next column
		std::priority_queue<T_ColumnV> cQ = columnsMap.at(min_col + i + column_offset);
		if(DEBUG_IT_COL_SWEEP)
			printf("%ld ", (min_col + i + column_offset));

		// Pop elements out of the column and add them to this partition
		while(!cQ.empty()) {
			auto it = pSolution->m_mPartitions.find(current_part);
			if(it != pSolution->m_mPartitions.end()) {
				// Add the top of cQ to this partition
				it->second.push_back(cQ.top().pV);
			}
			else {
				// Add a new partition vector and add the top of cQ to this partition
				pSolution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(current_part, std::vector<Vertex*>()));
				pSolution->m_mPartitions.at(current_part).push_back(cQ.top().pV);
			}

			cQ.pop();
		}

		running_count++;
		DBG_count++;

		// If we have added our quota of columns to this petition, then update
		if((current_part < (pSolution->m_nM - 1)) && (running_count >= (columnsMap.size() * pSharesVector->at(current_part)))) {
			running_count = 0;
			current_part++;
			if(DEBUG_IT_COL_SWEEP)
				printf("\n new part (gets %d): ", (int)(columnsMap.size() * pSharesVector->at(current_part)));
		}
	}

	// Sanity print
	if(DEBUG_IT_COL_SWEEP)
		printf("\n total count: %d, expected: %ld\n", DBG_count, columnsMap.size());
}

/*
 * Finds the columns, and stores them in columnsMap, for the given solution. Assumes that m_pA, m_pB,
 * and m_pC have already been assigned.
 */
int Iterative_Column_Sweeping::findColumns(Solution* solution, std::map<int, std::priority_queue<T_ColumnV>>* columnsMap) {
	// Minimum column indexes (we are guaranteed to have an index of 0 because this is where A is located)
	int min_col = 0;

	// Start pushing vertices into stacks for each row
	for(int i = 0; i < solution->m_nN; i++) {
		// Put vertex into a container to help track which row/column it belongs to
		Vertex* cV = solution->m_pVertexData + i;

		// Find which column to put this vertex in
		int col = getColumn(cV, solution->m_fR);
		std::map<int, std::priority_queue<T_ColumnV>>::iterator itC = columnsMap->find(col);
		if(itC != columnsMap->end()) {
			// Add this vertex to the column queue
			itC->second.push(cV);
		}
		else {
			if(DEBUG_IT_COL_SWEEP)
				printf("  new column: %d\n", col);
			columnsMap->insert(std::pair<int, std::priority_queue<T_ColumnV>>(col, std::priority_queue<T_ColumnV>()));
			columnsMap->at(col).push(cV);
		}

		// If necessary, update minimum column index
		if(min_col > col) {
			min_col = col;
		}

		if(DEBUG_IT_COL_SWEEP)
			printf(" %d => R%d\n", (solution->m_pVertexData + i)->nID, col);
	}

	if(DEBUG_IT_COL_SWEEP)
		printf(" min-column: %d\n", min_col);

	return min_col;
}

/*
 * Finds the columns that each vertex belongs to and puts them into a single list. The order of the list is
 * based off of the same order that findColumns() returns but with the resulting columns pushed (in ascending order)
 * into a single list.
 */
void Iterative_Column_Sweeping::findColumnList(Solution* solution, std::list<Vertex*>* columnsOrderedList) {
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

/*
 * Finds the best route using sweeping path-planning on partition p by alternating the order of referee vertices
 * A, B, and C. Assumes that the partition map already has all vertices for this partition in priority queues.
 */
void Iterative_Column_Sweeping::findBestSweepingRoute(Solution* solution, int p, std::list<Vertex*> &routeList) {
	// Ensure that the route list is cleared
	routeList.clear();

	// Determine orientation of A, B, and C.
	Vertex* minV = NULL;
	Vertex* midV = NULL;
	Vertex* topV = NULL;

	if((m_pA->fY < m_pB->fY) && (m_pA->fY < m_pC->fY)) {
		// A is the lowest
		minV = m_pA;
		if(m_pB->fY < m_pC->fY) {
			midV = m_pB;
			topV = m_pC;
		}
		else {
			midV = m_pC;
			topV = m_pB;
		}
	}
	else if((m_pB->fY < m_pA->fY) && (m_pB->fY < m_pC->fY)) {
		// B is the lowest
		minV = m_pB;
		if(m_pA->fY < m_pC->fY) {
			midV = m_pA;
			topV = m_pC;
		}
		else {
			midV = m_pC;
			topV = m_pA;
		}
	}
	else {
		// C is the lowest
		minV = m_pC;
		if(m_pA->fY < m_pB->fY) {
			midV = m_pA;
			topV = m_pB;
		}
		else {
			midV = m_pB;
			topV = m_pA;
		}
	}

	std::list<Vertex*> routeList1;
	// Solve path-planning with A := minV, B := topV, C := midV
	solveGivenDirection(solution, p, routeList1, minV, topV, midV);

	std::list<Vertex*> routeList2;
	// Solve path-planning with A := minV, B := midV, C := topV
	solveGivenDirection(solution, p, routeList2, minV, midV, topV);

	std::list<Vertex*> routeList3;
	// Solve path-planning with A := midV, B := topV, C := minV
	solveGivenDirection(solution, p, routeList3, midV, topV, minV);

	float dist1 = solution->GetTourDist(routeList1);
	float dist2 = solution->GetTourDist(routeList2);
	float dist3 = solution->GetTourDist(routeList3);

	std::list<Vertex*>* pBestRoute;
	if((dist1 < dist2) && (dist1 < dist3)) {
		pBestRoute = &routeList1;
	}
	else if((dist2 < dist1) && (dist2 < dist3)) {
		pBestRoute = &routeList2;
	}
	else {
		pBestRoute = &routeList3;
	}

	// Store the best found solution in routeList
	for(Vertex* v: (*pBestRoute)) {
		routeList.push_back(v);
	}
}

/*
 * Performs sweeping path-planning on the given partition p and partition map. Assumes that the
 * partition map already has all vertices for this partition in priority queues.
 *
 *  r 5 "test_baseline/plot_300.txt"
 */
void Iterative_Column_Sweeping::solveGivenDirection(Solution* solution, int p, std::list<Vertex*> &routeList, Vertex* pA, Vertex* pB, Vertex* pC) {
	// First add the depot to this route
	routeList.push_back(solution->m_pVertexData + (solution->m_nN + solution->m_nM + p));

	// Maps for the columns and rows of the solution
	std::map<int, std::priority_queue<T_ColumnV>> columnsMap;

	// minimum column and row indexes (we are guaranteed to have an index of 0 because this is where A and C are located)
	int min_col = 0;

	// Verify something didn't go wrong
	if(DEBUG_IT_COL_SWEEP)
		printf(" checking p = %d\n", p);
	if(solution->m_mPartitions.count(p) == 0) {
		fprintf(stderr, "[ERROR] : Iterative_Column_Sweeping::solveGivenDirection : partition p not in solution map\n p = %d\n", p);
		exit(1);
	}

	// Start pushing vertices into stacks for each row and column
	for(Vertex* v : solution->m_mPartitions.at(p)) {
		if(v->eVType == E_VertexType::e_Destination) {
			// Put vertex into a container to help track which row/column it belongs to
			T_ColumnV cV(v);
			T_RowV rV(v);

			// Find which column to put this vertex in
			int col = getColumn(v, pA, pB, solution->m_fR);
			std::map<int, std::priority_queue<T_ColumnV>>::iterator itC = columnsMap.find(col);
			if(itC != columnsMap.end()) {
				// Add this vertex to the column queue
				itC->second.push(cV);
			}
			else {
				if(DEBUG_IT_COL_SWEEP)
					printf("  new column: %d\n", col);
				columnsMap.insert(std::pair<int, std::priority_queue<T_ColumnV>>(col, std::priority_queue<T_ColumnV>()));
				columnsMap.at(col).push(cV);
			}

			// If necessary, update minimum column index
			if(min_col > col) {
				min_col = col;
			}
		}
	}

	// Make "out and back" sweeps in a straight line
	if(!(columnsMap.size() % 2)) { // Even number of columns
		int correct_column = 0;
		for(long unsigned int i = 0; i < columnsMap.size(); i++) {
			// Verify that there is an (ith + min_col)-numbered column
			while(!(columnsMap.count(min_col + i + correct_column) > 0)) {
				correct_column++;
			}

			// Determine if this is going "out" (even) or "back" (odd)
			if(!(i % 2)) {
				// The columns are in max-queues, so the even numbered columns (starting at 0th) need to be reversed
				std::stack<Vertex*> revCol;
				while(!columnsMap.at(min_col + i + correct_column).empty()) {
					revCol.push(columnsMap.at(min_col + i + correct_column).top().pV);
					columnsMap.at(min_col + i + correct_column).pop();
				}

				// Add this column to our solution
				while(!revCol.empty()) {
					routeList.push_back(revCol.top());

					revCol.pop();
				}
			}
			else {
				// Add this column to our solution
				while(!columnsMap.at(min_col + i + correct_column).empty()) {
					routeList.push_back(columnsMap.at(min_col + i + correct_column).top().pV);

					columnsMap.at(min_col + i + correct_column).pop();
				}
			}
		}
	}
	else {
		if(columnsMap.size() > 1) {
			int correct_column = 0;
			// Make "out and back" sweeps for the first n - 2 rows, then weave together the n-1st and nth rows.
			for(long unsigned int i = 0; i < (columnsMap.size() - 2); i++) {
				// Verify that there is an (ith + min_col)-numbered column
				while(!(columnsMap.count(min_col + i + correct_column) > 0)) {
					correct_column++;
				}

				// Determine if this is going "out" (even) or "back" (odd)
				if(!(i % 2)) {
					// The columns are in max-queues, so the even numbered columns (starting at 0th) need to be reversed
					std::stack<Vertex*> revCol;
					while(!columnsMap.at(min_col + i + correct_column).empty()) {
						revCol.push(columnsMap.at(min_col + i + correct_column).top().pV);
						columnsMap.at(min_col + i + correct_column).pop();
					}

					// Add this column to our solution
					while(!revCol.empty()) {
						routeList.push_back(revCol.top());

						revCol.pop();
					}
				}
				else {
					// Add this column to our solution
					while(!columnsMap.at(min_col + i + correct_column).empty()) {
						routeList.push_back(columnsMap.at(min_col + i + correct_column).top().pV);

						columnsMap.at(min_col + i + correct_column).pop();
					}
				}
			}

			// Verify that there is an ((columnsMap.size() - 2) + min_col)-numbered column
			while(!(columnsMap.count(min_col + (columnsMap.size() - 2) + correct_column) > 0)) {
				correct_column++;
			}

			std::priority_queue<T_ColumnV> penultimateCol = columnsMap.at(min_col + (columnsMap.size() - 2) + correct_column);

			// Verify that there is an ((columnsMap.size() - 1) + min_col)-numbered column
			while(!(columnsMap.count(min_col + (columnsMap.size() - 1) + correct_column) > 0)) {
				correct_column++;
			}

			std::priority_queue<T_ColumnV> lastCol = columnsMap.at(min_col + (columnsMap.size() - 1) + correct_column);

			// Alternate between the final two columns
			while(!lastCol.empty() && !penultimateCol.empty()) {
				if(getRow(lastCol.top().pV, pA, pB, pC, solution->m_fR) > getRow(penultimateCol.top().pV, pA, pB, pC, solution->m_fR)) {
					// Add top of lastCol next
					routeList.push_back(lastCol.top().pV);
					lastCol.pop();
				}
				else {
					// Add top of penultimateCol next
					routeList.push_back(penultimateCol.top().pV);
					penultimateCol.pop();
				}
			}

			// Empty whatever is left into a single straight line
			while(!penultimateCol.empty()) {
				routeList.push_back(penultimateCol.top().pV);

				penultimateCol.pop();
			}

			while(!lastCol.empty()) {
				routeList.push_back(lastCol.top().pV);

				lastCol.pop();
			}
		}
		else if(columnsMap.size() == 1) {
			// We have a single column, add a single, straight-line run
			std::vector<Vertex*> singleCol;
			while(!columnsMap.begin()->second.empty()) {
				singleCol.push_back(columnsMap.begin()->second.top().pV);
				columnsMap.begin()->second.pop();
			}

			// Determine which direction to run through the single column
			float front_first_dist = solution->GetDepotOfPartion(p)->GetDistanceTo(singleCol[0]) +
					solution->GetTerminalOfPartion(p)->GetDistanceTo(singleCol[singleCol.size()-1]);

			float back_first_dist = solution->GetDepotOfPartion(p)->GetDistanceTo(singleCol[singleCol.size()-1]) +
					solution->GetTerminalOfPartion(p)->GetDistanceTo(singleCol[0]);

			if(front_first_dist < back_first_dist) {
				// Push this single column onto the solution, in order given
				for(auto v : singleCol) {
					routeList.push_back(v);
				}
			}
			else {
				// Push this single column onto the solution, in reverse order
				for (std::vector<Vertex*>::reverse_iterator itV = singleCol.rbegin(); itV != singleCol.rend(); itV++) {
					routeList.push_back(*itV);
				}
			}
		}
		else {
			printf("[ERROR] : Given an empty column in Iterative_Column_Sweeping::solveGivenDirection\n");
			exit(1);
		}
	}

	// Add an arc from the final vertex to the terminal
	routeList.push_back(solution->m_pVertexData + (solution->m_nN + p));
}
