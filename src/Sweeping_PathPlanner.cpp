#include "Sweeping_PathPlanner.h"

Sweeping_PathPlanner::Sweeping_PathPlanner() {
	// Seed rand()
	srand(time(NULL));
}

Sweeping_PathPlanner::~Sweeping_PathPlanner() {
}

//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Uses a random ordering algorithm to "solve" the Path TSP on each partition in solution
 */
void Sweeping_PathPlanner::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\nRunning Sweeping Path-Planning Algorithm\n");
	for(int i = 0; i < solution->m_nM; i++) {
		// Run genetic algorithm
		RandomAlgorithm(solution, i);
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************

// Runs a simple genetic algorithm that solves Path TSP in each partitioning in pathSolution
void Sweeping_PathPlanner::RandomAlgorithm(Solution* solution, int p) {
	m_pA = NULL;
	m_pB = NULL;
	m_pC = NULL;

	// Grab the first reference vertex
	int _i = 0;
	while(true) {
		if(solution->m_mPartitions.at(p).at(_i)->eVType == E_VertexType::e_Destination) {
			m_pA = solution->m_mPartitions.at(p).at(_i);
			break;
		}
		else {
			_i++;
		}
	}

	// Search for two adjacent vertices that form a reference triangle
	for(Vertex* v : solution->m_mPartitions.at(p)) {
		if(v->eVType == E_VertexType::e_Destination) {
			if((v != m_pA) && (m_pA->GetDistanceTo(v) < (solution->m_fR + EPSILON_R))) {
				// Found a vertex one-hop away from a
				if(m_pB == NULL) {
					// Make this vertex b
					m_pB = v;
				}
				else {
					// Verify that this vertex is also one-hop away from b
					if(m_pB->GetDistanceTo(v) < (solution->m_fR + EPSILON_R)) {
						// Found vertex c
						m_pC = v;
					}
				}
			}
		}
	}

	if((m_pB == NULL) || (m_pC == NULL)) {
		// This partition is not 2-connected!
		Vertex* depot = solution->m_pVertexData + (solution->m_nN + solution->m_nM + p);
		Vertex* previous = depot;
		Vertex* terminal = solution->m_pVertexData + (solution->m_nN + p);

		// Sanity print
		printf("Sweeping Path-Planner on Partition %d\n -Partition isn't 2-connected\n", p);
		// This isn't very interesting to solve...
		std::priority_queue<T_ColumnV> columnsQueue;
		// Start pushing vertices into stacks for each row and column
		for(Vertex* v : solution->m_mPartitions.at(p)) {
			if(v->eVType == E_VertexType::e_Destination) {
				// Put vertex into a container to help track which row/column it belongs to
				T_ColumnV cV(v);
				columnsQueue.push(cV);
			}
		}

		std::list<Vertex*> maxList;
		while(!columnsQueue.empty()) {
			maxList.push_back(columnsQueue.top().pV);
			columnsQueue.pop();
		}

		// Determine which vertex should connect to the first node
		float d_to_top = depot->GetDistanceTo(maxList.front());
		float t_to_top = terminal->GetDistanceTo(maxList.front());
		float d_to_bottom = depot->GetDistanceTo(maxList.back());
		float t_to_bottom = terminal->GetDistanceTo(maxList.back());

		if((d_to_top + t_to_bottom) < (d_to_bottom + t_to_top)) {
			// Run through the list, starting with connecting the depot to the top vertex
			for(Vertex * v : maxList) {
				// Create a link from previous vertex to the vertex at the top of the list (a -> b)
				int a = previous->nID;
				int b = v->nID;

				if(a < b) {
					solution->m_pAdjMatrix[a][b] = true;
				}
				else {
					solution->m_pAdjMatrix[b][a] = true;
				}

				previous = v;
			}
		}
		else {
			// Run through the list backwards, starting with connecting the depot to the bottom vertex
			for(std::list<Vertex*>::reverse_iterator rIt = maxList.rbegin(); rIt != maxList.rend(); rIt++) {
				// Create a link from previous vertex to the vertex at the top of the list (a -> b)
				int a = previous->nID;
				int b = (*rIt)->nID;

				if(a < b) {
					solution->m_pAdjMatrix[a][b] = true;
				}
				else {
					solution->m_pAdjMatrix[b][a] = true;
				}

				previous = (*rIt);
			}
		}

		// Add an arc from the final vertex to the terminal
		solution->m_pAdjMatrix[previous->nID][terminal->nID] = true;

		// This shouldn't happen!! If the code ever stops here, fix the above to continue looking for a
		// reference triangle or verify that we actually don't have one for this graph!!!
		printf("[ERROR] : Couldn't find a reference triangle in Sweeping-Path-Planner\n");
		exit(1);
	}

	// Sanity print
	printf("Sweeping Path-Planner on Partition %d\n a: %d\n b: %d\n c: %d\n", p, m_pA->nID, m_pB->nID, m_pC->nID);

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
	m_pA = minV;
	m_pB = topV;
	m_pC = midV;
	solveGivenDirection(solution, p, routeList1);
	float route1Dist = solution->GetTourDist(routeList1);
	printf(" Found tour distance of: %f\n", route1Dist);

	std::list<Vertex*> routeList2;
	m_pA = minV;
	m_pB = midV;
	m_pC = topV;
	solveGivenDirection(solution, p, routeList2);
	float route2Dist = solution->GetTourDist(routeList2);
	printf(" Found tour distance of: %f\n", route2Dist);

	std::list<Vertex*> routeList3;
	m_pA = midV;
	m_pB = topV;
	m_pC = minV;
	solveGivenDirection(solution, p, routeList3);
	float route3Dist = solution->GetTourDist(routeList3);
	printf(" Found tour distance of: %f\n", route3Dist);

	// Determine which direction worked the best
	std::list<Vertex*>* bestRoute = NULL;
	if((route1Dist < route2Dist) && (route1Dist < route3Dist)) {
		bestRoute = &routeList1;
		printf(" Picking Route 1\n\n");
	}
	else if((route2Dist < route1Dist) && (route2Dist < route3Dist)) {
		bestRoute = &routeList2;
		printf(" Picking Route 2\n\n");
	}
	else {
		bestRoute = &routeList3;
		printf(" Picking Route 3\n\n");
	}


	// Iterate through the best route list and add it to the adjacency matrix
	std::list<Vertex *>::iterator it = bestRoute->begin();
	Vertex * previous = (*it);
	it++;
	for(; it != bestRoute->end(); it++) {
		Vertex* next = (*it);
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

/*
 * Performs sweeping path-planning using columns and rows.
 */
void Sweeping_PathPlanner::solveGivenDirection(Solution* solution, int p, std::list<Vertex*> &routeList) {
	// First add the depot to this route
	routeList.push_back(solution->m_pVertexData + (solution->m_nN + solution->m_nM + p));

	// Maps for the columns and rows of the solution
	std::map<int, std::priority_queue<T_ColumnV>> columnsMap;
	std::map<int, std::priority_queue<T_RowV>> rowsMap;

	// minimum column and row indexes (we are guaranteed to have an index of 0 because this is where A and C are located)
	int min_col = 0;
	int min_row = 0;

	// Start pushing vertices into stacks for each row and column
	for(Vertex* v : solution->m_mPartitions.at(p)) {
		if(v->eVType == E_VertexType::e_Destination) {
			// Put vertex into a container to help track which row/column it belongs to
			T_ColumnV cV(v);
			T_RowV rV(v);

			// Find which column to put this vertex in
			int col = getColumn(v, solution->m_fR);
			std::map<int, std::priority_queue<T_ColumnV>>::iterator itC = columnsMap.find(col);
			if(itC != columnsMap.end()) {
				// Add this vertex to the column queue
				itC->second.push(cV);
			}
			else {
				if(DEBUG_SWEEP_PP)
					printf("  new column: %d\n", col);
				columnsMap.insert(std::pair<int, std::priority_queue<T_ColumnV>>(col, std::priority_queue<T_ColumnV>()));
				columnsMap.at(col).push(cV);
			}

			// If necessary, update minimum column index
			if(min_col > col) {
				min_col = col;
			}

			// Find which row to put this vertex in
			int row = getRow(v, solution->m_fR);
			std::map<int, std::priority_queue<T_RowV>>::iterator itR = rowsMap.find(row);
			if(itR != rowsMap.end()) {
				// Add this vertex to the column queue
				itR->second.push(rV);
			}
			else {
				if(DEBUG_SWEEP_PP)
					printf("  new row: %d\n", row);
				rowsMap.insert(std::pair<int, std::priority_queue<T_RowV>>(row, std::priority_queue<T_RowV>()));
				rowsMap.at(row).push(rV);
			}

			// If necessary, update minimum row index
			if(min_row > row) {
				min_row = row;
			}

			if(DEBUG_SWEEP_PP)
				printf(" %d: %d, %d\n", v->nID, col, row);
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
				if(getRow(lastCol.top().pV, solution->m_fR) > getRow(penultimateCol.top().pV, solution->m_fR)) {
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
		else {
			// Something when wrong here...
			printf("[ERROR] : Sweeping-PathPlanner, graph is 2-connected but only has one column!\n a: %d, b: %d, c: %d\n",
					m_pA->nID, m_pB->nID, m_pC->nID);
			exit(0);
		}
	}

	// Add an arc from the final vertex to the terminal
	routeList.push_back(solution->m_pVertexData + (solution->m_nN + p));
}
