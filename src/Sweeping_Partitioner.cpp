#include "Sweeping_Partitioner.h"

Sweeping_Partitioner::Sweeping_Partitioner() {
	m_bCreatesTree = false;
}

Sweeping_Partitioner::~Sweeping_Partitioner() {
}

//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm for a sweeping approach. This assumes that the
 * graph representing the search area is at least 2-connected.
 */
void Sweeping_Partitioner::RunAlgorithm(Solution* solution) {
	// Ensure that there isn't already a partition
	solution->m_mPartitions.clear();

	// Maps for the columns and rows of the solution
	std::map<int, std::priority_queue<T_ColumnV>> columnsMap;
	std::map<int, std::priority_queue<T_RowV>> rowsMap;

	// Find the reference triangle
	findReferenceTriangle(solution, solution->m_fR);

	if((m_pB == NULL) || (m_pC == NULL)) {
		// Graph isn't 2-connected!!!
		// Sanity print
		printf("Sweeping partitioner\n -Graph isn't 2-connected!!\n");

		// Do the same thing as the Generic partitioner
		//  (put vertices into partitions in order they appear)
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

		return;
	}

	// Orient A, B, and C
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

	// Assign A at the bottom, B on top, and C in the middle
	m_pA = minV;
	m_pB = topV;
	m_pC = midV;

	// Sanity print
	printf("Sweeping partitioner\n a: %d\n b: %d\n c: %d\n", m_pA->nID, m_pB->nID, m_pC->nID);

	// minimum column and row indexes (we are guaranteed to have an index of 0 because this is where A and C are located)
	int min_col = 0;
	int min_row = 0;

	// Start pushing vertices into stacks for each row and column
	for(int i = 0; i < solution->m_nN; i++) {
		// Put vertex into a container to help track which row/column it belongs to
		T_ColumnV cV(solution->m_pVertexData + i);
		T_RowV rV(solution->m_pVertexData + i);

		// Find which column to put this vertex in
		int col = getColumn(solution->m_pVertexData + i, solution->m_fR);
		std::map<int, std::priority_queue<T_ColumnV>>::iterator itC = columnsMap.find(col);
		if(itC != columnsMap.end()) {
			// Add this vertex to the column queue
			itC->second.push(cV);
		}
		else {
//			printf("  new column: %d\n", col);
			columnsMap.insert(std::pair<int, std::priority_queue<T_ColumnV>>(col, std::priority_queue<T_ColumnV>()));
			columnsMap.at(col).push(cV);
		}

		// If necessary, update minimum column index
		if(min_col > col) {
			min_col = col;
		}

		// Find which row to put this vertex in
		int row = getRow(solution->m_pVertexData + i, solution->m_fR);
		std::map<int, std::priority_queue<T_RowV>>::iterator itR = rowsMap.find(row);
		if(itR != rowsMap.end()) {
			// Add this vertex to the column queue
			itR->second.push(rV);
		}
		else {
//			printf("  new row: %d\n", row);
			rowsMap.insert(std::pair<int, std::priority_queue<T_RowV>>(row, std::priority_queue<T_RowV>()));
			rowsMap.at(row).push(rV);
		}

		// If necessary, update minimum row index
		if(min_row > row) {
			min_row = row;
		}

//		printf(" %d: %d, %d\n", (solution->m_pVertexData + i)->nID, col, row);
	}

	// Sanity print. NOTE: this empties the queues!!! Don't us this normally!!!!!
//	printf("Columns\n");
//	for(auto pair : columnsMap) {
//		printf(" col: %d\n  ", pair.first);
//		while(!pair.second.empty()) {
//			printf("%d ", pair.second.top().pV->nID);
//			pair.second.pop();
//		}
//		printf("\n");
//	}
//	printf("\nRows\n");
//	for(auto pair : rowsMap) {
//		printf(" row: %d\n  ", pair.first);
//		while(!pair.second.empty()) {
//			printf("%d ", pair.second.top().pV->nID);
//			pair.second.pop();
//		}
//		printf("\n");
//	}

	// Divide up the columns
	int columns_per_part = columnsMap.size()/solution->m_nM;
	int exptra_columns = columnsMap.size() % solution->m_nM;

	int current_part = 0;
	int running_count = 0;
	for(long unsigned int i = 0; i < columnsMap.size(); i++) {
		std::priority_queue<T_ColumnV> cQ = columnsMap.at(min_col + i);

		// Pop elements out of the column and add them to this partition
		while(!cQ.empty()) {
			auto it = solution->m_mPartitions.find(current_part);
			if(it != solution->m_mPartitions.end()) {
				// Add the top of cQ to this partition
				it->second.push_back(cQ.top().pV);
			}
			else {
				// Add a new partition vector and add the top of cQ to this partition
				solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(current_part, std::vector<Vertex*>()));
				solution->m_mPartitions.at(current_part).push_back(cQ.top().pV);
			}

			cQ.pop();
		}

		running_count++;

		// If we have added our quota of columns to this petition, then update
		if(running_count == columns_per_part) {
			// Check if we should add another column to keep things spread out
			if(exptra_columns > 0) {
				i++;
				std::priority_queue<T_ColumnV> extraCQ = columnsMap.at(min_col + i);
				// Pop elements out of the column and add them to this partition
				while(!extraCQ.empty()) {
					solution->m_mPartitions.at(current_part).push_back(extraCQ.top().pV);
					extraCQ.pop();
				}
				exptra_columns--;
			}

			// Add the corresponding depot and terminal to this partition
			solution->m_mPartitions.at(current_part).push_back(solution->GetTerminalOfPartion(current_part));
			solution->m_mPartitions.at(current_part).push_back(solution->GetDepotOfPartion(current_part));

			running_count = 0;
			current_part++;
		}
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************
