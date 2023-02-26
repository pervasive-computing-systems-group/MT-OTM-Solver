#include "Tree_Cutting.h"

Tree_Cutting::Tree_Cutting() {
}

Tree_Cutting::~Tree_Cutting() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Runs tree-following algorithm
 */
void Tree_Cutting::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("Run Tree-Cutting Algorithm\n\n");

	std::list<Vertex*> lstDynamMemVertices;

	/// Find MST on way-points only
	std::list<UDEdge> result;
	Graph_Theory_Algorithms::MST_Prims(solution->m_pVertexData, solution->m_nNumVertices - 2, result);

	// Sanity print
	if(PRINT_TREE) {
		FILE * pForestFile;
		pForestFile = fopen("forest_output.txt", "w");
		printf("Found MST on way-points:\n");
		for(UDEdge edge : result) {
			printf(" (%d, %d)\n", edge.p_oAVertex->nID, edge.p_oBVertex->nID);
			fprintf(pForestFile, "%d %d\n", edge.p_oAVertex->nID, edge.p_oBVertex->nID);
		}
		fclose(pForestFile);
	}

	// Fill in matrix using results from MST
	for(UDEdge edge : result) {
		int a = edge.p_oAVertex->nID;
		int b = edge.p_oBVertex->nID;
		if(a < b) {
			solution->m_pAdjMatrix[a][b] = true;
		}
		else {
			solution->m_pAdjMatrix[b][a] = true;
		}
	}

	/// Connect base-station and ideal-stop to tree
	int closestToDepot = 0;
	float bestDistToDepot = std::numeric_limits<float>::max();
	int closestToIS = 0;
	float bestDistToIS = std::numeric_limits<float>::max();

	// Find way-points closest to the depot and ideal-stop
	for(int i = 0; i < solution->m_nN; i++) {
		// Check depot
		if(bestDistToDepot > solution->m_pVertexData[i].GetDistanceTo(solution->GetDepotOfPartion(0))) {
			bestDistToDepot = solution->m_pVertexData[i].GetDistanceTo(solution->GetDepotOfPartion(0));
			closestToDepot = i;
		}
		// Check ideal-stop
		if(bestDistToIS > solution->m_pVertexData[i].GetDistanceTo(solution->GetTerminalOfPartion(0))) {
			bestDistToIS = solution->m_pVertexData[i].GetDistanceTo(solution->GetTerminalOfPartion(0));
			closestToIS = i;
		}
	}

	// Sanity print
	if(TCUT_DEBUG) {
		printf("Found closest to depot: %d, with %f m\n", closestToDepot, bestDistToDepot);
		printf("Found closest to ideal-stop: %d, with %f m\n", closestToIS, bestDistToIS);
	}

	// Add these to the tree adjacency matrix
	solution->m_pAdjMatrix[closestToDepot][solution->GetDepotOfPartion(0)->nID] = true;
	solution->m_pAdjMatrix[closestToIS][solution->GetTerminalOfPartion(0)->nID] = true;

	/// Find path from base-station to ideal-stop through tree
	int* parentList = new int[solution->m_nNumVertices];
	// Fill parent list with -1
	for(int i = 0; i < solution->m_nNumVertices; i++) {
		parentList[i] = -1;
	}
	parentList[solution->GetDepotOfPartion(0)->nID] = solution->GetDepotOfPartion(0)->nID;

	// Do DFS to find path from depot to ideal-stop
	if(!Graph_Theory_Algorithms::DFS_TopAdjMtrx(solution->m_pAdjMatrix, parentList, solution->m_nNumVertices,
			solution->GetDepotOfPartion(0)->nID, solution->GetTerminalOfPartion(0)->nID)) {
		fprintf(stderr, "[ERROR] TotalPath_Follower : Could not find path from depot to ideal-stop\n\n");

		// Print adjacency matrix
		for(int i = 0; i < solution->m_nNumVertices; i++) {
			for(int j = 0; j < solution->m_nNumVertices; j++) {
				printf(" %d", solution->m_pAdjMatrix[i][j]);
			}
			printf("\n");
		}
		printf("\n");

		// Hard-fail, don't let algorithm continue
		exit(1);
	}

	/// Short-cut tree
	std::vector<Vertex*> partition;
	// Add all vertices into a single partition
	partition.push_back(solution->GetDepotOfPartion(0));
	for(int i = 0; i < solution->m_nN; i++) {
		partition.push_back(solution->m_pVertexData + i);
	}
	partition.push_back(solution->GetTerminalOfPartion(0));
	solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(0, partition));
	solution->m_bPartitioned = true;
	// Run short-cutting algorithm
	{
		ShortCutTree oShortCutTree;
		oShortCutTree.RunPathPlanningAlgorithm(solution);
	}

	// Sanity print
	if(TCUT_DEBUG) {
		printf("Total path:\n");
		printf("\n");
		// Print adjacency matrix
		for(int i = 0; i < solution->m_nNumVertices; i++) {
			for(int j = 0; j < solution->m_nNumVertices; j++) {
				printf(" %d", solution->m_pAdjMatrix[i][j]);
			}
			printf("\n");
		}
		printf("\n");
	}

	// Path is in top adjacency matrix, use DFS to extract path (code already implemented there)
	for(int i = 0; i < solution->m_nNumVertices; i++) {
		parentList[i] = -1;
	}
	parentList[solution->GetDepotOfPartion(0)->nID] = solution->GetDepotOfPartion(0)->nID;
	if(!Graph_Theory_Algorithms::DFS_TopAdjMtrx(solution->m_pAdjMatrix, parentList, solution->m_nNumVertices,
			solution->GetDepotOfPartion(0)->nID, solution->GetTerminalOfPartion(0)->nID)) {
		fprintf(stderr, "[ERROR] TotalPath_Follower : Short-cut path does not connect depot to ideal-stop\n\n");

		// Print adjacency matrix
		for(int i = 0; i < solution->m_nNumVertices; i++) {
			for(int j = 0; j < solution->m_nNumVertices; j++) {
				printf(" %d", solution->m_pAdjMatrix[i][j]);
			}
			printf("\n");
		}
		printf("\n");

		// Hard-fail, don't let algorithm continue
		exit(1);
	}

	/// Make list of total-tour minus depot and terminal
	bool foundEnd = false;
	std::list<int> totalPath;
	{
		// Start with ideal-stop
		int last = solution->GetTerminalOfPartion(0)->nID;

		// Walk through path backwards
		while(parentList[last] != solution->GetDepotOfPartion(0)->nID) {
			// Grab next stop
			int next = parentList[last];
			// Add to stack
			totalPath.push_front(next);
			// Update
			last = next;
		}
	}

	// Sanity print
	if(TCUT_DEBUG) {
		printf("Total Path has %ld stops\n", totalPath.size());
		std::list<int>::iterator prev = totalPath.begin();
		std::list<int>::iterator next = totalPath.begin();
		next++;

		while(next != totalPath.end()) {
			printf(" ( %d, %d)\n", *prev, *next);
			prev = next;
			next++;
		}
		printf("\n");
	}

	/// Determine the length of the total path
	double totalPath_length = 0.0;
	{
		std::list<int>::iterator prev = totalPath.begin();
		std::list<int>::iterator next = totalPath.begin();
		next++;

		while(next != totalPath.end()) {
			totalPath_length += solution->m_pVertexData[*prev].GetDistanceTo(solution->m_pVertexData + *next);
			prev = next;
			next++;
		}
	}

	if(SANITY_PRINT)
		printf("Total Path distance = %f\n\n", totalPath_length);

	/// Start cutting the tree into m-parts until the solution doesn't improve
	int m = 1;
	bool increate_m;
	double best_total_time = std::numeric_limits<float>::max();
	do{
		if(SANITY_PRINT)
			printf("Cut tree into m = %d parts\n", m);
		increate_m = false;

		// Each subtour should be 1/m in length
		double one_mth_dist = totalPath_length/m;
		double residual = 0;
		// Array of sub-tours
		std::vector<std::list<Vertex*>> tours;
		// Array of sub-tour speeds
		std::vector<float> speeds;

		// Sanity print
		if(TCUT_DEBUG)
			printf("Build M = %d subtours, each %f long\n", m, one_mth_dist);

		std::list<int>::iterator prev = totalPath.begin();
		std::list<int>::iterator next = totalPath.begin();
		do {
			// Start new tour
			std::list<Vertex*> subtour;
//			double subtour_dist = 0;
			double subtour_dist = residual;

			// Add stops to subtours until we are as close as possible to 1/m distance
			do {
				if(subtour.size() <= 0) {
					// Tour is empty, add the first stop
					subtour.push_back(solution->m_pVertexData + (*next));
				}
				else {
					// Determine the distance gained by adding the next vertex
					double subtour_prime = subtour_dist + subtour.back()->GetDistanceTo(solution->m_pVertexData + (*next));
					// Verify if this is a good decision
					if(subtour_prime <= (one_mth_dist + ROUND_ERROR)) {
						// Add to subtour
						subtour.push_back(solution->m_pVertexData + (*next));
						subtour_dist = subtour_prime;
					}
					else {
						residual = subtour_prime - one_mth_dist;
						break;
					}
				}
				prev = next;
				next++;
			} while(next != totalPath.end());
			tours.push_back(subtour);
		} while(next != totalPath.end());

		// Sanity print
		if(TCUT_DEBUG) {
			printf("Subtours:\n");
			for(std::list<Vertex*> subtour : tours) {
				for(Vertex* v : subtour) {
					printf(" %d", v->nID);
				}
				printf("\n");
			}
		}

		// Add depots and terminals
		for(unsigned long int i = 0; i < tours.size(); i++) {
			if(i == 0) {
				// First sub-tour, add the base station
				tours.at(i).push_front(solution->GetDepotOfPartion(i));
			}
			else {
				// Create a new depot
				Vertex* depot = new Vertex;
				lstDynamMemVertices.push_back(depot);
				Vertex* last_term = tours.at(i-1).back();

				depot->fX = last_term->fX + BATTERY_SWAP_TIME*solution->m_tBSTrajectory.mX;
				depot->fY = 0;
				depot->eVType = E_VertexType::e_Depot;

				// Add this new depot to the tour
				tours.at(i).push_front(depot);
			}

			// Create a depot for this subtour
			Vertex* tempV = new Vertex;
			lstDynamMemVertices.push_back(tempV);
			double v = V_MAX;
			determineTerminalLocation(tours.at(i), solution->m_tBSTrajectory.mX, v, tempV);

			// Add terminal to sub-tour
			tours.at(i).push_back(tempV);

			// Determine length of new sub-tour
			float sub_dist = getSubtourDist(tours.at(i));

			// Verify distance
			if(sub_dist > DIST_OPT) {
				// We went too far..

				if(SPEED_SCHEULING) {
					/// Try schedule velocity for this sub-tour
					// Ensure consistency. sub_dist is based on V_max, but we may not be able to go V_max
					bool consistent = false;
					while(!consistent) {
						float distGap_fourth = (DIST_MAX - DIST_OPT)/4;
						if(sub_dist <= (DIST_OPT + distGap_fourth)) {
							if(TCUT_DEBUG)
								printf(" sub-tour falls in first quarter of flexible distance: %f\n", sub_dist);

							v = sqrt(36791437195.0 - 10557000.0 * (DIST_OPT + distGap_fourth)) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							tours.at(i).pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(tours.at(i), solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							tours.at(i).push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(tours.at(i));
							if(new_sub_dist <= (DIST_OPT + distGap_fourth)) {
								// Solution is consistent
								consistent = true;
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else if(sub_dist <= (DIST_OPT + (distGap_fourth*2))) {
							if(TCUT_DEBUG)
								printf(" sub-tour falls in second quarter of flexible distance: %f\n", sub_dist);

							v = sqrt(36791437195.0 - 10557000.0 * (DIST_OPT + (distGap_fourth*2))) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							tours.at(i).pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(tours.at(i), solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							tours.at(i).push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(tours.at(i));
							if(new_sub_dist <= (DIST_OPT + (distGap_fourth*2))) {
								// Solution is consistent
								consistent = true;
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else if(sub_dist <= (DIST_OPT + (distGap_fourth*3))) {
							if(TCUT_DEBUG)
								printf(" sub-tour falls in third quarter of flexible distance: %f\n", sub_dist);

							v = sqrt(36791437195.0 - 10557000.0 * (DIST_OPT + (distGap_fourth*3))) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							tours.at(i).pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(tours.at(i), solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							tours.at(i).push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(tours.at(i));
							if(new_sub_dist <= (DIST_OPT + (distGap_fourth*3))) {
								// Solution is consistent
								consistent = true;
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else if(sub_dist <= (DIST_MAX)) {
							if(TCUT_DEBUG)
								printf(" sub-tour falls in last quarter of flexible distance: %f\n", sub_dist);

							v = sqrt(36791437195.0 - 10557000.0 * DIST_MAX) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							tours.at(i).pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(tours.at(i), solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							tours.at(i).push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(tours.at(i));
							if(new_sub_dist <= DIST_MAX) {
								// Solution is consistent
								consistent = true;
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else {
							if(TCUT_DEBUG)
								printf(" sub-tour is too long: %f\n", sub_dist);

							increate_m = true;
							consistent = true;
						}
					}
					if(TCUT_DEBUG)
						printf(" * go %f m/s\n", v);
				}
				else {
					increate_m = true;
				}
			}
			else if (sub_dist <= DIST_OPT) {
				// Sanity print
				if(TCUT_DEBUG) {
					printf("New sub-tour, size = %ld\n", tours.at(i).size());
					for(Vertex* n : tours.at(i)) {
						printf(" %d: (%f, %f)", n->nID, n->fX, n->fY);
					}
					printf("\n * distance = %f\n", getSubtourDist(tours.at(i)));
				}
			}
			else {
				if(TCUT_DEBUG)
					printf(" sub-tour is too long: %f\n", sub_dist);

				increate_m = true;
			}

			speeds.push_back(v);
		}

		if(!increate_m) {
			// Check to see if we improved the solution
			if(best_total_time > tours.back().back()->fX/solution->m_tBSTrajectory.mX) {
				// Sanity print
				if(SANITY_PRINT)
					printf("* Found better solution!\n* Total time: %f\n", tours.back().back()->fX/solution->m_tBSTrajectory.mX);

				// Give new tour-set to solution
				solution->BuildCompleteSolution(tours, speeds);

				// Increase m in an attempt to further improve the solution
				increate_m = true;
				best_total_time = tours.back().back()->fX/solution->m_tBSTrajectory.mX;
			}
			else {
				// This value of m didn't improve the solution, stop algorithm
				if(SANITY_PRINT)
					printf("* Tour didn't improve, stop algorithm\n");
			}
		}
		else {
			if(SANITY_PRINT)
				printf("* Tour unsatisfactory, increase m\n");
		}

		m++;
	} while(increate_m && (m <= solution->m_nN));

	/// Data clean-up
	for(Vertex* ptr : lstDynamMemVertices) {
		delete ptr;
	}
	delete[] parentList;
}


//***********************************************************
// Private Member Functions
//***********************************************************
