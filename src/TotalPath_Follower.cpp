#include "TotalPath_Follower.h"

TotalPath_Follower::TotalPath_Follower() {
}

TotalPath_Follower::~TotalPath_Follower() {
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
void TotalPath_Follower::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("Run Following Algorithm\n\n");

	std::list<Vertex*> lstDynamMemVertices;

	/// Find MST on way-points only
	std::list<UDEdge> result;
	Graph_Theory_Algorithms::MST_Prims(solution->m_pVertexData, solution->m_nNumVertices - 2, result);

	// Sanity print
	if(SANITY_PRINT) {
		FILE * pForestFile;
		pForestFile = fopen("forest_output.txt", "w");
		printf("Found MST on way-points:\n");
		for(UDEdge edge : result) {
			printf(" (%d, %d)\n", edge.p_oAVertex->nID, edge.p_oBVertex->nID);
			fprintf(pForestFile, "%d %d\n", edge.p_oAVertex->nID, edge.p_oBVertex->nID);
		}
		fclose(pForestFile);
	}

	if(solution->m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		// The following assumes a linear UGV... hard fail
		printf("[ERROR] : TotalPath_Follower::RunAlgorithm : Given non-linear UGV, expected linear\n");
		exit(0);
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
	printf("Found closest to depot: %d, with %f m\n", closestToDepot, bestDistToDepot);
	printf("Found closest to ideal-stop: %d, with %f m\n", closestToIS, bestDistToIS);

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
	{
		ShortCutTree oShortCutTree;
		oShortCutTree.RunPathPlanningAlgorithm(solution);
	}

	// Sanity print
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

	// Make list of total-tour
	bool foundEnd = false;
	std::list<int> totalPath;
	{
		// Start with ideal-stop
		int last = solution->GetTerminalOfPartion(0)->nID;

		printf("Building queue:\n");
		// Walk through path backwards
		while(last != solution->GetDepotOfPartion(0)->nID) {
			// Grab next stop
			int next = parentList[last];
			printf(" push %d\n", next);
			// Add to stack
			totalPath.push_front(next);
			// Update
			last = next;
		}
	}

	// Sanity print
	{
		printf("Total Path has %ld stops\n", totalPath.size());
		std::list<int>::iterator prev = totalPath.begin();
		std::list<int>::iterator next = totalPath.begin();
		next++;

		while(next != totalPath.end()) {
			printf(" ( %d, %d)\n", *prev, *next);
			prev = next;
			next++;
		}
	}

	/// Start building up sub-tours
	// Array of sub-tours
	std::vector<std::list<Vertex*>> tours;
	// Array of sub-tour speeds
	std::vector<float> speeds;

	/// While not at end of the total path
	while(totalPath.size() > 0) {
		// Start new tour
		std::list<Vertex*> subtour;
		float v = V_MAX;

		// Sanity print
		printf("Starting new sub-tour\n");

		if(tours.size() == 0) {
			// First sub-tour, add the base station
			subtour.push_back(solution->m_pVertexData + totalPath.front());

			// Advance stack
			totalPath.pop_front();

			// Sanity print
			printf(" * Added OG depot to tour\n");
		}
		else {
			// Create a new depot
			Vertex* depot = new Vertex;
			lstDynamMemVertices.push_back(depot);
			Vertex* last_term = tours.at(tours.size() - 1).back();

			depot->fX = last_term->fX + BATTERY_SWAP_TIME*solution->m_tBSTrajectory.mX;
			depot->fY = 0;
			depot->eVType = E_VertexType::e_Depot;

			// Add this new depot to the tour
			subtour.push_back(depot);

			// Sanity print
			printf(" * Added new depot to tour\n");
		}

		/// Add stops to the sub-tour
		bool build_sub = true;
		do {
			// Sanity print
			printf(" * Adding %d to tour\n", totalPath.front());

			// Add next stop
			subtour.push_back(solution->m_pVertexData + totalPath.front());

			// Find potential terminal with this new stop added to path
			Vertex* tempV = new Vertex;
			lstDynamMemVertices.push_back(tempV);
			determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, V_MAX, tempV);

			// Add terminal to sub-tour
			subtour.push_back(tempV);

			// Determine length of new sub-tour
			float sub_dist = getSubtourDist(subtour);

			// Verify distance
			if(sub_dist > DIST_OPT) {
				// We went too far..

				if(SPEED_SCHEULING) {
					/// Try to add this stop to the sub-tour by going slower (not always the best solution)
					// Ensure consistency. sub_dist is based on V_max, but we may not be able to go V_max
					bool consistent = false;
					while(!consistent) {
						float distGap_fourth = (DIST_MAX - DIST_OPT)/4;
						if(sub_dist <= (DIST_OPT + distGap_fourth)) {
							printf(" sub-tour falls in first quarter of flexible distance: %f\n", sub_dist);
							v = sqrt(36791437195.0 - 10557000.0 * (DIST_OPT + distGap_fourth)) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							subtour.pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							subtour.push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(subtour);
							if(new_sub_dist <= (DIST_OPT + distGap_fourth)) {
								// Solution is consistent
								consistent = true;
								// Advance stack
								totalPath.pop_front();
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else if(sub_dist <= (DIST_OPT + (distGap_fourth*2))) {
							printf(" sub-tour falls in second quarter of flexible distance: %f\n", sub_dist);
							v = sqrt(36791437195.0 - 10557000.0 * (DIST_OPT + (distGap_fourth*2))) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							subtour.pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							subtour.push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(subtour);
							if(new_sub_dist <= (DIST_OPT + (distGap_fourth*2))) {
								// Solution is consistent
								consistent = true;
								// Advance stack
								totalPath.pop_front();
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else if(sub_dist <= (DIST_OPT + (distGap_fourth*3))) {
							printf(" sub-tour falls in third quarter of flexible distance: %f\n", sub_dist);
							v = sqrt(36791437195.0 - 10557000.0 * (DIST_OPT + (distGap_fourth*3))) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							subtour.pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							subtour.push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(subtour);
							if(new_sub_dist <= (DIST_OPT + (distGap_fourth*3))) {
								// Solution is consistent
								consistent = true;
								// Advance stack
								totalPath.pop_front();
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else if(sub_dist <= (DIST_MAX)) {
							v = sqrt(36791437195.0 - 10557000.0 * DIST_MAX) / 10557.0 + 129221.0/10557.0;
							printf(" sub-tour falls in last quarter of flexible distance: %f\n", sub_dist);
							v = sqrt(36791437195.0 - 10557000.0 * DIST_MAX) / 10557.0 + 129221.0/10557.0;

							// Remove the new depot
							subtour.pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							subtour.push_back(tempV);

							// Verify that going a slower speed keeps us in this speed tier
							float new_sub_dist = getSubtourDist(subtour);
							if(new_sub_dist <= DIST_MAX) {
								// Solution is consistent
								consistent = true;
								// Advance stack
								totalPath.pop_front();
							}
							else {
								sub_dist = new_sub_dist;
							}
						}
						else {
							printf(" sub-tour is too long: %f\n", sub_dist);
							v = V_MAX;
							// Remove the last stop added
							subtour.pop_back();
							// Remove the new depot
							subtour.pop_back();
							// Find a depot for the set velocity
							Vertex* tempV = new Vertex;
							lstDynamMemVertices.push_back(tempV);
							determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, v, tempV);
							// Add terminal to sub-tour
							subtour.push_back(tempV);
							consistent = true;
						}
					}
					printf(" * go %f m/s\n", v);
				}
				else {
					/// Don't schedule velocity, we only go full-speed!
					printf(" sub-tour is too long: %f\n", sub_dist);
					v = V_MAX;
					// Remove the new depot
					subtour.pop_back();
					// Remove the last stop added
					subtour.pop_back();
					// Find terminal for remaining path
					Vertex* tempV = new Vertex;
					lstDynamMemVertices.push_back(tempV);
					determineTerminalLocation(subtour, solution->m_tBSTrajectory.mX, V_MAX, tempV);
					// Add terminal to sub-tour
					subtour.push_back(tempV);
				}

				// Done with sub-tour
				build_sub = false;
			}
			else if (sub_dist <= 3000.0) {	// Should be default
				// Advance stack
				totalPath.pop_front();
				if(totalPath.size() == 0) {
					// No more way-points to add to sub-tour
					build_sub = false;
				}
				else {
					// Remove the new depot
					subtour.pop_back();
				}
			}
		} while(build_sub);

		// Sanity print
		printf("New sub-tour, size = %ld\n", subtour.size());
		for(Vertex* n : subtour) {
			printf(" %d: (%f, %f)", n->nID, n->fX, n->fY);
		}
		printf("\n * distance = %f\n", getSubtourDist(subtour));

		speeds.push_back(v);
		tours.push_back(subtour);
	}

	// Sanity print
	printf("Total time: %f\n", tours.back().back()->fX/solution->m_tBSTrajectory.mX);

	/// Give new tour-set to solution
	solution->BuildCompleteSolution(tours, speeds);

	/// Data clean-up
	for(Vertex* ptr : lstDynamMemVertices) {
		delete ptr;
	}
	delete[] parentList;
}


//***********************************************************
// Private Member Functions
//***********************************************************
