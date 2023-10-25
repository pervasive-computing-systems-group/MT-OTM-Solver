#include "Tree_Cutting_LKH.h"

Tree_Cutting_LKH::Tree_Cutting_LKH() {
}

Tree_Cutting_LKH::~Tree_Cutting_LKH() {
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
void Tree_Cutting_LKH::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("Run Tree-Cutting Algorithm with LKH Heuristic\n\n");


	if(solution->m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		// The following assumes a linear UGV... hard fail
		printf("[ERROR] : Tree_Cutting_LKH::RunAlgorithm : Given non-linear UGV, expected linear\n");
		exit(0);
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
	if(TCUT_LKH_DEBUG) {
		printf(" Initial picks: a = %d, b = %d\n", closestToDepot, closestToIS);
	}

	if(closestToDepot == closestToIS) {
		// This isn't an what we want...
		float bestDistToIS = std::numeric_limits<float>::max();

		//Keep closestToDepot fixed, adjust closestToIS
		for(int i = 0; i < solution->m_nN; i++) {
			if((bestDistToIS > solution->m_pVertexData[i].GetDistanceTo(solution->GetTerminalOfPartion(0))) &&
					(i != closestToDepot)) {
				bestDistToIS = solution->m_pVertexData[i].GetDistanceTo(solution->GetTerminalOfPartion(0));
				closestToIS = i;
			}
		}
	}

	// Sanity print
	if(TCUT_LKH_DEBUG) {
		printf("Found closest to depot: %d, with %f m\n", closestToDepot, bestDistToDepot);
		printf("Found closest to ideal-stop: %d, with %f m\n", closestToIS, bestDistToIS);
	}

	/// Find fixed-Hamiltonian path from closestToDepot to closestToIS
	solution->PrintLKHDataFHPP(closestToDepot, closestToIS);

	if(TCUT_LKH_DEBUG)
		printf("Running LKH\n");
	std::system("./LKH-3.0.6/LKH FixedHPP.par");

	if(TCUT_LKH_DEBUG)
		printf("Found the following solution:\n");

	// Open file with results
	std::ifstream file("LKH_output.dat");
	// Remove the first few lines...
	std::string line;
	for(int i = 0; i < 6; i++) {
		std::getline(file, line);
	}

	/// Make list of total-tour minus depot and terminal
	std::list<int> totalPath;

	// Start parsing the data
	for(auto i = 0; i < solution->m_nN; i++) {
		std::getline(file, line);
		std::stringstream lineStreamN(line);
		// Parse the way-point from the line
		int n;
		lineStreamN >> n;
		totalPath.push_back(n-1);
		if(TCUT_LKH_DEBUG)
			printf(" %d", n-1);
	}
	if(TCUT_LKH_DEBUG)
		printf("\n");

	file.close();

	// Debug and tree printing
	if(PRINT_LKH_PATH) {
		FILE * pForestFile;
		pForestFile = fopen("forest_output.txt", "w");

		std::list<int>::iterator prev = totalPath.begin();
		std::list<int>::iterator next = totalPath.begin();
		next++;

		while(next != totalPath.end()) {
			fprintf(pForestFile, "%d %d\n",  *prev, *next);
			prev = next;
			next++;
		}
		fprintf(pForestFile, "%d %d\n",  *prev, totalPath.front());

		fclose(pForestFile);
	}

	/// Correct the returned list from the LKH solver
	// Check for weird (easy) edge-cases
	if((totalPath.front() == closestToDepot) && (totalPath.back() == closestToIS)) {
		// Nothing to fix...
	}
	else if((totalPath.front() == closestToIS) && (totalPath.back() == closestToDepot)) {
		// Easy fix, just reverse the list
		totalPath.reverse();
	}
	else {
		// Correcting the path will take a little more work...
		// Scan totalPath to determine the given order
		bool reverseList = true;
		{
			std::list<int>::iterator it = totalPath.begin();

			while((*it != closestToDepot) && (it != totalPath.end())) {
				if(*it == closestToIS) {
					reverseList = false;
				}
				it++;
			}
		}

		// Rotate list so that it starts at the closest-to-depot way-point,
		//  and ends with the closest-to-ideal-stop
		bool rotate_again = true;
		while(rotate_again) {
			if(totalPath.front() == closestToDepot) {
				// Total path has been corrected
				rotate_again = false;
			}
			else {
				// Keep rotating list
				int temp = totalPath.front();
				totalPath.pop_front();
				totalPath.push_back(temp);
			}
		}

		if(reverseList) {
			// We were given the list "backwards", we need to reverse it
			int temp = totalPath.front();
			totalPath.pop_front();
			totalPath.push_back(temp);

			totalPath.reverse();
		}
	}

	// Verify that the list is correct
	if((totalPath.front() != closestToDepot) || (totalPath.back() != closestToIS)) {
		// Something went wrong...
		fprintf(stderr, "[ERROR] : Tree_Cutting_LKH::RunAlgorithm : totalPath order is not as expected\n");
		for(int n : totalPath) {
			printf(" %d", n);
		}
		printf("\n");
		exit(1);
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
	std::list<Vertex*> lstDynamMemVertices;
	do{
		if(SANITY_PRINT)
			printf("Cut tree into m = %d parts\n", m);
		increate_m = false;

		// Each subtour should be 1/m in length
		double one_mth_dist = totalPath_length/m;
		// TODO: Add check to jump to next loop iteration if one_mth_dist > max-UAV distance
		double residual = 0;
		// Array of sub-tours
		std::vector<std::list<Vertex*>> tours;
		// Array of sub-tour speeds
		std::vector<float> speeds;

		// Sanity print
		if(TCUT_LKH_DEBUG)
			printf("Build M = %d subtours, each %f long\n", m, one_mth_dist);

		std::list<int>::iterator prev = totalPath.begin();
		std::list<int>::iterator next = totalPath.begin();
		do {
			// Start new tour
			std::list<Vertex*> subtour;
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
		if(TCUT_LKH_DEBUG) {
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
					// Determine the actual terminal position
					bool run_again = true;
					// Run until distance stops changing
					while(run_again) {
						// Get the "old" sub-tour distance
						float old_dist = getSubtourDist(tours.at(i));
						if(old_dist > DIST_MAX) {
							run_again = false;
							increate_m = true;
						}
						else {
							// Determine velocity for old distance
							float fV_u = solution->GetMaxVelocity(old_dist);
							// Determine terminal location using found velocity
							determineTerminalLocation(tours.at(i), solution->m_tBSTrajectory.mX, fV_u, tempV);
							// Determine new sub-tour distance
							float new_dist = getSubtourDist(tours.at(i));

							if(TCUT_LKH_DEBUG)
								printf(" old_dist: %f, velocity: %f, new_dist: %f, difference: %f\n", old_dist,
										fV_u, new_dist, abs(old_dist - new_dist));

							// Determine if new distance is reasonably close to old distance
							if(new_dist > DIST_MAX) {
								run_again = false;
								increate_m = true;
							}
							else if(abs(old_dist - new_dist) <= DIST_TOLERANCE) {
								// Close enough, continue algorithm
								v = fV_u;
								run_again = false;
							}
						}
					}

					if(TCUT_LKH_DEBUG)
						printf(" * go %f m/s\n", v);
				}
				else {
					increate_m = true;
				}
			}
			else if (sub_dist <= DIST_OPT) {
				// Sanity print
				if(TCUT_LKH_DEBUG) {
					printf("New sub-tour, size = %ld\n", tours.at(i).size());
					for(Vertex* n : tours.at(i)) {
						printf(" %d: (%f, %f)", n->nID, n->fX, n->fY);
					}
					printf("\n * distance = %f\n", getSubtourDist(tours.at(i)));
				}
			}
			else {
				if(TCUT_LKH_DEBUG)
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
}


//***********************************************************
// Private Member Functions
//***********************************************************
