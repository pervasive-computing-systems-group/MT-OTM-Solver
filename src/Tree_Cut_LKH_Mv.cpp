#include "Tree_Cut_LKH_Mv.h"

Tree_Cut_LKH_Mv::Tree_Cut_LKH_Mv() {
}

Tree_Cut_LKH_Mv::~Tree_Cut_LKH_Mv() {
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
void Tree_Cut_LKH_Mv::RunAlgorithm(Solution* solution) {
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
	if(TC_LKH_MV_DEBUG) {
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
	if(TC_LKH_MV_DEBUG) {
		printf("Found closest to depot: %d, with %f m\n", closestToDepot, bestDistToDepot);
		printf("Found closest to ideal-stop: %d, with %f m\n", closestToIS, bestDistToIS);
	}

	/// Find fixed-Hamiltonian path from closestToDepot to closestToIS
	solution->PrintLKHDataFHPP(closestToDepot, closestToIS);

	if(TC_LKH_MV_DEBUG)
		printf("Running LKH\n");
	std::system("LKH FixedHPP.par");

	if(TC_LKH_MV_DEBUG)
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
		if(TC_LKH_MV_DEBUG)
			printf(" %d", n-1);
	}
	if(TC_LKH_MV_DEBUG)
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
	do {
		if(SANITY_PRINT)
			printf("\nCut tree into m = %d parts\n", m);
		increate_m = false;
		double last_term_time = 0;

		// Each subtour should be 1/m in length
		double one_mth_dist = totalPath_length/m;
		// Can we actually do this...
		if(one_mth_dist > DIST_MAX) {
			// Need to increase m...
			increate_m = true;
			// Sanity print
			if(TC_LKH_MV_DEBUG)
				printf("M = %d too small! min sub-tour-dist = %f > %f = max-dist\n", m, one_mth_dist, DIST_MAX);
		}
		else {
			/// Start cutting the tree!
			double residual = 0;
			// Array of sub-tours
			std::vector<std::list<Vertex*>> tours;
			// Array of sub-tour speeds
			std::vector<float> speeds;

			// Sanity print
			if(TC_LKH_MV_DEBUG)
				printf("Build M = %d subtours, each %.3f m long\n", m, one_mth_dist);

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
			if(TC_LKH_MV_DEBUG) {
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
				double sub_tour_start = 0;
				if(i == 0) {
					// Create a new depot
					Vertex* depot = new Vertex;
					lstDynamMemVertices.push_back(depot);

					// Get closest point to first stop on tour
					Vertex* first_stop = tours.at(i).front();
					double x_fs, y_fs;
					solution->m_tBSTrajectory.ClostestPoint(first_stop->fX, first_stop->fY, &x_fs, &y_fs);
					double first_stop_time = solution->m_tBSTrajectory.getTimeAt(x_fs, y_fs);

					// Either start at time 0 or start at the closest point to the begging of the subtour
					sub_tour_start = std::min(0.0, first_stop_time);

					double x_u, y_u;
					// Start sub-tour at min-start position
					solution->m_tBSTrajectory.getPosition(sub_tour_start, &x_u, &y_u);
					depot->fX = x_u;
					depot->fY = y_u;
					depot->eVType = E_VertexType::e_Depot;

					// Add this new depot to the tour
					tours.at(i).push_front(depot);

					// Sanity print
					if(TC_LKH_MV_DEBUG)
						printf(" Added first depot at (%.3f, %.3f)\n", x_u, y_u);
				}
				else {
					// Create a new depot
					Vertex* depot = new Vertex;
					lstDynamMemVertices.push_back(depot);
					Vertex* last_term = tours.at(i-1).back();

					double last_term_time = solution->m_tBSTrajectory.getTimeAt(last_term->fX, last_term->fY);
					double min_start_time = last_term_time + BATTERY_SWAP_TIME;

					// Get closest point to first stop on tour
					Vertex* first_stop = tours.at(i).front();
					double x_fs, y_fs;
					solution->m_tBSTrajectory.ClostestPoint(first_stop->fX, first_stop->fY, &x_fs, &y_fs);
					double closest_time = solution->m_tBSTrajectory.getTimeAt(x_fs, y_fs);

					// Either start after battery swap or at closest point (at max)
					sub_tour_start = std::max(min_start_time, closest_time);

					// Determine time at depot
					double x_u, y_u;
					solution->m_tBSTrajectory.getPosition(sub_tour_start, &x_u, &y_u);
					depot->fX = x_u;
					depot->fY = y_u;
					depot->eVType = E_VertexType::e_Depot;

					// Add this new depot to the tour
					tours.at(i).push_front(depot);

					// Sanity print
					if(TC_LKH_MV_DEBUG)
						printf(" Added next depot at (%.3f, %.3f)\n", x_u, y_u);
				}

				// Create a terminal for this subtour
				Vertex* tempV = new Vertex;
				// Place terminal at closest point on UGV path
				double x_v, y_v;
				solution->m_tBSTrajectory.ClostestPoint(tours.at(i).back()->fX, tours.at(i).back()->fY, &x_v, &y_v);
				tempV->fX = x_v;
				tempV->fY = y_v;
				tempV->eVType = E_VertexType::e_Terminal;
				lstDynamMemVertices.push_back(tempV);

				// Sanity print
				if(TC_LKH_MV_DEBUG)
					printf(" Added terminal, initial guess = (%.3f, %.3f)\n", x_v, y_v);

				// Add terminal to sub-tour
				tours.at(i).push_back(tempV);

				/// Using time-space, predict terminal location (iteratively)
				// Estimate time of subtour
				double new_time_est = 0;
				double old_time_est = 0;
				bool predict_time_again = true;

				// Iteratively update our prediction
				while(predict_time_again) {
					// How far is the sub-tour?
					double tour_dist = 0;
					{
						Vertex* previous = tours.at(i).front();
						for(Vertex* v : tours.at(i)) {
							tour_dist += previous->GetDistanceTo(v);
							previous = v;
						}
					}

					// Verify that this tour isn't too long...
					double fV_u = V_MAX;
					if(tour_dist <= DIST_MAX) {
						// Determine the time to run this distance
						fV_u = solution->GetMaxVelocity(tour_dist);
					}
					else {
						// Sub-tour distance is too long...
						increate_m = true;
						predict_time_again = false;
					}
					new_time_est = tour_dist/fV_u;

					// Update the terminal's position
					solution->m_tBSTrajectory.getPosition(sub_tour_start + new_time_est, &x_v, &y_v);
					tempV->fX = x_v;
					tempV->fY = y_v;

					last_term_time = solution->m_tBSTrajectory.getTimeAt(x_v, y_v);

					// Sanity print
					if(TC_LKH_MV_DEBUG)
						printf("  Updated to (%.3f, %.3f) w/ time = %.3f\n", x_v, y_v, new_time_est);

					// Did we change time?
					if(equalFloats(new_time_est, old_time_est)) {
						// Found something consistent... stop looping
						predict_time_again = false;
						speeds.push_back(fV_u);
					}
					old_time_est = new_time_est;
				}
			}

			if(!increate_m) {
				// Check to see if we improved the solution
				if(best_total_time > last_term_time) {
					// Sanity print
					if(SANITY_PRINT)
						printf("* Found better solution!\n* Total time: %f\n", last_term_time);

					// Give new tour-set to solution
					solution->BuildCompleteSolution(tours, speeds);

					// Increase m in an attempt to further improve the solution
					increate_m = true;
					best_total_time = solution->m_Tt_k.back();
				}
				else {
					// This value of m didn't improve the solution, stop algorithm
					if(SANITY_PRINT)
						printf("* Total time: %f, Tour didn't improve, stop algorithm\n", last_term_time);
					increate_m = false;
				}
			}
			else {
				if(SANITY_PRINT)
					printf("* Tour unsatisfactory, increase m\n");
			}
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
