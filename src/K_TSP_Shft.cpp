#include "K_TSP_Shft.h"

K_TSP_Shft::K_TSP_Shft() {
	srand(time(NULL));
//	srand(1);
}

K_TSP_Shft::~K_TSP_Shft() {
}

//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Runs the K-means -> TSP -> Adjusting algorithm
 */
void K_TSP_Shft::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\nRunning K_TSP_Shft with n = %d, m = %d\n\n", solution->m_nN, solution->m_nM);

	/// Run k-means clustering to group together waypoints
	// Add waypoints to a vector
	std::vector<KMPoint> points;
	std::vector<KMPoint> centroids;


	{
		double min_x = std::numeric_limits<double>::max();
		double max_x = 0;
		double min_y = std::numeric_limits<double>::max();
		double max_y = 0;
		for(int i = 0; i < solution->m_nN; i++) {
			KMPoint pnt(solution->m_pVertexData+i);
			points.push_back(pnt);

			// Check min/max x
			if(solution->m_pVertexData[i].fX < min_x) {
				min_x = solution->m_pVertexData[i].fX;
			}
			if(solution->m_pVertexData[i].fX > max_x) {
				max_x = solution->m_pVertexData[i].fX;
			}
			// Check min/max y
			if(solution->m_pVertexData[i].fY < min_y) {
				min_y = solution->m_pVertexData[i].fY;
			}
			if(solution->m_pVertexData[i].fY > max_y) {
				max_y = solution->m_pVertexData[i].fY;
			}
		}

		// Set centroid locations
		for(int k = 0; k < solution->m_nM; k++) {
			// Pick equal points
			double step_x = (max_x - min_x)/solution->m_nM;
			double step_y = (max_y - min_y)/solution->m_nM;
			// Set centroid at this point
			KMPoint pnt(min_x + step_x*k, min_y + step_y*k);
			pnt.vID = k;

//			// Pick a random waypoint
//			int rand_wp = rand() % solution->m_nN;
//			// Set the details of this waypoint as the centroid
//			KMPoint pnt(solution->m_pVertexData[rand_wp].fX, solution->m_pVertexData[rand_wp].fY);
//			pnt.vID = k;

			// Add centroid to centroid list
			centroids.push_back(pnt);
		}

	}

	// Sanity print
	if(DEBUG_K_TSP_SHFT) {
		printf("Initial Centroids:\n");
		for(KMPoint cnt : centroids) {
			printf(" k=%d, (%.3f,%.3f)\n", cnt.vID, cnt.x, cnt.y);
		}
	}

	// Run k-means clustering with some fixed number of iterations
	kMeansClustering(&points, &centroids, 1000*solution->m_nN);

	// Sanity print
	if(DEBUG_K_TSP_SHFT) {
		printf("Cluster assignments:\n");
		for(KMPoint pnt : points) {
			printf("%d:%d, ", pnt.vID, pnt.cluster);
		}
		printf("\nCentroids:\n");
		for(KMPoint cnt : centroids) {
			printf(" k=%d, (%.3f,%.3f) ", cnt.vID, cnt.x, cnt.y);
		}
		printf("\n");
	}

	/// Set start/stop locations for each subtour by finding min-spanning tree on each subtour
	std::vector<double> cluster_dist;
	std::vector<double> cp_x;
	std::vector<double> cp_y;

	// For each cluster
	for(int k = 0; k < solution->m_nM; k++) {
		// Store pointers to the vertices in this cluster
		std::vector<Vertex*> cluster;
		for(KMPoint pnt : points) {
			if(pnt.cluster == k) {
				// Add vertex to the cluster
				cluster.push_back(solution->m_pVertexData + pnt.vID);
			}
		}

		// Sanity print
		if(DEBUG_K_TSP_SHFT) {
			printf("Cluster %d:\n {", k);
			for(Vertex* v : cluster) {
				printf(" %d", v->nID);
			}
			printf("}\n");
		}

		// Find point along UGV path closest to centroid
		double cent_x, cent_y;
		solution->m_tBSTrajectory.ClostestPoint(centroids.at(k).x, centroids.at(k).y, &cent_x, &cent_y);
		Vertex ugv(-1, cent_x, cent_y, E_VertexType::e_Depot);
		cluster.push_back(&ugv);
		cp_x.push_back(cent_x);
		cp_y.push_back(cent_y);

		double dist = Graph_Theory_Algorithms::MST_Prims(cluster);
		cluster_dist.push_back(dist);

		if(DEBUG_K_TSP_SHFT)
			printf(" dist = %f\n", dist);
	}

	// We need to order the clusters
	std::vector<int> clust_to_part;
	// Find cluster-to-partition mapping
	{
		// Make an array that tracks which clusters we have already assigned
		bool* cluster_marked = new bool[solution->m_nM];
		for(int i = 0; i < solution->m_nM; i++) {
			cluster_marked[i] = false;
		}

		if(DEBUG_K_TSP_SHFT)
			printf("Ordering cluster\n");
		// For each partition
		for(int i = 0; i < solution->m_nM; i++) {
			if(DEBUG_K_TSP_SHFT)
				printf(" partition %d\n", i);
			double next_closest_dist = 100000000000.0;
			int next_cluster = -1;
			// For each cluster
			for(int k = 0; k < solution->m_nM; k++) {
				if(DEBUG_K_TSP_SHFT)
					printf("  checking cluster %d (%.3f, %.3f)\n", k, cp_x.at(k), cp_y.at(k));
				// ASSUMPTION! order can be determined based on x position (least-first) with y position to break ties (least first)
				// ``sort'' by above condition
				if((cp_x.at(k) < next_closest_dist) && !cluster_marked[k]) {
					if(DEBUG_K_TSP_SHFT)
						printf("   update to this cluster\n");

					// New next-best
					next_cluster = k;
					next_closest_dist = cp_x.at(k);
				}
			}

			// Update what we found
			if(next_cluster != -1) {
				clust_to_part.push_back(next_cluster);
				cluster_marked[next_cluster] = true;
			}
			else {
				// Something went wrong... (two equal x values?)
				fprintf(stderr, "[ERROR] : K_TSP_Shft::RunAlgorithm() : Ordering clusters... next_cluster returned -1\n");
				exit(1);
			}
		}

		delete[] cluster_marked;
	}

	// Sanity print
	if(DEBUG_K_TSP_SHFT) {
		printf("Cluster-to-Partition:\n");
		for(int i = 0; i < clust_to_part.size(); i++) {
			printf(" %d:%d", i, clust_to_part.at(i));
		}
		printf("\nEstimated time-per-cluster:\n");
	}

	// Track the start times between iterations
	std::vector<double> previous_start_times;

	// Estimate the time to run each cluster
	for(int k = 0; k < solution->m_nM; k++) {
		// Determine the time of each centroid
		double cent_time = solution->m_tBSTrajectory.getTimeAt(cp_x.at(k), cp_y.at(k));
		// Assume that the UAV goes at max speed... what is the lower bound on the run time?
		double min_time = cluster_dist.at(k)/V_MAX;
		// Note start time
		double start_time = cent_time - min_time/2.0;
//		// Can't start earlier than 0
//		start_time = std::max(start_time, 0.0);

		// Place the start/end locations of the subtour at the central point +- 1/2 min time
		double depot_x, depot_y;
		solution->m_tBSTrajectory.getPosition(start_time, &depot_x, &depot_y);
		Vertex* depot_k = solution->GetDepotOfPartion(clust_to_part.at(k));
		depot_k->fX = depot_x;
		depot_k->fY = depot_y;

		double term_x, term_y;
		solution->m_tBSTrajectory.getPosition(start_time + min_time, &term_x, &term_y);
		Vertex* term_k = solution->GetTerminalOfPartion(clust_to_part.at(k));
		term_k->fX = term_x;
		term_k->fY = term_y;

		if(DEBUG_K_TSP_SHFT)
			printf(" %d: %.3f - start @ %.3f\n", k, min_time, start_time);

		// Record predicted start time
		previous_start_times.push_back(start_time);
	}

	// Sanity print...
//	solution->PrintGraph();
	if(DEBUG_K_TSP_SHFT)
		printf("\n** Solve TSP **\n");

	/// Run TSP on resulting fixed-HPP
	std::vector<std::vector<Vertex*>> previous_tours;
	// For each cluster
	for(int k = 0; k < solution->m_nM; k++) {
		// Store pointers to the vertices in this cluster
		std::vector<Vertex*> cluster;
		for(KMPoint pnt : points) {
			if(pnt.cluster == clust_to_part.at(k)) {
				// Add vertex to the cluster
				cluster.push_back(solution->m_pVertexData + pnt.vID);
			}
		}
		// Add in the depot/terminal
		cluster.push_back(solution->GetDepotOfPartion(k));
		cluster.push_back(solution->GetTerminalOfPartion(k));

		// Sanity print
		if(DEBUG_K_TSP_SHFT) {
			printf("Sub-tour %d:\n", k);
			for(Vertex* v : cluster) {
				printf(" %d", v->nID);
			}
			printf("\n");
		}

		std::vector<Vertex*> sub_tour;

		runLKH_TSP(solution, &cluster, &sub_tour);

		// Add this sub-tour to the solution
		previous_tours.push_back(sub_tour);
	}

	// Sanity print
	if(DEBUG_K_TSP_SHFT) {
		printf("Tours:\n");
		for(std::vector<Vertex*> sub_tour : previous_tours) {
			printf(" {");
			for(Vertex* v : sub_tour) {
				printf(" %d", v->nID);
			}
			printf("}\n");
		}
	}

	// Determine which partition-to-cluster mapping
	std::vector<int> part_to_cluster;
	for(int k = 0; k < solution->m_nM; k++) {
		for(int n = 0; n < solution->m_nM; n++) {
			if(k == clust_to_part.at(n)) {
				// Found opposite matching
				part_to_cluster.push_back(n);
				break;
			}
		}
	}

	// Sanity print
	if(DEBUG_K_TSP_SHFT) {
		printf("Partition-to-cluster mapping:\n");
		for(int k = 0; k < part_to_cluster.size(); k++) {
			printf(" %d:%d\n", k, part_to_cluster.at(k));
		}
	}

	/// Adjust start/stop locations until solution is consistent (iteratively)
	bool changed_times = true;
	int inner_counter = 0;
	bool tour_changed = true;
	int outter_counter = 0;
	std::vector<double> tour_duration;
	std::vector<double> tour_distance;

	// While still making changes to the sub-tours...
	while(tour_changed && outter_counter < 5) {
		tour_changed = false;
		outter_counter++;
		inner_counter = 0;
		// While still making changes to time...
		while(changed_times && inner_counter < 5) {
			// Assume we don't restart...
			changed_times = false;
			inner_counter++;

			/// Correct tour times based on found solution
			tour_duration.clear();
			tour_distance.clear();

			// Sanity print
			if(DEBUG_K_TSP_SHFT)
				printf("Correcting start/end times to be consistent with time\n");

			double const delta_t = 10;
			double earliest_start = 0;

			// For each sub-tour
			for(int k = 0; k < solution->m_nM; k++) {
				// Initial desired start time based current depot location
				double start_time = solution->m_tBSTrajectory.getTimeAt(solution->GetDepotOfPartion(k)->fX,
						solution->GetTerminalOfPartion(k)->fY);
				if(DEBUG_K_TSP_SHFT)
					printf(" Tour %d - starting @ %.3f\n", k, start_time);
				double previous_time = 0;
				double new_time = 0;
				bool run_again = true;

				// While we keep making updates...
				while(run_again) {
					// Determine the distance of the tour
					double dist = 0;
					Vertex* previous = previous_tours.at(k).front();
					for(Vertex* v : previous_tours.at(k)) {
						 dist += previous->GetDistanceTo(v);
						 previous = v;
					}

					// Sanity print
					if(DEBUG_K_TSP_SHFT)
						printf("  dist = %.3f\n", dist);

					// Determine the time to move this distance
					if(dist > DIST_MAX) {
						// This is too long to fly!
						new_time = std::numeric_limits<float>::max()-1;
						if(DEBUG_K_TSP_SHFT)
							printf("   too far... t = %f\n", new_time);

						// Mark this solution at infeasible
						solution->m_bFeasible = false;

						// Just return to avoid breaking things...
						return;
					}
					else if(dist <= DIST_OPT) {
						// Fly at max speed
						new_time = dist * (1.0/V_MAX);
						if(DEBUG_K_TSP_SHFT)
							printf("   go V_MAX = %f, t = %f\n", V_MAX, new_time);
					}
					else {
						// Determine fastest speed to move through this leg
						double v = solution->GetMaxVelocity(dist);

						new_time = dist/v;

						if(DEBUG_K_TSP_SHFT)
							printf("   go v = %f, t = %f\n", v, new_time);
					}

					// Adjust the depot/terminal based on found time
					Vertex* depot_k = solution->GetDepotOfPartion(k);
					Vertex* term_k = solution->GetTerminalOfPartion(k);
					// Determine "mid-time" of tour
					double old_start_time = solution->m_tBSTrajectory.getTimeAt(depot_k->fX, depot_k->fY);
					double old_end_time = solution->m_tBSTrajectory.getTimeAt(term_k->fX, term_k->fY);
					double old_mid_time = (old_end_time - old_start_time)/2 + old_start_time;

					// Place the start/end locations of the subtour at mid-time +- 1/2 new-time
					double depot_x, depot_y;
					solution->m_tBSTrajectory.getPosition(old_mid_time - new_time/2.0, &depot_x, &depot_y);
					depot_k->fX = depot_x;
					depot_k->fY = depot_y;
					double term_x, term_y;
					solution->m_tBSTrajectory.getPosition(old_mid_time + new_time/2, &term_x, &term_y);
					term_k->fX = term_x;
					term_k->fY = term_y;

					// Record the start time
					previous_start_times.at(k) = solution->m_tBSTrajectory.getTimeAt(depot_k->fX, depot_k->fY);

					// Sanity print
					if(DEBUG_K_TSP_SHFT)
						printf("  previous time = %.3f, new time = %.3f\n", previous_time, new_time);

					// While time changed, repeat!
					if(equalFloats(previous_time, new_time)) {
						// Can we push this back?
						double new_start_time = previous_start_times.at(k) - delta_t;
						if(dist <= DIST_OPT && new_start_time > earliest_start) { // Push start time back...
							// Place the start/end locations of the subtour at mid-time +- 1/2 new-time
							double depot_x, depot_y;
							solution->m_tBSTrajectory.getPosition(new_start_time, &depot_x, &depot_y);
							depot_k->fX = depot_x;
							depot_k->fY = depot_y;
							double term_x, term_y;
							solution->m_tBSTrajectory.getPosition(new_start_time + new_time, &term_x, &term_y);
							term_k->fX = term_x;
							term_k->fY = term_y;

							// Sanity print
							if(DEBUG_K_TSP_SHFT)
								printf("   Push-back 1: start from %.3f to %.3f\n", previous_start_times.at(k), new_start_time);
						}
						else {
							// Sanity print
							if(DEBUG_K_TSP_SHFT)
								printf("   * Consistent!\n");
							tour_duration.push_back(new_time);
							tour_distance.push_back(dist);
							earliest_start = new_start_time + new_time + BATTERY_SWAP_TIME;

							run_again = false;
						}
					}
					else {
						// Sanity print
						if(DEBUG_K_TSP_SHFT)
							printf("   Run update again...\n");
					}
					// Update previous before next run
					previous_time = new_time;
				}
			}

			/// Adjust start times based on QP
			std::vector<double> desired_times;
			std::vector<double> selected_times;
			// Determine start times
			for(int k = 0; k < solution->m_nM; k++) {
				// Initial desired start time based current depot location
				double desired_start_time = solution->m_tBSTrajectory.getTimeAt(solution->GetDepotOfPartion(k)->fX,
						solution->GetTerminalOfPartion(k)->fY);
				desired_times.push_back(desired_start_time);
			}

			// Run QP to get optimal start times
			solveStartTimesQP(&tour_duration, &desired_times, &selected_times);

			// Sanity print
			if(DEBUG_K_TSP_SHFT)
				printf("Adjusting correcting start/stop based on QP\n");

			// Update depot/terminals
			for(int k = 0; k < solution->m_nM; k++) {
				Vertex* depot_k = solution->GetDepotOfPartion(k);
				Vertex* term_k = solution->GetTerminalOfPartion(k);
				// Determine "mid-time" of tour
				double old_start_time = solution->m_tBSTrajectory.getTimeAt(depot_k->fX, depot_k->fY);
				double old_end_time = solution->m_tBSTrajectory.getTimeAt(term_k->fX, term_k->fY);
				double old_mid_time = (old_end_time - old_start_time)/2 + old_start_time;

				// Place the start/end locations of the subtour at mid-time +- 1/2 new-time
				double depot_x, depot_y;
				solution->m_tBSTrajectory.getPosition(selected_times[k], &depot_x, &depot_y);
				depot_k->fX = depot_x;
				depot_k->fY = depot_y;
				double term_x, term_y;
				solution->m_tBSTrajectory.getPosition(selected_times[k] + tour_duration[k], &term_x, &term_y);
				term_k->fX = term_x;
				term_k->fY = term_y;

				// Sanity print
				if(DEBUG_K_TSP_SHFT)
					printf(" %d: depot-(%.3f, %.3f), terminal-(%.3f, %.3f)\n", k, depot_x, depot_y, term_x, term_y);
			}

			// Sanity print
			if(DEBUG_K_TSP_SHFT)
				printf("Comparing start times\n");

			// Determine if we updated our start times...
			for(int k = 0; k < solution->m_nM; k++) {
				if(!equalFloats(selected_times.at(k), previous_start_times.at(k))) {
					// Sanity print
					if(DEBUG_K_TSP_SHFT)
						printf(" %d: previous=%.3f, new=%.3f\n", k, selected_times.at(k), previous_start_times.at(k));
					changed_times = true;
				}
				previous_start_times.at(k) = selected_times.at(k);
			}

			// Sanity print
			if(DEBUG_K_TSP_SHFT)
				printf("\n** Iteration %d, Changed times? %d**\n\n", inner_counter, changed_times);
		}




		/// Verify that we ended with something that is consistent
		if(changed_times) {
			if(DEBUG_K_TSP_SHFT)
				printf("Loop timed-out, fixing sub tours\n");

			// Make sure each tour is at least consistent
			double earliest_start = 0;
			tour_duration.clear();

			for(int k = 0; k < solution->m_nM; k++) {
				if(DEBUG_K_TSP_SHFT)
					printf(" Tour %d\n", k);

				// Verify that we aren't starting too early
				Vertex* depot = previous_tours.at(k).front();
				double start_time = solution->m_tBSTrajectory.getTimeAt(depot->fX, depot->fY);
				if(start_time < earliest_start) {
					// Fix this start time
					if(DEBUG_K_TSP_SHFT)
						printf("  Fixing start time: %.3f to %.3f\n", start_time, earliest_start);
					double x,y;
					solution->m_tBSTrajectory.getPosition(earliest_start, &x, &y);
					depot->fX = x;
					depot->fY = y;
					start_time = earliest_start;
				}

				// Current terminal position
				double previous_time = 0;
				double new_time = 0;
				bool run_again = true;
				// While we keep making updates...
				while(run_again) {
					// Determine the distance of the tour
					double dist = 0;
					Vertex* previous = previous_tours.at(k).front();
					for(Vertex* v : previous_tours.at(k)) {
						 dist += previous->GetDistanceTo(v);
						 previous = v;
					}

					// Sanity print
					if(DEBUG_K_TSP_SHFT)
						printf("  dist = %.3f\n", dist);

					// Determine the time to move this distance
					if(dist > DIST_MAX) {
						// This is too long to fly!
						if(DEBUG_K_TSP_SHFT)
							printf("   too far... t = %f\n", new_time);

						// Mark this solution at infeasible
						solution->m_bFeasible = false;

						// Just return to avoid breaking things...
						return;
					}
					else if(dist <= DIST_OPT) {
						// Fly at max speed
						new_time = dist * (1.0/V_MAX);
						if(DEBUG_K_TSP_SHFT)
							printf("   go V_MAX = %f, t = %f\n", V_MAX, new_time);
					}
					else {
						// Determine fastest speed to move through this leg
						double v = solution->GetMaxVelocity(dist);

						new_time = dist/v;

						if(DEBUG_K_TSP_SHFT)
							printf("   go v = %f, t = %f\n", v, new_time);
					}

					// Adjust the depot/terminal based on found time
					Vertex* term_k = solution->GetTerminalOfPartion(k);
					double term_x, term_y;
					solution->m_tBSTrajectory.getPosition(start_time + new_time, &term_x, &term_y);
					term_k->fX = term_x;
					term_k->fY = term_y;

					// Record the start time
					previous_start_times.at(k) = start_time;

					// Sanity print
					if(DEBUG_K_TSP_SHFT)
						printf("  previous time = %.3f, new time = %.3f\n", previous_time, new_time);

					// While time changed, repeat!
					if(equalFloats(previous_time, new_time)) {
						tour_duration.push_back(new_time);
						run_again = false;
						earliest_start = start_time + new_time + BATTERY_SWAP_TIME;

						// Sanity print
						if(DEBUG_K_TSP_SHFT)
							printf("   * Consistent! Earliest start = %.3f\n", earliest_start);
					}
					else {
						// Sanity print
						if(DEBUG_K_TSP_SHFT)
							printf("   Run fixer again...\n");
					}
					// Update previous before next run
					previous_time = new_time;
				}
			}
		}





		// Sanity print...
//		solution->PrintGraph();
		if(DEBUG_K_TSP_SHFT)
			printf("\n** Solve TSP **\n");

		/// Run TSP on resulting fixed-HPP
		std::vector<std::vector<Vertex*>> new_tours;
		// For each cluster
		for(int k = 0; k < solution->m_nM; k++) {
			// Store pointers to the vertices in this cluster (expects terminal & depot last)
			std::vector<Vertex*> cluster;
			for(int i = 1; i < previous_tours.at(k).size()-1; i++) {
				cluster.push_back(previous_tours.at(k).at(i));
			}
			// Add in the depot/terminal
			cluster.push_back(previous_tours.at(k).at(0));
			cluster.push_back(previous_tours.at(k).back());

			// Sanity print
			if(DEBUG_K_TSP_SHFT) {
				printf("Sub-tour %d:\n", k);
				for(Vertex* v : cluster) {
					printf(" %d", v->nID);
				}
				printf("\n");
			}

			std::vector<Vertex*> sub_tour;

			runLKH_TSP(solution, &cluster, &sub_tour);

			// Add this sub-tour to the solution
			new_tours.push_back(sub_tour);
		}

		// Sanity print
		if(DEBUG_K_TSP_SHFT) {
			printf("Previous tours\n");
			for(std::vector<Vertex*> sub_tour : previous_tours) {
				printf(" {");
				for(Vertex* v : sub_tour) {
					printf(" %d", v->nID);
				}
				printf("}\n");
			}
			printf("New tours\n");
			for(std::vector<Vertex*> sub_tour : new_tours) {
				printf(" {");
				for(Vertex* v : sub_tour) {
					printf(" %d", v->nID);
				}
				printf("}\n");
			}
		}

		// Did something change?
		for(int k = 0; k < solution->m_nM; k++) {
			// Check the order of vertices in this sub-tour
			for(int i = 0; i < previous_tours.at(k).size(); i++) {
				if(previous_tours.at(k).at(i)->nID != new_tours.at(k).at(i)->nID) {
					tour_changed = true;
					break;
				}
			}
			if(tour_changed) {
				break;
			}
		}

		// Update...
		for(int k = 0; k < solution->m_nM; k++) {
			previous_tours.at(k).clear();

			for(Vertex* v : new_tours.at(k)) {
				previous_tours.at(k).push_back(v);
			}
		}

		// Sanity print
		if(DEBUG_K_TSP_SHFT) {
			printf("Updated Previous tours\n");
			for(std::vector<Vertex*> sub_tour : previous_tours) {
				printf(" {");
				for(Vertex* v : sub_tour) {
					printf(" %d", v->nID);
				}
				printf("}\n");
			}

			// Sanity print
			printf("\n** Outter-Iteration %d, Changed sub-tours? %d**\n\n", outter_counter, tour_changed);
		}
	}

	/// Store final solution
	solution->m_Td_k.clear();
	solution->m_Tt_k.clear();

	for(int k = 0; k < solution->m_nM; k++) {
		solution->m_Td_k.push_back(previous_start_times.at(k));
		solution->m_Tt_k.push_back(previous_start_times.at(k) + tour_duration.at(k));
		Vertex* previous = previous_tours.at(k).front();
		for(int i = 1; i < previous_tours.at(k).size(); i++) {
			// Added edge to solution
			int a = previous->nID;
			int b = previous_tours.at(k).at(i)->nID;
			if(a < b) {
				solution->m_pAdjMatrix[a][b] = true;
			}
			else {
				solution->m_pAdjMatrix[b][a] = true;
			}

			previous = previous_tours.at(k).at(i);
		}
	}

	// Memory cleanup
}

//***********************************************************
// Private Member Functions
//***********************************************************

/*
 * Performs Lloyd's k-means clustering algorithm on points using centroids as the initial centroids. The
 * algorithm runs for epocks iterations.
 */

void K_TSP_Shft::kMeansClustering(std::vector<KMPoint>* points, std::vector<KMPoint>* centroids, int epochs) {
	// The index of the centroid within the centroids vector
	int* nPoints = new int[(int)centroids->size()];
	double* sumX = new double[(int)centroids->size()];
	double* sumY = new double[(int)centroids->size()];

	bool run_again = true;
	int iteration = 0;

	// Sanity print
//	printf("Starting k-means clustering\n");

	// Run clustering iterations
	while(run_again && iteration < epochs) {
		run_again = false;
		iteration++;

		// Sanity print
//		printf("Iteration: %d\n", iteration);

		// Check every edge against every centroid
		for(int p = 0; p < points->size(); p++) {
			// For each centroid, compute distance from the centroid to the point
			// and update the point's cluster if necessary
			double min_dist = __DBL_MAX__;
			int closest_k = -1;
			for(KMPoint cnt : *centroids) {
				// Check to see if this centroid is better than the last centroid
				double dist = cnt.distance(points->at(p));
				if(dist < min_dist) {
					min_dist = dist;
					closest_k = cnt.vID;
				}
			}

			// Sanity print
//			printf(" node %d - k=%d w/ dist=%.3f\n", p, closest_k, min_dist);

			// Did we make an update?
			if(closest_k != points->at(p).cluster) {
				points->at(p).cluster = closest_k;
				run_again = true;
			}
		}

//		// For each cluster... measure the MST distance
//		std::vector<double> cluster_dists;
//		for(int k = 0; k < centroids->size(); k++) {
//			std::vector<Vertex*> cluster_k;
//			// For each point, check which cluster it belongs to
//			for(KMPoint pnt : *points) {
//				if(pnt.cluster == k) {
//					// Put the points of this cluster into an array of vertices
//					Vertex* v = new Vertex(-1, pnt.x, pnt.y, E_VertexType::e_Destination);
//					cluster_k.push_back(v);
//				}
//			}
//			// Run Prim's algorithm on this cluster
//			double clst_dist = Graph_Theory_Algorithms::MST_Prims(cluster_k);
//			cluster_dists.push_back(clst_dist);
//
//			// Memory cleanup
//			for(Vertex* v : cluster_k) {
//				delete v;
//			}
//		}

		// Reset helper arrays
		for(int j = 0; j < (int)centroids->size(); ++j) {
			nPoints[j]=0;
			sumX[j]=0.0;
			sumY[j]=0.0;
		}
		// Iterate over points to append data to centroids
		for(KMPoint wp : *points) {
			nPoints[wp.cluster] += 1;
			sumX[wp.cluster] += wp.x;
			sumY[wp.cluster] += wp.y;
		}
		// Verify that each centroid has at least one point
		for(int cnt = 0; cnt < centroids->size(); cnt++) {
			if(nPoints[cnt] <= 0) {
				// Does not contain a point.. give it one randomly
//				printf(" Found empty cluster! k=%d\n", cnt);
				int rand_wp = rand() % points->size();
				nPoints[cnt] += 1;
				sumX[cnt] += points->at(rand_wp).x;
				sumY[cnt] += points->at(rand_wp).y;
				run_again = true;
			}
		}
		// Compute the new centroids
//		printf(" Updating clusters\n");
		for(int cnt = 0; cnt < centroids->size(); cnt++) {
			centroids->at(cnt).x = sumX[cnt] / nPoints[cnt];
			centroids->at(cnt).y = sumY[cnt] / nPoints[cnt];
//			printf(" k=%d, (%.3f,%.3f)\n", centroids->at(cnt).vID, centroids->at(cnt).x, centroids->at(cnt).y);
		}
	}



	delete[] nPoints;
	delete[] sumX;
	delete[] sumY;
}

// Run LKH TSP solver on the give cluster of vertices
void K_TSP_Shft::runLKH_TSP(Solution* solution, std::vector<Vertex*>* cluster, std::vector<Vertex*>* sub_tour) {
	// Prep stuff
	int depot_index = cluster->size()-2;
	int terminal_index = cluster->size()-1;
	sub_tour->clear();

	// Create input file for LKH solver
	solution->PrintLKHData(*cluster);

	if(DEBUG_K_TSP_SHFT)
		printf("Running LKH\n");
	// Run TSP solver on this sub-tour
	std::system("LKH FixedHPP.par");

//	if(DEBUG_K_TSP_SHFT)
//		printf("Found the following solution:\n");

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
	for(int i = 0; i < cluster->size(); i++) {
		std::getline(file, line);
		std::stringstream lineStreamN(line);
		// Parse the way-point from the line
		int n;
		lineStreamN >> n;
		totalPath.push_back(n-1);
//		if(DEBUG_K_TSP_SHFT)
//			printf(" %d", n-1);
	}
//	if(DEBUG_K_TSP_SHFT)
//		printf("\n");

	file.close();

	/// Correct the returned list from the LKH solver
	// Check for weird (easy) edge-cases
	if((totalPath.front() == depot_index) && (totalPath.back() == terminal_index)) {
		// Nothing to fix...
	}
	else if((totalPath.front() == terminal_index) && (totalPath.back() == depot_index)) {
		// Easy fix, just reverse the list
		totalPath.reverse();
	}
	else {
		// Correcting the path will take a little more work...
		// Scan totalPath to determine the given order
		bool reverseList = true;
		{
			std::list<int>::iterator it = totalPath.begin();

			while((*it != depot_index) && (it != totalPath.end())) {
				if(*it == terminal_index) {
					reverseList = false;
				}
				it++;
			}
		}

		// Rotate list so that it starts at the closest-to-depot way-point,
		//  and ends with the closest-to-ideal-stop
		bool rotate_again = true;
		while(rotate_again) {
			if(totalPath.front() == depot_index) {
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
	if((totalPath.front() != depot_index) || (totalPath.back() != terminal_index)) {
		// Something went wrong...
		fprintf(stderr, "[ERROR] : K_TSP_Shft::RunAlgorithm() : totalPath order is not as expected\n");
		for(int n : totalPath) {
			printf(" %d", n);
		}
		printf("\n");
		exit(1);
	}

	// Sanity print
	if(DEBUG_K_TSP_SHFT) {
		printf("Fixed total path:\n");
		for(int n : totalPath) {
			printf(" %d", n);
		}
		printf("\n\n");
	}

	// Create an ordered vector based on found path
	for(int n : totalPath) {
		sub_tour->push_back(cluster->at(n));
	}
}

/*
 * Determines what time each sub-tour should begin using quadratic programming (Gurobi) - problem is a
 * sum-of-squares. Store result in selected_times, tries to keep these as close as possible to the
 * desired_times while constrained by tour_times.
 */
void K_TSP_Shft::solveStartTimesQP(std::vector<double>* tour_times, std::vector<double>* desired_times, std::vector<double>* selected_times) {
	int M = tour_times->size();

	// Sanity print
	printf("Solving QP to find optimal start/end points\n");

	// Verify tour_times and desired_times are same size
	if(desired_times->size() != M) {
		// Unbalanced input! Hard fail...
		fprintf(stderr, "[ERROR] : K_TSP_Shft::solveStartTimesQP() : Unbalanced input, M = %d while |desired_times| = %ld\n", M, desired_times->size());
		exit(1);
	}

	try {
		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		GRBVar* T = new GRBVar[M];

		// Create variables
		for(int k = 0; k < M; k++) {
			T[k] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "T_"+itos(k));
		}

		// Set objective
		GRBQuadExpr obj = 0;
		for(int k = 0; k < M; k++) {
			obj += (T[k] - desired_times->at(k))*(T[k] - desired_times->at(k));
		}

		model.setObjective(obj);

		// Add constraints
		for(int k = 1; k < M; k++) {
			GRBLinExpr cnst = 0;
			model.addConstr(T[k-1] + tour_times->at(k) + BATTERY_SWAP_TIME <= T[k], "T_"+itos(k-1)+"_leq_"+itos(k));
		}

		// Optimize model
		model.optimize();

		// Clear result vector
		selected_times->clear();

		// Sanity print
//		printf(" QP Result:\n");

		// Get result
		for(int k = 0; k < M; k++) {
			double start_time = T[k].get(GRB_DoubleAttr_X);
			selected_times->push_back(start_time);

			// Sanity print
//			printf("  %d: wanted %.3f, got %.3f\n", k, desired_times->at(k), start_time);
		}
	}
	catch(GRBException e) {
		// Something went wrong...
		fprintf(stderr, "[ERROR] : K_TSP_Shft::solveStartTimesQP() : Error code = %d\n", e.getErrorCode());
		puts(e.getMessage().c_str());
		exit(1);
	}
	catch(...) {
		// Something else went wrong...
		fprintf(stderr, "[ERROR] : K_TSP_Shft::solveStartTimesQP() : Exception during optimization\n");
		exit(1);
	}
}
