#include "KMeans_Partitioner.h"

KMeans_Partitioner::KMeans_Partitioner() {
	m_bCreatesTree = false;
	srand(time(NULL));
}

KMeans_Partitioner::~KMeans_Partitioner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Runs the partitioning algorithm
 */
void KMeans_Partitioner::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Creating k-means Partitions *\n\n");
	solution->m_mPartitions.clear();

	// Verify that there is something to be partitioned
	if(solution->m_nM == 1) {
		// No partitioning needed, just add all the vertices
		solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(0, std::vector<Vertex*>()));
		for(int i = 0; i < solution->m_nNumVertices; i++) {
			solution->m_mPartitions.at(0).push_back(solution->m_pVertexData + i);
		}

		return;
	}

	// The set of points that we will perform k-means clustering on
	std::vector<KPoint> pointsSet;
	// The evolving centroid set
	std::vector<KPoint> centroidSet;

//	// Keep track of the max/min limits
//	float x_max, x_min, y_max, y_min;
//	x_max = std::numeric_limits<float>::min();
//	x_min = std::numeric_limits<float>::max();
//	y_max = std::numeric_limits<float>::min();
//	y_min = std::numeric_limits<float>::max();

	// Add all way-point vertices to the points set
	for(int i = 0; i < solution->m_nN; i++) {
		Vertex* v = (solution->m_pVertexData + i);
		pointsSet.push_back(KPoint(v->fX, v->fY, v));

//		if(v->fX > x_max) {
//			x_max = v->fX;
//		}
//		if(v->fX < x_min) {
//			x_min = v->fX;
//		}
//		if(v->fY > y_max) {
//			y_max = v->fY;
//		}
//		if(v->fY < y_min) {
//			y_min = v->fY;
//		}
	}

//	if(DEBUG_KMEANS) {
//		printf("Found: x_max = %f, x_min = %f, y_max = %f, y_min = %f\n", x_max, x_min, y_max, y_min);
//	}
//
//	// Pick random centroids float r3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
//	for(int i = 0; i < solution->m_nM; i++) {
//		float x = x_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(x_max - x_min)));
//		float y = y_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(y_max - y_min)));
//		KPoint cpoint(x, y);
//		cpoint.cluster = i;
//		centroidSet.push_back(cpoint);
//
//		if(DEBUG_KMEANS) {
//			printf("Added centroid at x = %f, y = %f\n", x, y);
//		}
//	}

	// Add the centroids of each depot/terminal pair to points set
	for(int i = 0; i < solution->m_nM; i++) {
		float x = (solution->GetDepotOfPartion(i)->fX - solution->GetTerminalOfPartion(i)->fX)/2.0 + solution->GetDepotOfPartion(i)->fX;
		float y = (solution->GetDepotOfPartion(i)->fY - solution->GetTerminalOfPartion(i)->fY)/2.0 + solution->GetDepotOfPartion(i)->fY;
		KPoint cpoint(x, y);
		cpoint.cluster = i;
		centroidSet.push_back(cpoint);
	}

	// Run k-means with KMEANS_ITERATIONS iterations
	kMeansClustering(&pointsSet, &centroidSet, 10*solution->m_nN);

//	if(DEBUG_KMEANS) {
//		printf("Finished cluster: ");
//		for(auto c : centroidSet) {
//			printf("%d: (%f, %f) ", c.cluster, c.x, c.y);
//		}
//		printf("\nDistances:\n");
//	}
//
//	// Find the distance between depot/terminal centroids, and cluster centroids
//	std::vector<std::vector<int>> A;
//	for(int i = 0; i < solution->m_nM; i++) {
//		A.push_back(std::vector<int>());
//		float x = (solution->GetDepotOfPartion(i)->fX - solution->GetTerminalOfPartion(i)->fX)/2.0 + solution->GetDepotOfPartion(i)->fX;
//		float y = (solution->GetDepotOfPartion(i)->fY - solution->GetTerminalOfPartion(i)->fY)/2.0 + solution->GetDepotOfPartion(i)->fY;
//		for(int j = 0; j < solution->m_nM; j++) {
//			int dist = (int)sqrt(pow((centroidSet.at(j).x - x), 2) + pow((centroidSet.at(j).y - y), 2));
//			A.at(i).push_back(dist);
//
//			if(DEBUG_KMEANS) {
//				printf("\t%d", dist);
//			}
//		}
//		if(DEBUG_KMEANS) {
//			printf("\n");
//		}
//	}

//	// Find a matching between depot/terminal centroids, and cluster centroids
//	std::vector<std::pair<int, int>> result;
//	Graph_Theory_Algorithms::Hungarian_Algorithm(A, result);
//
//	if(DEBUG_KMEANS) {
//		printf("Matching: \n");
//		for(auto p : result) {
//			printf(" %d: %d\n", p.first, p.second);
//		}
//	}


	// Add lists to the partitions to hold each set of vertices
	for(int i = 0; i < solution->m_nM; i++) {
		solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(i, std::vector<Vertex*>()));
	}

//	// Add the vertices to the partitions
//	for(auto p : pointsSet) {
//		if(p.v != NULL) {
//			int assigned_cluster = result.at(p.cluster).second - 1;
//			if(DEBUG_KMEANS) {
//				printf("Adding %d to partition %d\n", p.v->nID, assigned_cluster);
//			}
//			solution->m_mPartitions.at(assigned_cluster).push_back(p.v);
//		}
//	}
//
//	// Add the base-stations
//	for(int i = 0; i < solution->m_nM; i++) {
//		solution->m_mPartitions.at(i).push_back(solution->m_pVertexData + solution->m_nN + i);
//		solution->m_mPartitions.at(i).push_back(solution->m_pVertexData + solution->m_nN + solution->m_nM + i);
//	}

	// Verify that each centroid has at least one vertex
	if(DEBUG_KMEANS)
		printf("Have clusters, verifying assignments\n");
//	for(int i = 0; i < solution->m_nM; i++) {
//		bool has_a_stop = false;
//		int p_count = 0;
//		int single_id = -1;
//		for(int j = 0; j < (int)pointsSet.size(); j++) {
//			if(pointsSet.at(j).cluster == i) {
//				has_a_stop = true;
//				p_count++;
//				single_id = j;
//			}
//		}
//
//		if(has_a_stop && (p_count == 1)) {
//			// Lock-in the single vertex
//			pointsSet.at(single_id).locked = true;
//		}
//		else if(!has_a_stop) {
//			float cent_x = (solution->GetDepotOfPartion(i)->fX - solution->GetTerminalOfPartion(i)->fX)/2.0 + solution->GetDepotOfPartion(i)->fX;
//			float cent_y = (solution->GetDepotOfPartion(i)->fY - solution->GetTerminalOfPartion(i)->fY)/2.0 + solution->GetDepotOfPartion(i)->fY;
//			int closest = -1;
//			float dist_closest = std::numeric_limits<float>::max();
//			for(int j = 0; j < (int)pointsSet.size(); j++) {
//				if(!pointsSet.at(j).locked) {
//					float j_closest = sqrt(pow((pointsSet.at(j).x - cent_x), 2) + pow((pointsSet.at(j).y - cent_y), 2));
//					if(j_closest < dist_closest) {
//						closest = j;
//						dist_closest = j_closest;
//					}
//				}
//			}
//			if(closest > -1) {
//				pointsSet.at(closest).cluster = i;
//				pointsSet.at(closest).locked = true;
//				printf(" Correcting cluster %d\n", i);
//			}
//			else {
//				printf(" Couldn't find a good match for %d!!\n", i);
//			}
//		}
//	}

	bool fixMatches = true;
	while(fixMatches) {
		fixMatches = false;
		int* counts = new int[solution->m_nM];
		for(int j = 0; j < solution->m_nM; j++) {
			counts[j] = 0;
		}

		// Count the size of each partition
		for(int i = 0; i < (int)pointsSet.size(); i++) {
			counts[pointsSet.at(i).cluster]++;
		}

		// Verify that each partition has at least one vertex
		for(int j = 0; j < solution->m_nM; j++) {
			if(counts[j] == 0) {
				// Need to fix this cluster
				fixMatches = true;
				float cent_x = (solution->GetDepotOfPartion(j)->fX -
						solution->GetTerminalOfPartion(j)->fX)/2.0 +
						solution->GetDepotOfPartion(j)->fX;
				float cent_y = (solution->GetDepotOfPartion(j)->fY -
						solution->GetTerminalOfPartion(j)->fY)/2.0 +
						solution->GetDepotOfPartion(j)->fY;
				int closest = -1;
				float dist_closest = std::numeric_limits<float>::max();

				// Search every vertex, find the closest
				for(int i = 0; i < (int)pointsSet.size(); i++) {
					if(!pointsSet.at(i).locked) {
						float i_closest = sqrt(pow((pointsSet.at(i).x - cent_x), 2) + pow((pointsSet.at(i).y - cent_y), 2));
						if(i_closest < dist_closest) {
							closest = i;
							dist_closest = i_closest;
						}
					}
				}

				// Fix partition using best found available vertex
				if(closest > -1) {
					pointsSet.at(closest).cluster = j;
					pointsSet.at(closest).locked = true;
					if(DEBUG_KMEANS)
						printf(" Correcting cluster %d\n", j);
				}
				else {
					printf(" Couldn't find a good match for %d!!\n", j);
				}
			}
		}

		delete[] counts;
	}


	// Add the vertices to the partitions
	for(auto p : pointsSet) {
		if(p.v != NULL) {
			if(DEBUG_KMEANS) {
				printf("Adding %d to partition %d\n", p.v->nID, p.cluster);
			}
			solution->m_mPartitions.at(p.cluster).push_back(p.v);
		}
	}

	// Add the base-stations
	for(int i = 0; i < solution->m_nM; i++) {
		solution->m_mPartitions.at(i).push_back(solution->m_pVertexData + solution->m_nN + i);
		solution->m_mPartitions.at(i).push_back(solution->m_pVertexData + solution->m_nN + solution->m_nM + i);
	}


	// Sanity print
	if(DEBUG_KMEANS) {
		printf("Final partition:\n");
		for(auto p : solution->m_mPartitions) {
			printf(" (%d:", p.first);
			for(Vertex* v : p.second) {
				printf(" %d", v->nID);
			}
			printf(")\n size: %ld\n", p.second.size());
		}
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************

/*
 * Performs Lloyd's k-means clustering algorithm on points using centroids as the initial centroids. The
 * algorithm runs for epocks iterations.
 */
void KMeans_Partitioner::kMeansClustering(std::vector<KPoint>* points, std::vector<KPoint>* centroids, int epochs) {
	// The index of the centroid within the centroids vector
	int n = points->size();
	int k = centroids->size();
	bool run_again = true;
	int iteration = 0;

	// Run clustering iterations
	while(run_again && iteration < epochs) {
		run_again = false;
		iteration++;

		// Check every point against every centroid
		for(int p = 0; p < n; p++) {
			// For each centroid, compute distance from the centroid to the point
			// and update the point's cluster if necessary
			int closest = -1;
			double min_dist = __DBL_MAX__;

			for(int c = 0; c < k; c++) {
				// Check to see if this centroid is better than the last centroid
				if(points->at(p).distance(centroids->at(c)) < min_dist) {
					min_dist = points->at(p).distance(centroids->at(c));
					closest = c;
				}
			}

			// If not -1, assign point to centroid
			if(closest == -1) {
				// Something went wrong...
				fprintf(stderr,"[ERROR] KMeans_Partitioner::kMeansClustering() : Closest centroid was -1\n");
				exit(1);
			}
			else if(closest != points->at(p).cluster) {
				points->at(p).cluster = closest;
				run_again = true;
			}
		}

		// Verify that each centroid has at least one vertex
		for(int c = 0; c < k; c++) {
			bool has_a_stop = false;
			for(KPoint p : (*points)) {
				if(p.cluster == c) {
					has_a_stop = true;
				}
			}

			if(!has_a_stop) {
				run_again = true;
				float cent_x = centroids->at(c).x;
				float cent_y = centroids->at(c).y;
				int closest = 0;
				float dist_closest = sqrt(pow((points->at(0).x - cent_x), 2) + pow((points->at(0).y - cent_y), 2));
				for(int j = 0; j < k; j++) {
					float j_closest = sqrt(pow((points->at(j).x - cent_x), 2) + pow((points->at(j).y - cent_y), 2));
					if(j_closest < dist_closest) {
						closest = j;
						dist_closest = j_closest;
					}
				}
				points->at(closest).cluster = c;
				if(DEBUG_KMEANS)
					printf(" Correcting cluster %d\n", c);
			}
		}

		// Create vectors to keep track of data needed to compute means
		std::vector<int> nPoints;
		std::vector<double> sumX, sumY;
		for (int j = 0; j < k; ++j) {
			nPoints.push_back(0);
			sumX.push_back(0.0);
			sumY.push_back(0.0);
		}

		// Iterate over points to append data to centroids
		for(std::vector<KPoint>::iterator it = points->begin(); it != points->end(); ++it) {
			int clusterId = it->cluster;
			if(clusterId < 0 || clusterId >= k) {
				fprintf(stderr,"[ERROR] KMeans_Partitioner::kMeansClustering() : Bad cluster assignment: %d\n", clusterId);
				exit(1);
			}
			nPoints[clusterId] += 1;
			sumX[clusterId] += it->x;
			sumY[clusterId] += it->y;

			it->minDist = __DBL_MAX__;  // reset distance
		}
		// Compute the new centroids
		for(std::vector<KPoint>::iterator c = centroids->begin(); c != centroids->end(); ++c) {
			int clusterId = c->cluster;
			if(clusterId < 0 || clusterId >= k) {
				fprintf(stderr,"[ERROR] KMeans_Partitioner::kMeansClustering() : c->cluster is bad..\n");
				exit(1);
			}
			c->x = sumX[clusterId] / nPoints[clusterId];
			c->y = sumY[clusterId] / nPoints[clusterId];
		}
	}
}
