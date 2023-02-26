#include "MinTree_Baseline.h"

MinTree_Baseline::MinTree_Baseline() {
	m_fMinTreeDistance = std::numeric_limits<float>::max();
	m_bSolved = false;
	m_nM = -1;
}

MinTree_Baseline::~MinTree_Baseline() {
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
void MinTree_Baseline::RunAlgorithm(Solution* solution) {
	// Determine the distance of the minimum spanning tree
	m_nM = solution->m_nM;
	float tree_dist = R * (solution->m_nN - m_nM);

	// Find the shortest edge for each depot/base station
	for(int i = 0; i < m_nM; i++) {
		float shortest_to_depot = 1000000000000000.0;
		float shortest_to_terminal = 1000000000000000.0;
		Vertex* depot = solution->GetDepotOfPartion(i);
		Vertex* terminal = solution->GetTerminalOfPartion(i);

		// Run through all vertices
		for(int j = 0; j < solution->m_nN; j++) {
			// Check depot
			float dist_to_v = depot->GetDistanceTo((solution->m_pVertexData + j));
			if(dist_to_v < shortest_to_depot) {
				shortest_to_depot = dist_to_v;
			}

			// Check terminal
			dist_to_v = terminal->GetDistanceTo((solution->m_pVertexData + j));
			if(dist_to_v < shortest_to_terminal) {
				shortest_to_terminal = dist_to_v;
			}
		}

		// Add these distance to distance of min-spanning tree
		tree_dist += shortest_to_depot;
		tree_dist += shortest_to_terminal;
	}

	m_fMinTreeDistance = tree_dist;
	m_bSolved = true;
}

/*
 * Returns the baseline time to run through the minimum spanning tree
 */
float MinTree_Baseline::GetMinTreeRTBaseline() {
	float time = 0;

	// Verify that we have set
	if(!m_bSolved) {
		// We didn't solve this problem yet!!
		printf("[ERROR] : Asked for min-tree run time but a solution hasn't been solved yet!\n");
		exit(0);
	}

	if(SANITY_PRINT)
		printf("Calculating Min-Tree Runtime for m = %d\n", m_nM);


	// We assume an even split of the tree
	float legDist = m_fMinTreeDistance / m_nM;
	float m = (float)(DIST_MAX - DIST_OPT)/(V_MAX - V_OPT);

	if(SANITY_PRINT)
		printf(" legDist = %f\n", legDist);

	// Determine time per-leg based on distance/velocity constraint
	if(legDist > DIST_MAX) {
		if(SANITY_PRINT)
			printf(" TOO FAR!\n");

		return std::numeric_limits<float>::max();
	}
	else if(legDist <= DIST_OPT) {
		// Fly at max speed
		time = legDist * (1.0/V_MAX);

		if(SANITY_PRINT)
			printf(" go V_MAX = %f, t = %f\n", V_MAX, time);
	}
	else {
		// Determine fastest speed to move through this leg
		float v = (legDist - DIST_OPT + m * V_OPT)/m;
		time = legDist * (1.0/v);

		if(SANITY_PRINT)
			printf(" go v = %f, t = %f\n", v, time);
	}

	// We do this amount of time per-leg
	time = time * m_nM;
	// Add in the battery swap time
	time += BATTERY_SWAP_TIME * (m_nM - 1);
	if(SANITY_PRINT)
		printf(" total time = %f\n", time);

	return time;
}

//***********************************************************
// Private Member Functions
//***********************************************************
