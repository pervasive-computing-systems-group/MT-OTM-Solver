#include "Tree_Solution.h"

Tree_Solution::Tree_Solution(std::string gaph_path, int m) : Solution(gaph_path, m) {}

Tree_Solution::~Tree_Solution() {}


//***********************************************************
// Public Member Functions
//***********************************************************


// Determine the amount of time required for a UAV to run this solution
float Tree_Solution::TimeToRunSolution(bool bLagrangianRelaxation) {
	float time = 0;

	if(SANITY_PRINT)
		printf("Calculating Min-Tree Runtime for m = %d\n", m_nM);


	// We assume an even split of the tree
	float legDist = getMinTreeLength() / m_nM;
	float m = (float)(DIST_MAX - DIST_OPT)/(V_MAX - V_OPT);

	if(SANITY_PRINT)
		printf(" legDist = %f\n", legDist);

	// Determine time per-leg based on distance/velocity constraint
	if(legDist > DIST_MAX) {
		if(bLagrangianRelaxation) {
			// Relax the max-distance constraint (used for training)
			time = legDist * (1.0/V_OPT);
			// Add penalty for over-shooting
			time += (legDist - DIST_MAX) * (1.0/V_MAX);

			if(SANITY_PRINT)
				printf(" TOO FAR! t = %f\n", time);
		}
		else {
			// This is too long to fly!
			return std::numeric_limits<float>::max();
		}
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

// Return the distance (in meters) of partition n of this solution
float Tree_Solution::DistanceOfPartition(int n) {
	return getMinTreeLength() / m_nM;
}

// Returns the distance of the maximum leg in this solution
float Tree_Solution::GetMaxLegDist() {
	return getMinTreeLength() / m_nM;
}

//***********************************************************
// Private Member Functions
//***********************************************************

// Determines the minimum time to run this solution given m partitions
float Tree_Solution::getMinTreeLength() {
	// Determine the distance of the minimum spanning tree
	float tree_dist = R * (m_nN - m_nM);

	// Find the shortest edge for each depot/base station
	for(int i = 0; i < m_nM; i++) {
		float shortest_to_depot = 1000000000000000.0;
		float shortest_to_terminal = 1000000000000000.0;
		Vertex* depot = GetDepotOfPartion(i);
		Vertex* terminal = GetTerminalOfPartion(i);

		// Run through all vertices
		for(int j = 0; j < m_nN; j++) {
			// Check depot
			float dist_to_v = depot->GetDistanceTo((m_pVertexData + j));
			if(dist_to_v < shortest_to_depot) {
				shortest_to_depot = dist_to_v;
			}

			// Check terminal
			dist_to_v = terminal->GetDistanceTo((m_pVertexData + j));
			if(dist_to_v < shortest_to_terminal) {
				shortest_to_terminal = dist_to_v;
			}
		}

		// Add these distance to distance of min-spanning tree
		tree_dist += shortest_to_depot;
		tree_dist += shortest_to_terminal;
	}

	return tree_dist;
}
