#include "Short_Horizons.h"

Short_Horizons::Short_Horizons(float omega_0_prime, float omega_1_prime, float omega_0, float omega_1, float omega_2) {
	m_fOmega_0_prime = omega_0_prime;
	m_fOmega_1_prime = omega_1_prime;
	m_fOmega_0 = omega_0;
	m_fOmega_1 = omega_1;
	m_fOmega_2 = omega_2;
}

Short_Horizons::~Short_Horizons() {
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
void Short_Horizons::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("Running Short-Horizons Path-Planning Algorithm\n");

	// Reset drones
	m_vSHDrones.clear();

	// List to hold all vertices
	std::list<Vertex*> availableVerticesList;

	// Put all way-point vertices into a single list
	if(DEBUG_SH)
		printf(" Adding all vertices to available list, ");
	for(int i = 0; i < solution->m_nN; i++) {
		availableVerticesList.push_back(solution->m_pVertexData + i);
	}
	if(DEBUG_SH)
		printf("list size = %ld\n", availableVerticesList.size());

	// Create short-horizon drones
	for(int i = 0; i < solution->m_nM; i++) {
		Vertex* depot_i = solution->GetDepotOfPartion(i);
		T_SHDrone temp(i, depot_i->fX, depot_i->fY);

		// Add this drone to the list of drones
		m_vSHDrones.push_back(temp);
		if(DEBUG_SH)
			printf(" Added drone %d\n", temp.mID);
	}

	// Drone index tracker
	int drone_i = 0;

	// Iteratively select vertices for the drones to visit
	while(!availableVerticesList.empty()) {
		// Track the best found "next-step" for the given drone
		std::list<Vertex*>::iterator itBestNext = availableVerticesList.begin();
		float bestNext_fitness = getFitness(solution, m_vSHDrones.at(drone_i), (*itBestNext));

		// Check all available vertices, pick best next vertex to visit
		for(std::list<Vertex*>::iterator itVNext = itBestNext; itVNext != availableVerticesList.end(); itVNext++) {
			// Determine if v is a better next-step than the current best next-step
			float vNext_fitness = getFitness(solution, m_vSHDrones.at(drone_i), (*itVNext));
			if(vNext_fitness > bestNext_fitness) {
				// v is better than the previous! Update next best vertex
				itBestNext = itVNext;
				bestNext_fitness = vNext_fitness;
			}
		}

		// Add the found Best-Next vertex to the drone and remove it from the available list
		m_vSHDrones.at(drone_i).mNextVertex = (*itBestNext);
		if(DEBUG_SH)
			printf(" Selected vertex %d to drone %d. \n", (*itBestNext)->nID, drone_i);
		availableVerticesList.erase(itBestNext);
		if(DEBUG_SH)
			printf("  |a-list| = %ld\n", availableVerticesList.size());

		// Update drone index
		drone_i++;
		if(drone_i == (int)m_vSHDrones.size()) {
			// All drone have been given a next-step, update all drone positions
			if(DEBUG_SH)
				printf(" Update drone positions\n");
			for(std::vector<T_SHDrone>::iterator it = m_vSHDrones.begin(); it != m_vSHDrones.end(); it++) {
				(*it).mX = (*it).mNextVertex->fX;
				(*it).mY = (*it).mNextVertex->fY;
				(*it).mVertexVisitList.push_back((*it).mNextVertex);
				if(DEBUG_SH)
					printf("  Added vertex %d to drone %d. |%d.v-list| = %ld\n", (*it).mNextVertex->nID,
							(*it).mID, (*it).mID, (*it).mVertexVisitList.size());
				(*it).mNextVertex = NULL;
			}

			drone_i = 0;
		}
	}

	// Update any drone that was still waiting to update its position
	if(DEBUG_SH)
		printf(" Update last drone positions\n");
	for(std::vector<T_SHDrone>::iterator it = m_vSHDrones.begin(); it != m_vSHDrones.end(); it++) {
		if((*it).mNextVertex != NULL) {
			(*it).mVertexVisitList.push_back((*it).mNextVertex);
			if(DEBUG_SH)
				printf("  Added vertex %d to drone %d. |%d.v-list| = %ld\n", (*it).mNextVertex->nID,
						(*it).mID, (*it).mID, (*it).mVertexVisitList.size());
			(*it).mNextVertex = NULL;
		}
	}

	// Add each drone's path to the solution
	for(T_SHDrone drone : m_vSHDrones) {
		Vertex* v = solution->GetDepotOfPartion(drone.mID);
		if(DEBUG_SH)
			printf(" Drone %d, will visit: %ld\n", drone.mID, drone.mVertexVisitList.size());
		for(Vertex* u : drone.mVertexVisitList) {
			// Add edge (v, u) to solution adjacency list
			int a = v->nID;
			int b = u->nID;

			if(a < b) {
				solution->m_pAdjMatrix[a][b] = true;
			}
			else {
				solution->m_pAdjMatrix[b][a] = true;
			}

			v = u;
		}

		// Add the terminal vertex
		solution->m_pAdjMatrix[v->nID][solution->GetTerminalOfPartion(drone.mID)->nID] = true;
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************

/*
 * Determines the fitness, w_s,v , of drone sHDrone moving to vertex v.
 * w_s,v = ω_0 * χ_s,v + ω_1 * min (χ_s',v), where χ_s,v is the distance from
 * given drone s to vertex v and χ_s',v is the distance of some other drone s'
 * to v, s'=\= s.
 */
float Short_Horizons::getFitness(Solution* solution, T_SHDrone &sHDrone, Vertex* v) {
	float fitness = 0;

	if(solution->m_nM > 1) {
		// Find closest other drone
		float closest_dist = 1000000000000000000;
		for(int i = 0; i < (int)m_vSHDrones.size(); i++) {
			float dist_to_i = getDistance(m_vSHDrones.at(i), v);
			if((i != sHDrone.mID) && (dist_to_i < closest_dist)) {
				// Found new closest vertex
				closest_dist = dist_to_i;
			}
		}

		// Determine fitness using distance to v and distance of next-closest drone to v
		fitness = m_fOmega_0 * getDistance(sHDrone, v) + m_fOmega_1 * closest_dist
				+ m_fOmega_2 * solution->GetTerminalOfPartion(sHDrone.mID)->GetDistanceTo(v);
	}
	else {
		// The given drone is alone, determine fitness based only on drone distance
		fitness = m_fOmega_0_prime * getDistance(sHDrone, v)
				+ m_fOmega_1_prime * solution->GetTerminalOfPartion(sHDrone.mID)->GetDistanceTo(v);
	}

//	printf(" fitness(d:%d, v:%d) = %f\n", sHDrone.mID, v->nID, fitness);
	return fitness;
}

/*
 * Returns the distance from drone sHDrone to vertex v
 */
float Short_Horizons::getDistance(T_SHDrone &sHDrone, Vertex* v) {
	return sqrt(pow((sHDrone.mX - v->fX), 2) + pow((sHDrone.mY - v->fY), 2));
}
