#include "Short_Horizons_II.h"

Short_Horizons_II::Short_Horizons_II(float omega_0_prime, float omega_1_prime, float omega_0, float omega_1, float omega_2) {
	m_fOmega_0_prime = omega_0_prime;
	m_fOmega_1_prime = omega_1_prime;
	m_fOmega_0 = omega_0;
	m_fOmega_1 = omega_1;
	m_fOmega_2 = omega_2;
}

Short_Horizons_II::~Short_Horizons_II() {
}

// operator overload to compare two T_SHDroneII
bool operator<(const T_SHDroneII &a, const T_SHDroneII &b) {
	return a.mDist_traveled > b.mDist_traveled;
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
void Short_Horizons_II::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("Running Short-Horizons Path-Planning Algorithm\n");

	// Reset drones
	m_vSHDrones.clear();

	// List to hold all vertices
	std::list<Vertex*> availableVerticesList;

	// Put all way-point vertices into a single list
	if(DEBUG_SH_II)
		printf(" Adding all vertices to available list, ");
	for(int i = 0; i < solution->m_nN; i++) {
		availableVerticesList.push_back(solution->m_pVertexData + i);
	}
	if(DEBUG_SH_II)
		printf("list size = %ld\n", availableVerticesList.size());

	// Create short-horizon drones
	for(int i = 0; i < solution->m_nM; i++) {
		Vertex* depot_i = solution->GetDepotOfPartion(i);
		T_SHDroneII temp(i, depot_i->fX, depot_i->fY);

		// Add this drone to the list of drones
		m_vSHDrones.push_back(temp);
		if(DEBUG_SH_II)
			printf(" Added drone %d\n", temp.mID);
	}

	// Drone index tracker
	int drone_i = 0;
	int iteration = 0;

	// Iteratively select vertices for the drones to visit
	while(!availableVerticesList.empty()) {
		// For the first few steps through the search space, let the drones take turns
		if(!switchToDistEqualling(solution, iteration)) {
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
	//		m_vSHDrones.at(drone_i % m_vSHDrones.size()).mVertexVisitList.push_back(*itBestNext);
			if(DEBUG_SH_II)
				printf(" Selected vertex %d to drone %d. \n", (*itBestNext)->nID, drone_i);
			availableVerticesList.erase(itBestNext);
			if(DEBUG_SH_II)
				printf("|a-list| = %ld\n", availableVerticesList.size());

			// Update drone index
			drone_i++;
			if(drone_i == (int)m_vSHDrones.size()) {
				// All drone have been given a next-step, update all drone positions
				if(DEBUG_SH_II)
					printf(" Update drone positions\n");
				for(std::vector<T_SHDroneII>::iterator it = m_vSHDrones.begin(); it != m_vSHDrones.end(); it++) {
					// Update distance traveled
					(*it).mDist_traveled += getDistance((*it), (*it).mNextVertex);
					// Update current position
					(*it).mX = (*it).mNextVertex->fX;
					(*it).mY = (*it).mNextVertex->fY;
					(*it).mVertexVisitList.push_back((*it).mNextVertex);
					if(DEBUG_SH_II)
						printf("  Added vertex %d to drone %d. |%d.v-list| = %ld\n", (*it).mNextVertex->nID,
								(*it).mID, (*it).mID, (*it).mVertexVisitList.size());
					(*it).mNextVertex = NULL;
				}

				drone_i = 0;
			}

			iteration++;

			if(switchToDistEqualling(solution, iteration)) {
				// Update all current positions
				for(T_SHDroneII drone : m_vSHDrones) {
					if(drone.mNextVertex != NULL) {
						// Update distance traveled
						drone.mDist_traveled += getDistance(drone, drone.mNextVertex);
						// Update current position
						drone.mX = drone.mNextVertex->fX;
						drone.mY = drone.mNextVertex->fY;
						drone.mVertexVisitList.push_back(drone.mNextVertex);
						drone.mNextVertex = NULL;
					}

					// Add the drones to our queue, next iteration we will start pulling drone from here
					m_qSHDrones.push(drone);
				}
			}
		}
		else { // Always let the drone with the shortest distance traveled go first
			// Grab the drone with the shortest distance traveled
			T_SHDroneII drone = m_qSHDrones.top();
			m_qSHDrones.pop();

			// Track the best found "next-step" for the given drone
			std::list<Vertex*>::iterator itBestNext = availableVerticesList.begin();
			float bestNext_fitness = getFitness(solution, drone, (*itBestNext));

			// Check all available vertices, pick best next vertex to visit
			for(std::list<Vertex*>::iterator itVNext = itBestNext; itVNext != availableVerticesList.end(); itVNext++) {
				// Determine if v is a better next-step than the current best next-step
				float vNext_fitness = getFitness(solution, drone, (*itVNext));
				if(vNext_fitness > bestNext_fitness) {
					// v is better than the previous! Update next best vertex
					itBestNext = itVNext;
					bestNext_fitness = vNext_fitness;
				}
			}

			// Update drones position
			drone.mDist_traveled += getDistance(drone, (*itBestNext));
			drone.mX = (*itBestNext)->fX;
			drone.mY = (*itBestNext)->fY;
			drone.mVertexVisitList.push_back((*itBestNext));

			if(DEBUG_SH_II)
				printf(" Selected vertex %d to drone %d. \n", (*itBestNext)->nID, drone.mID);

			availableVerticesList.erase(itBestNext);

			if(DEBUG_SH_II)
				printf("|a-list| = %ld\n", availableVerticesList.size());

			// Add drone back to queue
			m_qSHDrones.push(drone);
		}
	}

	// Add each drone's path to the solution
	while(!m_qSHDrones.empty()) {
		T_SHDroneII drone = m_qSHDrones.top();
		m_qSHDrones.pop();

		printf("Adding path for drone %d\n size = %ld\n ", drone.mID, drone.mVertexVisitList.size());
		Vertex* v = solution->GetDepotOfPartion(drone.mID);
		for(Vertex* u : drone.mVertexVisitList) {
			printf(" %d ->", v->nID);
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
		printf(" %d\n", solution->GetTerminalOfPartion(drone.mID)->nID);
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
float Short_Horizons_II::getFitness(Solution* solution, T_SHDroneII &sHDrone, Vertex* v) {
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
float Short_Horizons_II::getDistance(T_SHDroneII &sHDrone, Vertex* v) {
	return sqrt(pow((sHDrone.mX - v->fX), 2) + pow((sHDrone.mY - v->fY), 2));
}

/*
 * Returns true if the desired condition has been met to pick drone by distance completed
 */
bool Short_Horizons_II::switchToDistEqualling(Solution* solution, int iteration) {
	return iteration >= (solution->m_nM * 3);
//	return iteration >= (solution->m_nN / 3);
}
