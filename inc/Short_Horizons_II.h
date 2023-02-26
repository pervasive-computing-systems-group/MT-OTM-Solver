/*
 * Short_Horizons_II.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov 09, 2021
 *
 * Description: Based on original short horizons path-planning approach except
 * this version attempts to keep the traveled distance of each UAV equal.
 */

#pragma once

#include <list>
#include <vector>
#include <queue>
#include <math.h>

#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"


#define DEBUG_SH_II	0 || DEBUG

struct T_SHDroneII {
	int mID;
	float mX;
	float mY;
	float mDist_traveled;
	Vertex* mNextVertex;
	// Ordered list of vertices to visit
	std::list<Vertex*> mVertexVisitList;

	T_SHDroneII(int id, float x, float y) {
		mID = id;
		mX = x;
		mY = y;
		mDist_traveled = 0;
		mNextVertex = NULL;
	}

	T_SHDroneII(const T_SHDroneII &drone) {
		mID = drone.mID;
		mX = drone.mX;
		mY = drone.mY;
		mDist_traveled = drone.mDist_traveled;
		mNextVertex = drone.mNextVertex;
		mVertexVisitList = drone.mVertexVisitList;
	}
};


class Short_Horizons_II : public Hybrid_Planner {
public:
	Short_Horizons_II(float omega_0_prime, float omega_1_prime, float omega_0, float omega_1, float omega_2);
	virtual ~Short_Horizons_II();

protected:
	/*
	 * Runs a hybrid short-horizons approach
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	/*
	 * Determines the fitness of drone sHDrone moving to vertex v
	 */
	float getFitness(Solution* solution, T_SHDroneII &sHDrone, Vertex* v);
	/*
	 * Returns the distance from drone sHDrone to vertex v
	 */
	float getDistance(T_SHDroneII &sHDrone, Vertex* v);
	/*
	 * Returns true if the desired condition has been met to pick drone by distance completed
	 */
	bool switchToDistEqualling(Solution* solution, int iteration);

	// Vector of Short-Horizon Drones
	std::vector<T_SHDroneII> m_vSHDrones;
	// Queue used to sort drones based on distance traveled
	std::priority_queue<T_SHDroneII> m_qSHDrones;
	// Fitness weighting parameters single partition
	float m_fOmega_0_prime;
	float m_fOmega_1_prime;
	// Fitness weighting parameters multiple partitions
	float m_fOmega_0;
	float m_fOmega_1;
	float m_fOmega_2;
};
