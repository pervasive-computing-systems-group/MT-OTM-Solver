/*
 * Short_Horizons.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov 02, 2021
 *
 * Description: Short horizons path-planning approach. This is based on work
 * from Jürgen Scherer's "Short and Full Horizon Motion Planning for Persistent
 * multi-UAV Surveillance with Energy and Communication Constraints"
 */

#pragma once

#include <list>
#include <vector>
#include <math.h>

#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"


#define DEBUG_SH	0 || DEBUG

struct T_SHDrone {
	int mID;
	float mX;
	float mY;
	Vertex* mNextVertex;
	// Ordered list of vertices to visit
	std::list<Vertex*> mVertexVisitList;

	T_SHDrone(int id, float x, float y) {
		mID = id;
		mX = x;
		mY = y;
		mNextVertex = NULL;
	}
};


class Short_Horizons : public Hybrid_Planner {
public:
	Short_Horizons(float omega_0_prime, float omega_1_prime, float omega_0, float omega_1, float omega_2);
	virtual ~Short_Horizons();

protected:
	/*
	 * Runs a hybrid short-horizons approach
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	/*
	 * Determines the fitness of drone sHDrone moving to vertex v
	 */
	float getFitness(Solution* solution, T_SHDrone &sHDrone, Vertex* v);
	/*
	 * Returns the distance from drone sHDrone to vertex v
	 */
	float getDistance(T_SHDrone &sHDrone, Vertex* v);

	// Vector of Short-Horizon Drones
	std::vector<T_SHDrone> m_vSHDrones;
	// Fitness weighting parameters single partition
	float m_fOmega_0_prime;
	float m_fOmega_1_prime;
	// Fitness weighting parameters multiple partitions
	float m_fOmega_0;
	float m_fOmega_1;
	float m_fOmega_2;
};
