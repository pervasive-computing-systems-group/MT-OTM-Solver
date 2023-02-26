/*
 * Iterative_GAISE.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 28, 2021
 *
 * Description: The Iterative_GAISE class iteratively runs partitioning with the GA-ISE
 * path-planner. The partitions are based on the x position of each vertex.
 */

#pragma once

#include <stdlib.h>
#include <map>
#include <queue>
#include <list>
#include <stack>

#include "Sweeping_Approach.h"
#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"
#include "GA_ISE_PathTSP.h"

#define DEBUG_IT_GAISE			0 || DEBUG

#define EPSILON_IT_GAISE_ERROR	0.01


// Wrapper struct to help sort vertices for vertical partitioning
struct T_VERT_NODE {
	Vertex* pV;

	T_VERT_NODE() {
		pV = NULL;
	}

	T_VERT_NODE(Vertex* v) {
		pV = v;
	}

	T_VERT_NODE(const T_VERT_NODE &vertNode) {
		pV = vertNode.pV;
	}
};


// Wrapper struct to help sort vertices for vertical partitioning
struct T_VSWEEP_NODE {
	Vertex* pV;
	float fTheta;

	T_VSWEEP_NODE(Vertex* v, float theta) {
		pV = v;
		fTheta = theta;
	}

	T_VSWEEP_NODE(const T_VSWEEP_NODE &vertNode) {
		pV = vertNode.pV;
		fTheta = vertNode.fTheta;
	}
};

class Iterative_GAISE : public Hybrid_Planner {
public:
	Iterative_GAISE();
	virtual ~Iterative_GAISE();

protected:
	/*
	 * Runs a hybrid iterative sweeping approach
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	/*
	 * Runs a vertical partitioner based on given parameters
	 */
	void orderedPartition(Solution* pSolution, std::vector<float>* pSharesVector);
	/*
	 * Finds the columns that each vertex belongs to and puts them into a single list. The order of the list is
	 * based off of the same order that findColumns() returns but with the resulting columns pushed (in ascending order)
	 * into a single list.
	 */
	void findVerticalOrder(Solution* solution, std::list<Vertex*>* verticalOrderedList);
	/*
	 * Determines the order to add vertices based on a sweeping vector approach. One can visualize this as dividing up
	 * the search space into pizza slices
	 */
	void findVectorSweepingOrder(Solution* solution, std::list<Vertex*>* verticalOrderedList);

	// Member instance of GA_ISE_PathTSP solver
	GA_ISE_PathTSP m_oGAISE_PathTSP;
};
