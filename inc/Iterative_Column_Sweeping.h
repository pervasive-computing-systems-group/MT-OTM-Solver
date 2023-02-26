/*
 * Iterative_Column_Sweeping.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 25, 2021
 *
 * Description:
 */

#pragma once

#include <stdlib.h>
#include <map>
#include <queue>
#include <list>
#include <stack>

#include "defines.h"
#include "Solution.h"
#include "Sweeping_Approach.h"
#include "Hybrid_Planner.h"

#define DEBUG_IT_COL_SWEEP	0 || DEBUG
#define EPSILON_II_ERROR	0.05

class Iterative_Column_Sweeping : public Hybrid_Planner, public Sweeping_Approach {
public:
	Iterative_Column_Sweeping();
	virtual ~Iterative_Column_Sweeping();

protected:
	/*
	 * Runs a hybrid iterative sweeping approach
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	/*
	 * Runs sweeping partitioning based on given parameters
	 */
	void sweepingPartition(Solution* pSolution, std::vector<float>* pSharesVector);
	/*
	 * Finds the columns, and stores them in columnsMap, for the given solution. Assumes that m_pA, m_pB,
	 * and m_pC have already been assigned. Returns the minimum column index number (min_col <= 0).
	 */
	int findColumns(Solution* solution, std::map<int, std::priority_queue<T_ColumnV>>* columnsMap);
	/*
	 * Finds the columns that each vertex belongs to and puts them into a single list. The order of the list is
	 * based off of the same order that findColumns() returns but with the resulting columns pushed (in ascending order)
	 * into a single list.
	 */
	void findColumnList(Solution* solution, std::list<Vertex*>* columnsOrderedList);
	/*
	 * Finds the best route using sweeping path-planning on partition p by alternating the order of referee vertices
	 * A, B, and C. Assumes that the partition map already has all vertices for this partition in priority queues.
	 */
	void findBestSweepingRoute(Solution* solution, int p, std::list<Vertex*> &routeList);
	/*
	 * Performs sweeping path-planning on the given partition p and partition map. Assumes that the
	 * partition map already has all vertices for this partition in priority queues.
	 */
	void solveGivenDirection(Solution* solution, int p, std::list<Vertex*> &routeList, Vertex* pA, Vertex* pB, Vertex* pC);
};
