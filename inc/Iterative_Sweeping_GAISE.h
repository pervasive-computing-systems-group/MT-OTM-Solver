/*
 * Iterative_Sweeping_GAISE.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 28, 2021
 *
 * Description: The Iterative_Sweeping_GAISE class iteratively runs partitioning with the GA-ISE
 * path-planner. The partitions are based on the sweeping approach using a reference triangle to
 * break the tessellation graph up into columns, where all or some portion of a column is given to a partition.
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

#define DEBUG_IT_SGAISE			0 || DEBUG

#define EPSILON_SGAISE_ERROR	0.01

class Iterative_Sweeping_GAISE : public Hybrid_Planner, public Sweeping_Approach {
public:
	Iterative_Sweeping_GAISE();
	virtual ~Iterative_Sweeping_GAISE();

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

	// Member instance of GA_ISE_PathTSP solver
	GA_ISE_PathTSP m_oGAISE_PathTSP;
};
