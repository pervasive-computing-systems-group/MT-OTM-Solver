/*
 * MinTree_Baseline.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov 04, 2021
 *
 * Description:
 */

#pragma once

#include <math.h>

#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"


#define DEBUG_MIN_TREE	0 || DEBUG


class MinTree_Baseline : public Hybrid_Planner {
public:
	MinTree_Baseline();
	virtual ~MinTree_Baseline();

	/*
	 * Returns the baseline time to run through the minimum spanning tree
	 */
	float GetMinTreeRTBaseline();

protected:
	/*
	 * Runs a hybrid short-horizons approach
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	int m_nM;
	bool m_bSolved;
	float m_fMinTreeDistance;
};
