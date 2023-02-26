/*
 * GA_ISE_PathPlanner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 27, 2021
 *
 * Description:
 */

#pragma once

#include "Path_Planner.h"
#include "GA_defines.h"
#include "GA_ISE_PathTSP.h"


class Basic_GA_ISE : public Path_Planner {
public:
	Basic_GA_ISE();
	virtual ~Basic_GA_ISE();
protected:

	/*
	 * Uses a genetic algorithm to solve the Path TSP on each partition in solution
	 */
	void RunAlgorithm(Solution* solution);

private:
	GA_ISE_PathTSP m_oGAISE_PathTSP;
};
