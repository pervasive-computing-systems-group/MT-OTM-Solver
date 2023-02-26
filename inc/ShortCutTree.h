/*
 * ShortCutTree.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 30, 2021
 *
 * Description:
 */

#pragma once

#include "Path_Planner.h"

#define T_SHORT_CUT_DEBUG	0 || DEBUG


class ShortCutTree : public Path_Planner {
public:
	ShortCutTree();
	virtual ~ShortCutTree();
protected:

	/*
	 * Short-cuts the trees in a constrained forest to make multiple Hamiltonian paths.
	 * This is based on the short-cuting algorithm given with the Bae-Rathinam algorithm.
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
};
