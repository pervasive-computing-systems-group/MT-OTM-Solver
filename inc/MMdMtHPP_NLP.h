/*
 * MMdMtHPP_NLP.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 19, 2023
 *
 * Description: Uses the Gurobi solver to solve moving-MdMtHP Problems as a NLP.
 */

#pragma once

#include <stdlib.h>
#include <sstream>
#include <string>


#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"
#include "gurobi_c++.h"


#define DEBUG_MUGV_NLP		0 || DEBUG

class MMdMtHPP_NLP : public Hybrid_Planner {
public:
	MMdMtHPP_NLP();
	virtual ~MMdMtHPP_NLP();

protected:
	/*
	 * Runs a hybrid iterative approach with MIP
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	int U_d_k(int k);

	int N;
	int M;
	int L_M;
	int BIG_M;
};
