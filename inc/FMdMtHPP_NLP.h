/*
 * FMdMtHPP_NLP.h
 *
 * Created by:	Jonathan Diller
 * On: 			Apr 19, 2022
 *
 * Description: This class runs a NonLinear Program on Gurobi that solves
 * the fixed-MdMtHPP while minimizing completion time.
 */

#pragma once

#include <stdlib.h>
#include <sstream>
#include <string>


#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"
#include "gurobi_c++.h"


#define DEBUG_MT_NLP		1 || DEBUG

class FMdMtHPP_NLP : public Hybrid_Planner {
public:
	FMdMtHPP_NLP();
	virtual ~FMdMtHPP_NLP();

protected:
	/*
	 * Runs a hybrid iterative approach with MIP
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	// Takes in an integer and returns a string of said integer
	std::string itos(int i);

	int U_d_k(int k);

	int N;
	int M;
	int L_M;
	int BIG_M;
};
