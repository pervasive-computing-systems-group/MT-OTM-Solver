/*
 * MIP_FMDMTHPP.h
 *
 * Created by:	Jonathan Diller
 * On: 			Feb 07, 2022
 *
 * Description: This class formulates a Mixed Integer Program (MIP) for the Fixed Multi-Depot,
 * Multi-Terminal, Hamiltonian Path Problem (FMDMTHPP) and solvers it.
 */

#pragma once

#include <string>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <iostream>

#include "Solution.h"
#include "gurobi_c++.h"

#define DEBUG_MIP_FMDMTHPP		0 || DEBUG
#define MIP_GAP					0.01


class MIP_FMDMTHPP {
public:
	MIP_FMDMTHPP();
	virtual ~MIP_FMDMTHPP();

	/*
	 * Formulate the MIP and run it on the Gurobi solver
	 */
	void Run_MIP(Solution* solution);
protected:
private:
	// Assigned sequence numbers for terminals and depots
	int U_d_k(int k);
	int U_t_k(int k);
	std::string itos(int i);

	int N;
	int M;
	int L_M;
	int BIG_M;
};
