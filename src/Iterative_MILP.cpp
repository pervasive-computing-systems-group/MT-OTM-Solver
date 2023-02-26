#include "Iterative_MILP.h"

Iterative_MILP::Iterative_MILP() {
}

Iterative_MILP::~Iterative_MILP() {
}

//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Runs a hybrid iterative sweeping approach
 */
void Iterative_MILP::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Running Iterative MILP *\n\n");
	MILP_FMDMTHPP mip;

	mip.Run_MILP(solution);
}

//***********************************************************
// Private Member Functions
//***********************************************************

