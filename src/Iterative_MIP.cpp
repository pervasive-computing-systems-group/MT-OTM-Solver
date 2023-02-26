#include "Iterative_MIP.h"

Iterative_MIP::Iterative_MIP() {
}

Iterative_MIP::~Iterative_MIP() {
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
void Iterative_MIP::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Running Iterative MIP *\n\n");
	MIP_FMDMTHPP mip;

	mip.Run_MIP(solution);
}

//***********************************************************
// Private Member Functions
//***********************************************************

