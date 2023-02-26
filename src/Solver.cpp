#include "Solver.h"

Solver::Solver(Hybrid_Planner* pHybridPlanner) {
	m_pPartitioner = NULL;
	m_pPath_Planner = NULL;
	m_pHybridPlanner = pHybridPlanner;
	m_pEmptyGraphPlanner = NULL;
}

Solver::Solver(EmptyGraph_Planner* pEmptyGraphPlanner) {
	m_pPartitioner = NULL;
	m_pPath_Planner = NULL;
	m_pHybridPlanner = NULL;
	m_pEmptyGraphPlanner = pEmptyGraphPlanner;
}

Solver::Solver(Partitioner* pPartitioner, Path_Planner* pPath_Planner) {
	// Seed rand()
	srand(time(NULL));
	m_pPartitioner = pPartitioner;
	m_pPath_Planner = pPath_Planner;
	m_pHybridPlanner = NULL;
	m_pEmptyGraphPlanner = NULL;
}

Solver::~Solver() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * This function solves the given path in the gaph_path file using m partitions.
 * It uses the Partitioner and Path_Planner object that were given to the solver   Generic_Partitioner
 * in its constructor.
 */
void Solver::SolvePathPlanning(Solution* solution) {
	if(solution->IsFeasible()) {
		if(m_pHybridPlanner != NULL) {
			// Run hybrid partitioner/path-planner algorithm
			m_pHybridPlanner->RunHybridAlgorithm(solution);
		}
		else {
			// Run the partitioning algorithm
			if(this->m_pPartitioner != NULL) {
				m_pPartitioner->RunPartitionAlgorithm(solution);
			}
			else {
				// Run generic partitioner
				Generic_Partitioner genPartitioner;
				genPartitioner.RunPartitionAlgorithm(solution);
			}

			// Run path planner
			if(this->m_pPath_Planner != NULL) {
				m_pPath_Planner->RunPathPlanningAlgorithm(solution);
			}
			else {
				// Run generic path planner
				Random_PathPlanner randPP;
				randPP.RunPathPlanningAlgorithm(solution);
			}
		}

		// Verify that our solution is correct
//		solution->CorrectSolution();
	}
	else if(m_pEmptyGraphPlanner != NULL) {
		m_pEmptyGraphPlanner->RunEGAlgorithm(solution);
	}
	else {
		// Don't waste time on this, it is not possible to solve the given solution
		printf("Solution is not feasible, increase number of partitions\n");
	}
}
