// Partitioning and route solver for Minimum-Time Search-And-Rescue project
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <limits>
#include <chrono>
#include <string.h>

#include "Solver.h"
#include "Solution.h"
#include "defines.h"
#include "Short_Horizons.h"
#include "TotalPath_Follower.h"
#include "Tree_Cutting.h"
#include "Tree_Cutting_LKH.h"

#define DEFAULT_ALGORITHM	1
#define DEFAULT_V_FLAG		E_VelocityFlag::e_NotFixed
#define DEFAULT_FILEPATH	"Test_Set_01/plot_25_2.txt"
#define DEFAULT_DATA_LOG	0
#define DEFAULT_DATA_LOG_LOCATION	"Test_Set_01"


// Run MT-OTM algorithm
int main(int argc, char** argv) {
	int algApproach;
	const char* filePath;
	bool printData;
	const char* outputPath;
	E_VelocityFlag velocity_flag;

	if(argc == 6) {
		algApproach = atoi(argv[1]);
		filePath = argv[2];
		printData = atoi(argv[3]);
		outputPath = argv[4];
		velocity_flag = (E_VelocityFlag)atoi(argv[5]);
	}
	else if(argc == 5) {
		algApproach = atoi(argv[1]);
		filePath = argv[2];
		printData = atoi(argv[3]);
		outputPath = argv[4];
		velocity_flag = DEFAULT_V_FLAG;
	}
	else if(argc == 4) {
		algApproach = atoi(argv[1]);
		filePath = argv[2];
		printData = atoi(argv[3]);
		outputPath = DEFAULT_DATA_LOG_LOCATION;
		velocity_flag = DEFAULT_V_FLAG;
	}
	else if(argc == 3) {
		algApproach = atoi(argv[1]);
		filePath = argv[2];
		printData = DEFAULT_DATA_LOG;
		outputPath = DEFAULT_DATA_LOG_LOCATION;
		velocity_flag = DEFAULT_V_FLAG;
	}
	else {
		printf("Expected 2 ~ 4 arguments:\n (1) Algorithm approach\n (2) File path to plot data\n (3) [OPTIONAL] \"0\" for no data logging, \"1\" for data logging\n (4) [OPTIONAL] File path for output\n (5) [OPTIONAL] velocity flag\n\n");

		exit(0);
	}


	/*
	 * This algorithm searches for the best path for a UAV such that the total time to
	 * survey the entire given search space in minimized. The general algorithms is as
	 * follows:
	 *
	 * Function treeSolve()
	 * variable:
	 * P : collection of paths
	 * m : (partition size)
	 * P_Best : matching of P' (the best found set of paths) and t_tot (total time to fly P' + down-time)
	 * 0. P <- /0, m = 1, P_Best <- {P, inf}
	 * 1. P <- shortest path without partitioning (m = 1)
	 * 2. while dist_max(P) > t_min
	 * 2.1	m := m + 1
	 * 2.2	G := make_G(m)
	 * 2.3	G' = partition(G)
	 * 2.4	P = find_paths(G')
	 * 2.5	t_tot = min{time(P), t_tot}
	 * 3. return best-found-P, t_tot
	 */

	printf("\n\n** Running Min-Time OTM Tree Solver **\n\n");

	Solver* solver;

	TotalPath_Follower tpfAlg;
	Tree_Cutting tfAlg;
	Tree_Cutting_LKH tf_LKH_Alg;

	// Create a solver object, which performs step 1 and 2.3/2.4
	switch(algApproach) {
	case 1: // Tree-walking algorithm
		solver = new Solver(&tpfAlg);
		break;

	case 2:	// Tree-cutting algorithm
		solver = new Solver(&tfAlg);
		break;

	case 3:	// Tree-cutting algorithm with LKH heuristic
		solver = new Solver(&tf_LKH_Alg);
		break;

	default:
		solver = new Solver(&tpfAlg);
	}

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();

	// Run algorithm
	Solution bestSolution(filePath);
	solver->SolvePathPlanning(&bestSolution);

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();

	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

	// Determine mission completion time
	float t_best = bestSolution.TimeToRunSolution(velocity_flag);

	// Print results
	bestSolution.PrintSolution();
	printf("\nBest Found Solution:\n Time: %f\n m: %d\n Comp. Time: %lldms\n", t_best, bestSolution.m_nM, duration);
	double duration_s = (double)duration/1000.0;

	// Print results to file
	if(printData) {
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, "%s", outputPath);
		sprintf(buff + strlen(buff), "/alg_tr_%d.dat", algApproach);
		printf("%s\n", buff);
		pOutputFile = fopen(buff, "a");
		fprintf(pOutputFile, "%d %f %d %.3f\n", bestSolution.m_nN, t_best, bestSolution.m_nM, duration_s);
		fclose(pOutputFile);

//		FILE * pPlotBuildFile;
//		pPlotBuildFile = fopen("PlotBuildFile_tree.txt", "a");
//		fprintf(pPlotBuildFile, "%d %f\n", bestSolution.m_nN, bestSolution.m_tBSTrajectory.mX*t_best);
//		fclose(pPlotBuildFile);
	}
}
