// Partitioning and route solver for Minimum-Time Search-And-Rescue project
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <limits>
#include <chrono>
#include <string.h>

#include "defines.h"
#include "Solution_Multi.h"
#include "Solver.h"
#include "MMdMtHPP_NLP.h"
#include "K_TSP.h"
#include "K_TSP_Shft.h"

#define DEFAULT_ALGORITHM	1
#define DEFAULT_NUM_UAVS	1
#define DEFAULT_V_FLAG		E_VelocityFlag::e_NotFixed
#define DEFAULT_FILEPATH	"Test_Set_01/plot_25_2.txt"
#define DEFAULT_DATA_LOG	0
#define DEFAULT_DATA_LOG_LOCATION	"Test_Set_01"
#define INF					1000000000000000000.0



// Run MT-OTM algorithm
int main(int argc, char** argv) {
	int algApproach;
	const char* filePath;
	E_VelocityFlag velocity_flag;
	bool printData;
	const char* outputPath;

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
		printf("Expected 2 ~ 5 arguments:\n (1) Algorithm approach\n (3) File path to plot data\n (4) [OPTIONAL] \"0\" for no data logging, \"1\" for data logging\n (5) [OPTIONAL] File path for output\n [OPTIONAL] (6) Velocity flag\n\n\n");

		exit(0);
	}

	printf("\n\n** Running Min-Time OTM algorithm with approach %d **\n\n", algApproach);

	Solver* solver;

	MMdMtHPP_NLP moveGV_NLP;
	K_TSP kTSP;
	K_TSP_Shft kTSPshft;

	// Create a solver object, which performs step 1 and 2.3/2.4
	switch(algApproach) {
	case 1: // Moving GV NLP
		solver = new Solver(&moveGV_NLP);
		break;

	case 2: // Clustering-TSP
		solver = new Solver(&kTSP);
		break;

	case 3: // Clustering-TSP-Shifting
		solver = new Solver(&kTSPshft);
		break;

	default:
		solver = new Solver(&kTSPshft);
	}

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();

	//
	/// Run min-time-OTM algorithm
	//

	// Start with m = 1 (a single partitioning of the search space)
	int m = 0;
	// Initial empty solution
	Solution_Multi* bestSolution = NULL;
	double t_best = INF;

	bool run = true;
	do {
		// Update m
		m += 1;

		// Make a solution with m subtours
		Solution_Multi* retSolution =  new Solution_Multi(filePath, m, 1, 0.0);

		printf("\nRunning Multi-OTM solver with m = %d\n", m);
		// Solve path-planning
		solver->SolvePathPlanning(retSolution);

		// Find the time it takes to run the found solution
		double t_tot = retSolution->TimeToRunMultiSolution();

		// Sanity print
		printf("With m = %d, Total time = %f\n\n", m, t_tot);

		// Run again?
		if(m == retSolution->m_nN) {
			run = false;
		}

		// Did we find a better solution?
		if(t_tot < t_best) {
			// Found new best solution
			if(bestSolution != NULL) {
				delete bestSolution;
			}
			bestSolution = retSolution;
			t_best = t_tot;
		}
		// Is this the first run?
		else if(bestSolution == NULL) {
			bestSolution = retSolution;
		}
		else if(t_best < INF-1) {
			// We are no longer improving by increasing m
			printf("M is too large! (m = %d, best-time = %f)\n Have better solution, stopping algorithm...\n", m, t_best);
			delete retSolution;
			run = false;
		}

	} while(run);

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();

	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();

	// 3. return best-found-P, t_tot
	bestSolution->PrintSolution();
	bestSolution->PrintGraph();
	printf("\nBest Found Solution:\n Time: %f\n m: %d\n Comp. Time: %lldms\n", t_best, bestSolution->m_nM, duration);
	double duration_s = (double)duration/1000.0;

	if(printData) {
		// Print results to file
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, "%s", outputPath);
		sprintf(buff + strlen(buff), "/alg_mlt_%d.dat", algApproach);
		printf("%s\n", buff);
		pOutputFile = fopen(buff, "a");
		fprintf(pOutputFile, "%d %f", bestSolution->m_nN, t_best);
		fprintf(pOutputFile, " %d", bestSolution->m_nM);
		fprintf(pOutputFile, " %.3f\n", duration_s);
		fclose(pOutputFile);
	}

	delete solver;
	delete bestSolution;
}
