// Partitioning and route solver for Minimum-Time Search-And-Rescue project
#include <stdio.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <limits>
#include <chrono>
#include <string.h>

#include "Solution_Multi.h"
#include "Solver.h"
#include "defines.h"
#include "PathTSP_MIP_PathPlanner.h"
#include "Short_Horizons.h"
#include "KMeans_Partitioner.h"
#include "FMdMtHPP_NLP.h"
#include "PathTSP_LKH.h"

#define DEFAULT_ALGORITHM	1
#define DEFAULT_NUM_UAVS	1
#define DEFAULT_V_FLAG		E_VelocityFlag::e_NotFixed
#define DEFAULT_FILEPATH	"Test_Set_01/plot_25_2.txt"
#define DEFAULT_DATA_LOG	0
#define DEFAULT_DATA_LOG_LOCATION	"Test_Set_01"
#define PRINT_M				1
#define ITERATION_LIMIT		10
#define INF					1000000000000000000.0

// The % error between predicted completion time vs found completion time
#define BS_POSITION_ERROR	0.05
// The magnitude of the difference between the predicted shares vector and the actual shares vector
#define SHARES_VECT_DIFF	0.05

double runtimeFromDistance(double dist, int m, int numUAVs, E_VelocityFlag velocityFlag) {
	double minMax_leg = dist/(double)m;
	// Maximum number of times any single UAV is launched
	int max_num_launches = ceil(m/(double)numUAVs);
	double bestTime = INF;

	// Evaluate each velocity case differently
	if(velocityFlag == e_NotFixed) {
		// Determine if this value for m is feasible
		if(minMax_leg > DIST_MAX) {
			// Not feasible
			bestTime = INF;
		}
		else {
			// Determine best possible time to cover way-points
			bestTime = minMax_leg/V_MAX;
		}
	}
	else if(velocityFlag == e_FixedMax) {
		if(minMax_leg > DIST_OPT) {
			bestTime = INF;
		}
		else {
			bestTime = minMax_leg/V_MAX;
		}
	}
	else { // velocityFlag == e_FixedOpt
		if(minMax_leg > DIST_MAX) {
			bestTime = INF;
		}
		else {
			bestTime = minMax_leg/V_OPT;
		}
	}

	int bat_changes = max_num_launches - 1;

	// Sanity print
	printf("Make %d battery changes\n", bat_changes);

	if(bestTime < INF) {
		bestTime = bestTime * max_num_launches;
		bestTime += (BATTERY_SWAP_TIME * bat_changes);
	}

	return bestTime;
}


/*
 * Determines a minimum-spanning constrained forest for the given plot at filePath
 */
double guessRuntime(std::string filePath, int m, int numUAVs, E_VelocityFlag velocityFlag) {
	// Create basic solution
	Solution solution(filePath, 1, 0);
	// Find distance of MSF of basic solution
	double min_dist = Graph_Theory_Algorithms::MSF_Prims(solution.m_pVertexData, solution.m_nN + 1, m);
	// Find running time based on MSF
	double MSF_time = runtimeFromDistance(min_dist, m, numUAVs, velocityFlag);
	// Verify that this minimum distance was feasible
	if(MSF_time < INF) {
		// Create a new solution using the MSF time
		Solution_Multi solution(filePath, m, numUAVs, MSF_time);
		// Find new minimum distance based on full MSF with all depots/terminals
		double total_min_dist = Graph_Theory_Algorithms::MSF_Prims(solution.m_pVertexData, solution.m_nN + 2*m, m);
		// Find running time of full MSF
		MSF_time = runtimeFromDistance(total_min_dist, m, numUAVs, velocityFlag);
	}

	printf("Found best possible time of %fs\n", MSF_time);
	return MSF_time;
}

float* vect_diff(float* A, float* A_prime, float* diff_A_Aprime, int m) {
	for(int i = 0; i < m; i++) {
		diff_A_Aprime[i] = A[i] - A_prime[i];
	}

	return diff_A_Aprime;
}

float magnitude(float* A, int m) {
	float length = 0;
	for(int i = 0; i < m; i++) {
		length += A[i] * A[i];
	}

	return (float)sqrt(length);
}

/*
 *
 */
Solution_Multi* iterativePathPlanning(Solver* solver, std::string filePath, int m, int numUAVs, E_VelocityFlag velocityFlag) {
	// TODO: need to do memory cleanup!!

	Solution_Multi* retSolution = NULL;

	// Make t based on guess of completion time
//	double t = 1000.0;
	double t = guessRuntime(filePath, m, numUAVs, velocityFlag);

	// Verify that this configuration is possible
	if(t >= INF) {
		// Not feasible, return an unsolved solution
		printf("Bad choice of M! m = %d (pre-solve)\n", m);
		retSolution =  new Solution_Multi(filePath, m, numUAVs, 500);
		retSolution->m_bFeasible = false;
	}
	else {
		// Make shares vector, A, with even splits
		double* A = new double[m];
		for(int i = 0; i < m; i++) {
			A[i] = t/((double)m);
		}

		// Guess variables
		double t_prime = INF;
		double* A_prime = NULL;
		double A_error = INF;
		double time_error = INF;

		// Iteration counter
		int it_counter = 0;

		// Iterative run path-planning until guess == actual
		bool run = true;
		do {
			// Update guess with actual
			t_prime = t;
			A_prime = A;

			// Create a solution object (G')
			retSolution = new Solution_Multi(filePath, m, numUAVs, A_prime);

			// Solve path-planning
			solver->SolvePathPlanning(retSolution);

			// Determine actual completion time
			t = retSolution->TimeToRunSolution(velocityFlag);

			// Determine actual shares
			A = new double[m];
			for(int i = 0; i < retSolution->m_nM; i++) {
				A[i] = retSolution->GetSubtourTime(i);
			}

			if(SANITY_PRINT) {
				printf("Predicted time: %f, actual time: %f\n", t_prime, t);
				printf("Predicted shares:\n");
				for(int i = 0; i < m; i++) {
					printf(" %f", A_prime[i]);
				}
				printf("\n");
				printf("Actual shares:\n");
				for(int i = 0; i < m; i++) {
					printf(" %f", A[i]);
				}
				printf("\n");
			}

			// Verify that we didn't end up with something unrealistic
			if(t >= INF) {
				printf("Bad choice of M! m = %d (post-solve)\n", m);
				run = false;
				retSolution->m_bFeasible = false;
			}
			else {
				// Find the worst predicted time
				A_error = 0;
				for(int i = 0; i < m; i++) {
					double temp = abs(A[i] - A_prime[i])/A[i];
					if(temp > A_error) {
						A_error = temp;
					}
				}

				time_error = abs(t - t_prime)/t;
				it_counter++;

				if((it_counter > ITERATION_LIMIT) || ((time_error <= BS_POSITION_ERROR) && (A_error <= SHARES_VECT_DIFF))) {
					run = false;
				}
			}
		} while(run);

		printf("(Maybe) Found consistent solution!\n t-error: %f, A-error: %f\nCorrecting error...\n", time_error, A_error);
		printf(" Found time %f\n", retSolution->TimeToRunMultiSolution());
		retSolution->CorrectSolution(velocityFlag);
	}

	return retSolution;
}


// Run MT-OTM algorithm
int main(int argc, char** argv) {
	int algApproach;
	int numUAVs;
	const char* filePath;
	bool printData;
	const char* outputPath;
	E_VelocityFlag velocity_flag;

	if(argc == 7) {
		algApproach = atoi(argv[1]);
		numUAVs = atoi(argv[2]);
		filePath = argv[3];
		printData = atoi(argv[4]);
		outputPath = argv[5];
		velocity_flag = (E_VelocityFlag)atoi(argv[6]);
	}
	else if(argc == 6) {
		algApproach = atoi(argv[1]);
		numUAVs = atoi(argv[2]);
		filePath = argv[3];
		printData = atoi(argv[4]);
		outputPath = argv[5];
		velocity_flag = DEFAULT_V_FLAG;
	}
	else if(argc == 5) {
		algApproach = atoi(argv[1]);
		numUAVs = atoi(argv[2]);
		filePath = argv[3];
		printData = atoi(argv[4]);
		outputPath = DEFAULT_DATA_LOG_LOCATION;
		velocity_flag = DEFAULT_V_FLAG;
	}
	else if(argc == 4) {
		algApproach = atoi(argv[1]);
		numUAVs = atoi(argv[2]);
		filePath = argv[3];
		printData = DEFAULT_DATA_LOG;
		outputPath = DEFAULT_DATA_LOG_LOCATION;
		velocity_flag = DEFAULT_V_FLAG;
	}
	else {
		printf("Expected 3 ~ 6 arguments:\n (1) Algorithm approach\n (2) Number of UAVs\n (3) File path to plot data\n (4) [OPTIONAL] \"0\" for no data logging, \"1\" for data logging\n (5) [OPTIONAL] File path for output\n (6) [OPTIONAL] Velocity flag\n\n");

		exit(0);
	}

	printf("\n\n** Running Min-Time OTM algorithm with approach %d with %d UAVs**\n\n", algApproach, numUAVs);

	Solver* solver;

	// Select a partitioning algorithm
	KMeans_Partitioner kmeanPar;

	// Select a path-planning algorithm
	PathTSP_MIP_PathPlanner pTSP_MIP;
	PathTSP_LKH pTSP_LKH;

	// Select a hybrid method
	FMdMtHPP_NLP minTime_NLP;
	Short_Horizons shAlg(-3.054193, 2.740000, -22.584961, 12.125000, 4.253906);

	// Create a solver object, which performs step 1 and 2.3/2.4
	switch(algApproach) {
	case 1: // Short Horizons approach
		solver = new Solver(&shAlg);
		break;

	case 2:	// k-IP
		solver = new Solver(&kmeanPar, &pTSP_MIP);
		break;

	case 3: // NLP
		solver = new Solver(&minTime_NLP);
		break;

	case 4:	// k-TSP
		solver = new Solver(&kmeanPar, &pTSP_LKH);
		break;

	default:
		solver = new Solver(&shAlg);
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

		printf("\nRunning Multi-OTM solver with m = %d\n", m);
		// Solve path-planning
		Solution_Multi* solution = iterativePathPlanning(solver, filePath, m, numUAVs, velocity_flag);

		// Find the time it takes to run the found solution
		double t_tot = solution->TimeToRunMultiSolution();

		// Sanity print
		printf("With m = %d, Total time = %f\n\n", m, t_tot);

		// Run again?
		if(m == solution->m_nN) {
			run = false;
		}
//
//		if(m == 7) {
//			bestSolution = solution;
//		}

		// Did we find a better solution?
		if(t_tot < t_best) {
			// Found new best solution
			if(bestSolution != NULL) {
				delete bestSolution;
			}
			bestSolution = solution;
			t_best = t_tot;
		}
		// Is this the first run?
		else if(bestSolution == NULL) {
			bestSolution = solution;
		}
		else if(t_best < INF-1) {
			// We are no longer improving by increasing m
			printf("M is too large! (m = %d, best-time = %f)\n Have better solution, stopping algorithm...\n", m, t_best);
			delete solution;
			run = false;
		}

	} while(run && m <= 6);

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
		if(PRINT_M) {
			fprintf(pOutputFile, " %d", bestSolution->m_nM);
		}
		fprintf(pOutputFile, " %.3f\n", duration_s);
		fclose(pOutputFile);
	}

	delete solver;
	delete bestSolution;
}
