// Partitioning and route solver for Minimum-Time Search-And-Rescue project
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <limits>
#include <bits/stdc++.h>
#include <cstdlib>

#include "Solver.h"
#include "Solution.h"
#include "Iterative_GAISE.h"
#include "Short_Horizons.h"
#include "Iterative_MIP.h"

#define DEBUG_TUNING		0

#define FILEPATH	"Test_Set_01/plot_25_2.txt"
#define BS_TUNING_ERROR		0.03
#define INF					1000000000000000000.0

#define RUN_SINGLE			0
#define RUN_NEXT_NEIGHBOR	1
#define RUN_PROBING			2
#define RUN_BS_TUNING		3
#define RUN_DATA_GEN		4
#define RUN_MIP				5
#define RUN_LKH_DATA		6

#define RUN					RUN_LKH_DATA

typedef std::pair<float, int> step_pair;

float runtimeFromDistance(float dist, int m, E_VelocityFlag velocityFlag) {
	float max_leg = dist/(float)m;
	float bestTime = INF;

	// Evaluate each velocity case differently
	if(velocityFlag == e_NotFixed) {
		// Determine if this value for m is feasible
		if(max_leg > DIST_MAX) {
			// Not feasible
			bestTime = INF;
		}
		else {
			// Determine best possible time to cover way-points
			bestTime = dist/V_MAX;
		}
	}
	else if(velocityFlag == e_FixedMax) {
		if(max_leg > DIST_OPT) {
			bestTime = INF;
		}
		else {
			bestTime = dist/V_MAX;
		}
	}
	else { // velocityFlag == e_FixedOpt
		if(max_leg > DIST_MAX) {
			bestTime = INF;
		}
		else {
			bestTime = dist/V_OPT;
		}
	}

	bestTime += BATTERY_SWAP_TIME * (m - 1);

	return bestTime;
}


/*
 * Determines a minimum-spanning constrained forest for the given plot at filePath
 */
float guessRuntime(std::string filePath, int m, E_VelocityFlag velocityFlag) {
	// Create basic solution
	Solution solutionI(filePath, 1, 0);
	// Find distance of MSF of basic solution
	float min_dist = Graph_Theory_Algorithms::MSF_Prims(solutionI.m_pVertexData, solutionI.m_nN + 1, m);
	// Find running time based on MSF
	float MSF_time = runtimeFromDistance(min_dist, m, velocityFlag);

	// Create a new solution using the MSF time
	Solution solutionII(filePath, m, MSF_time);
	// Find new minimum distance based on full MSF with all depots/terminals
	float total_min_dist = Graph_Theory_Algorithms::MSF_Prims(solutionII.m_pVertexData, solutionII.m_nN + 2*m, m);
	// Find running time of full MSF
	MSF_time = runtimeFromDistance(total_min_dist, m, velocityFlag);

	return MSF_time;
}

float runASet(Solver* solver, std::string filePath, int m) {
	float retVal = -1;
	float t_bl = guessRuntime(filePath, m, E_VelocityFlag::e_NotFixed);
	Solution solution(filePath, m, t_bl);
	solver->SolvePathPlanning(&solution);
	float t_sol = solution.TimeToRunSolution(E_VelocityFlag::e_NotFixed, true);
	retVal = t_sol - t_bl;

//	printf("Found time-over: %f\n t_bl: %f, t_sol: %f\n", retVal, t_bl, t_sol);
	return retVal;
}


// Running training set
float runTrainingSet(float o0_p, float o1_p, float o0, float o1, float o2, bool run_singles, bool run_multis) {
	float running_time_over = 0;

	Short_Horizons shAlg(o0_p, o1_p, o0, o1, o2);
	Solver solver(&shAlg);

	if(run_singles) {
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_5_0.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_5_1.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_5_2.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_12_0.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_12_1.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_12_2.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_12_3.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_15_0.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_15_1.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_15_2.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_15_3.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_15_4.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_20_0.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_20_1.txt", 1);
		running_time_over += runASet(&solver, "SH_train_rnd_I/plot_20_2.txt", 1);
	}

	if(run_multis){


		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_20_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_20_1.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_20_2.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_25_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_25_1.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_25_2.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_35_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_35_1.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_35_2.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_40_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_40_1.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_40_2.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_50_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_50_1.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_50_2.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_50_3.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_55_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_55_1.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_55_2.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_60_0.txt", 4);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_60_1.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_65_0.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_65_1.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_65_2.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_70_0.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_70_1.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_75_0.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_75_0.txt", 2);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_80_0.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_80_1.txt", 4);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_80_2.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_100_0.txt", 4);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_100_1.txt", 4);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_100_2.txt", 3);
		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_100_3.txt", 3);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_125_0.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_125_1.txt", 5);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_125_2.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_125_3.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_175_0.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_175_1.txt", 5);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_175_2.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_200_0.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_200_1.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_200_2.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_200_3.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_200_4.txt", 4);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_250_0.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_250_1.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_250_2.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_250_3.txt", 8);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_250_4.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_300_0.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_300_1.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_300_2.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_300_3.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_300_4.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_350_0.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_350_1.txt", 9);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_350_2.txt", 8);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_350_3.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_350_4.txt", 8);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_400_0.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_400_1.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_400_2.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_400_3.txt", 6);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_400_4.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_450_0.txt", 9);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_450_1.txt", 7);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_450_2.txt", 9);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_450_3.txt", 8);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_500_0.txt", 13);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_500_1.txt", 13);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_500_2.txt", 12);
//		running_time_over += runASet(&solver, "SH_train_rnd_II/plot_500_3.txt", 11);
	}

	if(run_singles && !run_multis) {
		// 15 singles
		return running_time_over/15.0;
	}
	else if(!run_singles && run_multis) {
		// 60 multis
		return running_time_over/35.0;
	}
	else {
		// 75 total
		return running_time_over/50.0;
	}
}

float determineActualRunTime(const char* filePath, int m) {
	Iterative_GAISE itGAISE;
	Solver solver(&itGAISE);

	// Create a solution object
	Solution* bestSolution = new Solution(filePath, m);
	// Solve path-planning
	solver.SolvePathPlanning(bestSolution);
	// Determine run time
	float time_1 = bestSolution->TimeToRunSolution();
	// Delete solution
	delete bestSolution;

	// Repeat two more times
	bestSolution = new Solution(filePath, m);
	solver.SolvePathPlanning(bestSolution);
	float time_2 = bestSolution->TimeToRunSolution();
	delete bestSolution;

	bestSolution = new Solution(filePath, m);
	solver.SolvePathPlanning(bestSolution);
	float time_3 = bestSolution->TimeToRunSolution();

	// Determine which solution worked the best
	float temp_best = std::min(time_1, time_2);
	float best_time = std::min(temp_best, time_3);
	printf("Time found: %f, Predicted: %f\n", best_time, bestSolution->GetPredictedCompletionTime());

	// Verify that we didn't end up with something unrealistic
	if(best_time > (std::numeric_limits<float>::max() - 10.0)) {
		printf("M is too small!\n");
		return best_time;
	}

	int tries = 0;
	while(abs((best_time - bestSolution->GetPredictedCompletionTime())/best_time) > BS_TUNING_ERROR) {
		printf("\nRe-calculating completion time\n");

		// Reset solution
		delete bestSolution;

		// Repeat the process
		bestSolution = new Solution(filePath, m, best_time);
		solver.SolvePathPlanning(bestSolution);
		time_1 = bestSolution->TimeToRunSolution();
		delete bestSolution;

		bestSolution = new Solution(filePath, m, best_time);
		solver.SolvePathPlanning(bestSolution);
		time_2 = bestSolution->TimeToRunSolution();
		delete bestSolution;

		bestSolution = new Solution(filePath, m, best_time);
		solver.SolvePathPlanning(bestSolution);
		time_3 = bestSolution->TimeToRunSolution();

		// Determine which solution worked the best
		temp_best = std::min(time_1, time_2);
		best_time = std::min(temp_best, time_3);
		printf("Time found: %f, Predicted: %f\n", best_time, bestSolution->GetPredictedCompletionTime());

		// Verify that we didn't end up with something unrealistic
		if(best_time > (std::numeric_limits<float>::max() - 10.0)) {
			printf("M is too small!\n");
			break;
		}

		tries++;
		if(tries > 4) {
			printf("Doesn't settle! %s\n", filePath);
			break;
		}
	}

	printf("\nFound a realistic solution time of %f\n", best_time);

	delete bestSolution;

	return best_time;
}

void getCentroid(Solution* solution, float &x, float &y) {
	// Find the cumulative x and y values
	float cumulative_x = 0;
	float cumulative_y = 0;

	for(int i = 0; i < solution->m_nN; i++) {
		cumulative_x += (solution->m_pVertexData + i)->fX;
		cumulative_y += (solution->m_pVertexData + i)->fY;
	}

	x = cumulative_x/solution->m_nN;
	y = cumulative_y/solution->m_nN;
}

float getCentroidVectorMag(Solution* solution) {
	float x;
	float y;

	// Get the location of the centroid
	getCentroid(solution, x, y);

	if(DEBUG_TUNING)
		printf("x: %f, y: %f\n", x, y);

	return sqrt(pow(x, 2) + pow(y, 2));
}

float getVectorTheta(Solution* solution) {
	// Get the location of the centroid
	float x;
	float y;
	getCentroid(solution, x, y);

	// Calculate the angle formed by this vector: cos-1(x/Vmag)
	float cvTheta = acos(x/getCentroidVectorMag(solution));

	return cvTheta;
}

void runBSTuning(std::string graph_path, int m, FILE* dataFile) {
	printf("Evaluating %s\n", graph_path.c_str());
	// Create a solution and determine how many fast it can be competed, given m
	Solution solution(graph_path, m);
	fprintf(dataFile, "%d, %f, %f, %f, %f, %f\n", m, solution.m_fR, getCentroidVectorMag(&solution),
			getVectorTheta(&solution), solution.m_tBSTrajectory.mX,
			determineActualRunTime(graph_path.c_str(), m));
	fflush(dataFile);
}

// Run Min-SAR algorithm
int main(int argc, char** argv) {

	if(RUN == RUN_SINGLE) {
		printf("\n\n** Running Single Solver **\n\n");
		// Create a an algorithm and solver
		Short_Horizons shAlg(-25.500000, 13.000000, -28.088198, 7.993812, 12.035486);
		Solver solver(&shAlg);
		// Create a solution, test with m = 2
		Solution bestSolution(FILEPATH, 1);
		// Solve solution
		solver.SolvePathPlanning(&bestSolution);
		bestSolution.PrintSolution();
		printf("%.1f\n", bestSolution.TimeToRunSolution());
	}
	else if(RUN == RUN_NEXT_NEIGHBOR) {
    	// Run simplified gradient decent
		printf("\n\n** Running Simplified Gradient Decent **\n\n");

    	// Initial guesses -31.000000, 36.000000, 18.500000  -44.000000 32.000000 8.000000  -44.000000, 32.000000, 7.980000  -40.000000 36.000000 20.000000
		// -39.800003, 36.199997, 20.200001   -28.088198, 7.993812, 12.035486  -13.799992 7.399998 2.600000

		// Print results to file
		FILE * pOmegaWalkFile;
		char buff[100];
	    sprintf(buff, "Omega/omega_walk.dat");
	    printf("%s\n", buff);
	    pOmegaWalkFile = fopen(buff, "a");
	    // Disable buffering
	    setbuf(pOmegaWalkFile, NULL);

		float bestFound = 10000000000.0;
		for(int i = 0; i < 100; i++) {

			float o0 = -1.0 * (float)(rand()%50);
			float step_size0 = 4;
			float o1 = (float)(rand()%50);
			float step_size1 = 4;
			float o2 = (float)(rand()%50);
			float step_size2 = 4;
			bool run_multis = true;

			int lstmv = -1, lstlstmv = -1, lstlstlstmv = -1;
			bool cut_step = false;
			float lstbest = 1000000.0;

			// Ignore omega_i^prime from now -3.070001, 2.740000
			float o0_p = -3.070001;
			float o1_p = 2.740000;
			bool run_singles = false;

			while(true) {
				/// Take a single step in each direction

				std::priority_queue<step_pair, std::vector<step_pair>, std::greater<step_pair> > pq;

				// Best next neighbor
				if(run_singles) {
					// Omega' 0
					float o0p_sf = runTrainingSet(o0_p + step_size0, o1_p, o0, o1, o2, run_singles, run_multis);
					pq.push(std::make_pair(o0p_sf, 1));
					float o0p_sb = runTrainingSet(o0_p - step_size0, o1_p, o0, o1, o2, run_singles, run_multis);
					pq.push(std::make_pair(o0p_sb, 2));
					float o0p_b = (o0p_sb - o0p_sf);
					// Omega' 1
					float o1p_sf = runTrainingSet(o0_p, o1_p + step_size2, o0, o1, o2, run_singles, run_multis);
					pq.push(std::make_pair(o1p_sf, 3));
					float o1p_sb = runTrainingSet(o0_p, o1_p - step_size2, o0, o1, o2, run_singles, run_multis);
					pq.push(std::make_pair(o1p_sb, 4));
					float o1p_b = (o1p_sb - o1p_sf);
				}

				// Omega 0
				float o0_sf = runTrainingSet(o0_p, o1_p, o0 + step_size0, o1, o2, run_singles, run_multis);
				pq.push(std::make_pair(o0_sf, 5));
				float o0_sb = runTrainingSet(o0_p, o1_p, o0 - step_size0, o1, o2, run_singles, run_multis);
				pq.push(std::make_pair(o0_sb, 6));
				float o0_b = (o0_sb - o0_sf);
				// Omega 1
				float o1_sf = runTrainingSet(o0_p, o1_p, o0, o1 + step_size1, o2, run_singles, run_multis);
				pq.push(std::make_pair(o1_sf, 7));
				float o1_sb = runTrainingSet(o0_p, o1_p, o0, o1 - step_size1, o2, run_singles, run_multis);
				pq.push(std::make_pair(o1_sb, 8));
				float o1_b = (o1_sb - o1_sf);
				// Omega 2
				float o2_sf = runTrainingSet(o0_p, o1_p, o0, o1, o2 + step_size2, run_singles, run_multis);
				pq.push(std::make_pair(o2_sf, 9));
				float o2_sb = runTrainingSet(o0_p, o1_p, o0, o1, o2 - step_size2, run_singles, run_multis);
				pq.push(std::make_pair(o2_sb, 10));
				float o2_b = (o2_sb - o2_sf);

				// Look at which move gave the best result
				int best_move = pq.top().second;
				switch(best_move) {
				case 1:
					o0_p = o0_p + step_size0;
					break;
				case 2:
					o0_p = o0_p - step_size0;
					break;
				case 3:
					o1_p = o1_p + step_size2;
					break;
				case 4:
					o1_p = o1_p - step_size2;
					break;
				case 5:
					o0 = o0 + step_size0;
					break;
				case 6:
					o0 = o0 - step_size0;
					break;
				case 7:
					o1 = o1 + step_size1;
					break;
				case 8:
					o1 = o1 - step_size1;
					break;
				case 9:
					o2 = o2 + step_size2;
					break;
				case 10:
					o2 = o2 - step_size2;
					break;
				}

				// Record position
				float current_fitness = runTrainingSet(o0_p, o1_p, o0, o1, o2, run_singles, run_multis);
				fprintf(pOmegaWalkFile, "(%f, %f) (%f, %f, %f) : %f\n", o0_p, o1_p, o0, o1, o2, current_fitness);
				printf("(%f, %f) (%f, %f, %f) : %f\n", o0_p, o1_p, o0, o1, o2, current_fitness);
				if(step_size0 < 0.001) {
					printf("Step size has become too small: %f\n Stopping walk\n", step_size0);
					if(current_fitness <= bestFound) {
						bestFound = current_fitness;
					}
					break;
				}

				if(cut_step) {
					// Reduce step size
					printf("Reducing step size\n");
					step_size0 = step_size0/2.0;
					step_size2 = step_size1 = step_size0;

					lstmv = -1;
					lstlstmv = -1;
					lstbest = 100000.0;
					cut_step = false;
				}
				else if(current_fitness >= lstbest) {
					// We aren't making progress
					cut_step = true;
				}
				else {
					// Update progress
					lstbest = current_fitness;
				}

//				if((best_move == lstlstmv) && (best_move != lstmv)) {
//					// We aren't making progress..
//					if(current_fitness <= lstbest) {
//						// Reduce step size
//						printf("Reducing step size\n");
//						step_size0 = step_size0/2.0;
//						step_size2 = step_size1 = step_size0;
//
//						lstmv = -1;
//						lstlstmv = -1;
//						lstbest = 100000.0;
//					}
//					else {
//						// Let it run one more time, then update
//						lstlstmv = lstmv;
//						lstmv = best_move;
//						lstbest = current_fitness;
//					}
//				}
//				else {
//					// Making progress, nothing to update
//					lstlstlstmv = lstlstmv;
//					lstlstmv = lstmv;
//					lstmv = best_move;
//					lstbest = current_fitness;
//				}


	//			// Update omega_0,1,2 and primes
	//			if((fabs(o0p_b) > fabs(o1p_b)) && (fabs(o0p_b) > fabs(o0_b)) && (fabs(o0p_b) > fabs(o1_b)) && (fabs(o0p_b) > fabs(o2_b))) {
	//				o0_p += step_size0*(o0p_b/fabs(o0p_b));
	//			}
	//			else if((fabs(o1p_b) > fabs(o0p_b)) && (fabs(o1p_b) > fabs(o0_b)) && (fabs(o1p_b) > fabs(o1_b)) && (fabs(o1p_b) > fabs(o2_b))) {
	//				o1_p += step_size2*(o1p_b/fabs(o1p_b));
	//			}
	//			else if((fabs(o0_b) > fabs(o0p_b)) && (fabs(o0_b) > fabs(o1p_b)) && (fabs(o0_b) > fabs(o1_b)) && (fabs(o0_b) > fabs(o2_b))) {
	//				o0 += step_size0*(o0_b/fabs(o0_b));
	//			}
	//			else if((fabs(o1_b) > fabs(o0p_b)) && (fabs(o1_b) > fabs(o1p_b)) && (fabs(o1_b) > fabs(o0_b)) && (fabs(o1_b) > fabs(o2_b))) {
	//				o1 += step_size1*(o1_b/fabs(o1_b));
	//			}
	//			else if((fabs(o2_b) > fabs(o0p_b)) && (fabs(o2_b) > fabs(o1p_b)) && (fabs(o2_b) > fabs(o0_b)) && (fabs(o2_b) > fabs(o1_b))) {
	//				o2 += step_size2*(o2_b/fabs(o2_b));
	//			}
	//			else {
	//				break;
	//			}

				// Gradient approach
	//				// Omega 0
	//				float o0_sf = runTrainingSet(o0_p, o1_p, o0 + step_size0, o1, o2, run_singles, run_multis);
	//				float o0_sb = runTrainingSet(o0_p, o1_p, o0 - step_size0, o1, o2, run_singles, run_multis);
	//				float o0_b = -1*(o0_sf - o0_sb);
	//				// Omega 1
	//				float o1_sf = runTrainingSet(o0_p, o1_p, o0, o1 + step_size1, o2, run_singles, run_multis);
	//				float o1_sb = runTrainingSet(o0_p, o1_p, o0, o1 - step_size1, o2, run_singles, run_multis);
	//				float o1_b = -1*(o1_sf - o1_sb);
	//				// Omega 2
	//				float o2_sf = runTrainingSet(o0_p, o1_p, o0, o1, o2 + step_size2, run_singles, run_multis);
	//				float o2_sb = runTrainingSet(o0_p, o1_p, o0, o1, o2 - step_size2, run_singles, run_multis);
	//				float o2_b = -1*(o2_sf - o2_sb);
	//
	//				// Normalize the negative gradient vector
	//				float b_mag = sqrt(pow(o0_b, 2) + pow(o1_b, 2) + pow(o2_b, 2));
	//				float o0_b_norm = o0_b / b_mag;
	//				float o1_b_norm = o1_b / b_mag;
	//				float o2_b_norm = o2_b / b_mag;
	//
	//				// Update omega_0,1,2
	//				o0 = o0 + o0_b_norm * step_size0;
	//				o1 = o1 + o1_b_norm * step_size1;
	//				o2 = o2 + o2_b_norm * step_size2;
			}
		}
		fprintf(pOmegaWalkFile, "\nBest result: %f\n", bestFound);

		// Close the file
		fclose(pOmegaWalkFile);

	}
	else if(RUN == RUN_PROBING) {
    	// Run sweeping probe-search
		printf("\n\n** Running Sweeping Search-Space Probing **\n\n");

		// Print results to file
		FILE * pOmegaFile;
		char buff[100];
	    sprintf(buff, "Omega/omega6.dat");
	    printf("%s\n", buff);
	    pOmegaFile = fopen(buff, "a");

    	// Map the search space   -3.070001, 2.740000, -44.000000, 32.000000, 7.980000
	    //  -40.800007 33.200001 8.599999   -14.399991	7.599998	2.7
		float o0 = -19.2;
		float o1 = 7;
		float o2 = 0.00000;
		float o1_rest = 5.0;
		float o2_rest = o2;
		float step0 = 0.2;
		float step1 = 0.2;
		float step2 = 0.1;
		int iterations = 50;
		float best = 1000000000000000000.0;
		float best_0 = o0;
		float best_1 = o1;
		float best_2 = o2;

		for(int i = 0; i < iterations; i++, o0 += step0) {
			for(int j = 0; j < iterations; j++, o1 += step1) {
				for(int l = 0; l < 150; l++, o2 += step2) {
					// Run omega_0-2 on training set, get average time over baseline
					float avg = runTrainingSet(o0, o1, o0, o1, o2, false, true);

					// Record results
					fprintf(pOmegaFile, "%f %f %f %f\n", o0, o1, o2, avg);
					if(avg < best) {
						best = avg;
						best_0 = o0;
						best_1 = o1;
						best_2 = o2;
					}

					printf(" %f %f %f %f\n", o0, o1, o2, avg);
				}

//				// Run omega'_0,1 on training set, get average time over baseline
//				float avg = runTrainingSet(o0, o1, o0, o1, o2, true, false);
//
//				// Record results
//				fprintf(pOmegaFile, "%f %f %f\n", o0, o1, avg);
//				if(avg < best) {
//					best = avg;
//					best_0 = o0;
//					best_1 = o1;
//					best_2 = o2;
//
//				}
				o2 = o2_rest;
				printf("o1: %f\n", o1);
			}
			o1 = o1_rest;

			printf("o0: %f\n", o0);
		}

		printf("\nBest: %f at ( %f, %f, %f )\n", best, best_0, best_1, best_2);
		fprintf(pOmegaFile, "\n# Best: %f at ( %f, %f, %f )\n", best, best_0, best_1, best_2);
		fclose(pOmegaFile);
	}
	else if(RUN == RUN_BS_TUNING) {
		printf("\n\n** Running Base-Station Position Tuning **\n\n");

		// Print results to file
		FILE * pDataFile;
		char buff[100];
	    sprintf(buff, "Output/BS_data.csv");
	    printf("%s\n", buff);
	    pDataFile = fopen(buff, "a");

		fprintf(pDataFile, "m, R, |vector|, theta(vector), v_x, time\n");
		// Run on random plots
		runBSTuning("test_rand/plot_30_0.txt", 1, pDataFile);
	}
	else if(RUN == RUN_DATA_GEN) {
		Solution solution(FILEPATH, 2, 398.293274);
		solution.PrintAMPLData();
	}
	else if(RUN == RUN_MIP) {
		Iterative_MIP itMIP;
		Solver solver(&itMIP);
		Solution solution(FILEPATH, 2, 268.478333);
		solver.SolvePathPlanning(&solution);
		solution.PrintSolution();
	}
	else if(RUN == RUN_LKH_DATA) {
		Solution solution(FILEPATH);
//		solution.PrintLKHData();
		solution.PrintLKHDataFHPP(21, 12);

		printf("Running LKH\n");
		std::system("./LKH-3.0.6/LKH FixedHPP.par");

		printf("Found the following solution:\n");
		// Open file with results
		std::ifstream file("LKH_output.dat");
		// Remove the first few lines...
		std::string line;
		for(int i = 0; i < 6; i++) {
			std::getline(file, line);
		}

		// Start parsing the data
		for(auto i = 0; i < solution.m_nN; i++) {
			std::getline(file, line);
			std::stringstream lineStreamN(line);
			// Parse the way-point from the line
			int n;
			lineStreamN >> n;
			printf(" %d", n-1);
		}
		printf("\n");
		file.close();
	}
}
