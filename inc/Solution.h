/*
 * Solution.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 21, 2021
 *
 * Description:
 */

#pragma once

#include <map>
#include <vector>
#include <sstream>
#include <fstream>
#include <limits>
#include <list>
#include <algorithm>

#include "defines.h"
#include "Roots.h"
#include "Vertex.h"
#include "Graph_Theory_Algorithms.h"
#include "UGVTrajectory.h"

// Allows for solution specific debugging
#define DEBUG_SOLUTION	1 || DEBUG
#define PRINT_SOLUTION	0 || DEBUG
// Allowable sub-tour distance tolerance
#define ST_DIST_TOLERANCE		1


class Solution {
public:
	Solution();
	Solution(std::string graph_path);
	Solution(std::string graph_path, int m, float ct_guess = -1.0);
	Solution(std::string graph_path, int m, float ct_guess, float* A);
	Solution(Solution* cp_solution, float ct, float* A);
	virtual ~Solution();

	// Build complete solution
	bool BuildCompleteSolution(std::vector<std::list<Vertex*>>& tours, std::vector<float>& speeds);
	// Prints out the adjacency matrix contained in this solution (or nothing if adjacency matrix was not set-up)
	void PrintSolution();
	// Prints out the input graph for associated with this solution
	void PrintGraph();
	// Prints out the data to be used in an AMPL model. This will look like a large 2D array of absolute distances
	void PrintAMPLData();
	// Prints out the data to be used in the TSPLIB file format for the LKH solver
	void PrintLKHData();
	// Prints out the data to be used in the TSPLIB file format for the LKH solver for the vertices in lst
	void PrintLKHData(std::vector<Vertex*> &lst);
	// Prints out the data to be used in the TSPLIB file format for the LKH solver where the weight between a and b is 0
	void PrintLKHDataFHPP(int a, int b);
	// Clears the adjacency matrix (resets all to false) (if adjacency matrix was created)
	void ClearAdjacencyMatrix();
	/*
	 * Determine the amount of time required for a UAV to run this solution.
	 *
	 * Setting bLagrangianRelaxation to true will cause the algorithm to use Lagrangian relaxation
	 * to relax the max distance constraint. In this case, any partition leg longer than DIST_MAX
	 * will be run at the optimal velocity (V_OPT) with an additional time penalty based on the
	 * distance that the individual leg over-shot DIST_MAX.
	 */
	virtual float TimeToRunSolution(E_VelocityFlag fixedVelocityFlag = E_VelocityFlag::e_NotFixed, bool bLagrangianRelaxation = false);
	// Return the distance (in meters) of partition n of this solution
	virtual float DistanceOfPartition(int n);
	// Returns the distance of the maximum leg in this solution
	virtual float GetMaxLegDist();
	// Returns the distance of the given tour of vertices found in lst
	float GetTourDist(std::list<Vertex*> &lst);
	// Return a pointer to the depot vertex of partition p
	Vertex* GetDepotOfPartion(int p);
	// Return a pointer to the terminal vertex of partition p
	Vertex* GetTerminalOfPartion(int p);
	// Determines if this solution is feasible given n, m, and radios R
	bool IsFeasible();
	/*
	 * Returns the distance of a minimum spanning constrained forest containing m trees. The forest
	 * must connect every depot and terminal to at least one destination vertex. Note that this
	 * minimum constrained forest is less constrained compared to the minimum constrained forest
	 * found using Bae-Rathinam's algorithm and provides a lower bound for any solution but will not
	 * be as tight of a lower bound as the forest found using Bae-Rathinam's algorithm.
	 */
	float GetMinSpanningForestDistance(bool findForest = false);
	/*
	 * Returns the distance of a minimum spanning forest containing m trees. This should be used on
	 * random graph-types!
	 */
	float GetMinSpanningForestDistanceRND();
	// Returns the baseline time to run through the minimum spanning constrained forest
	float GetMinSpanningForestRT(bool findForest = false);
	/*
	 * Makes the solution consistent with the assigned partitions by adjusting the positions of the
	 * depots/terminals. Returns false if the solution hasn't been solved yet or if it determines
	 * that a sub-tour is longer than DIST_MAX (and does not adjust anything). There is no guarantee
	 * that the sub-tours meet max UAV traveling constraints!
	 */
	bool CorrectSolution(E_VelocityFlag velocityFlag=E_VelocityFlag::e_NotFixed);
	// Does the math to determine the maximum velocity the UAV can go for distance dist
	double GetMaxVelocity(double dist, E_VelocityFlag velocityFlag=E_VelocityFlag::e_NotFixed);

	// Getters
	bool isSetup() {return m_bSetup;}

	Vertex* m_pVertexData;
	bool** m_pAdjMatrix;
	std::map<int, std::vector<Vertex*>> m_mPartitions;
	std::vector<float> m_vSpeeds;
	int m_nNumVertices;
	int m_nN;
	int m_nM;
	float m_fR;
	bool m_bHasTree;
	bool m_bPartitioned;
	bool m_bSolved;
	bool m_bFeasible;
	UGVTrajectory m_tBSTrajectory;
	// Variables for time
	std::vector<double> m_Td_k;
	std::vector<double> m_Tt_k;

protected:
	bool m_bSetup;
	bool m_bInheritedSpeeds;
private:
	/*
	 * Determine the location of the terminal using sub-tour subTour and UAV velocity magnitude fV_u.
	 *
	 * This function assumes that the first vertex on the the list is the depot of the sub-tour. We
	 * also assume that the base station's velocity vector only points in the positive x-direction
	 * (i.e. y_b = 0).
	 */
	void determineTerminalLocation(std::list<Vertex*> &subTour, float fV_u, Vertex* terminal);
	// Compares A and B to determine if they are within some tolerance of each other
	bool compareFloatsAB(float A, float B);
};
