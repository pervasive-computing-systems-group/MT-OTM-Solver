/*
 * K_TSP_Shft.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov 4, 2023
 *
 * Description: Solves moving-MdMtHP Problems by: k-mean clustering waypoints,
 * guessing sub-tour times using min-spanning trees, solves sub TSP on sub-tours,
 * then shifts the depot/terminal to reduce subtour time
 */

#pragma once

#include <stdlib.h>
#include <sstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <time.h>

#include "gurobi_c++.h"

#include "defines.h"
#include "Solution.h"
#include "Hybrid_Planner.h"
#include "Graph_Theory_Algorithms.h"


#define DEBUG_K_TSP_SHFT	1 || DEBUG

struct KMPoint {
	double x, y;
	int cluster;
	int vID;

	KMPoint(double x, double y) : x(x), y(y), cluster(-1), vID(-1) {}
	KMPoint(Vertex* vrtx) : x(vrtx->fX), y(vrtx->fY), cluster(-1), vID(vrtx->nID) {}
	KMPoint(const KMPoint& pnt) : x(pnt.x), y(pnt.y), cluster(pnt.cluster), vID(pnt.vID) {}

	/**
	* Computes the (square) Euclidean distance between this point and another
	*/
	double distance(KMPoint& p) {
		return sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
	}
};

class K_TSP_Shft : public Hybrid_Planner {
public:
	K_TSP_Shft();
	virtual ~K_TSP_Shft();

protected:
	/*
	 * Runs the K-means -> TSP -> Adjusting algorithm
	 */
	void RunAlgorithm(Solution* pathSolution);

private:
	void kMeansClustering(std::vector<KMPoint>* points, std::vector<KMPoint>* centroids, int epochs);
	// Run LKH TSP solver on the give cluster of vertices. Store ordered results in sub_tour
	void runLKH_TSP(Solution* solution, std::vector<Vertex*>* cluster, std::vector<Vertex*>* sub_tour);
	/*
	 * Determines what time each sub-tour should begin using quadratic programming (Gurobi) - problem is a
	 * sum-of-squares. Store result in selected_times, tries to keep these as close as possible to the
	 * desired_times while constrained by tour_times.
	 */
	void solveStartTimesQP(std::vector<double>* tour_times, std::vector<double>* desired_times, std::vector<double>* selected_times);
};
