/*
 * KMeans_Partitioner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Feb 12, 2022
 *
 * Description: This class runs Lloyd's Algorithm for k-means clustering to form partitions.
 * The algorithm uses the centroids of the depot/terminal pairs as the initial partition centroids.
 * It only runs for a set number of iterations as opposed to running until the clusters settle.
 * This is meant to prevent the algorithm from shifting the partitions too far away from the
 * depot/terminal centroids.
 *
 * Thanks to Robert Andrew Martin for giving a great explanation on Lloyd’s algorithm and for making
 * his code publicly available. This implementation is a modification of his work.
 *
 * https://reasonabledeviations.com/2019/10/02/k-means-in-cpp/
 *
 */

#pragma once

#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <limits>

#include "Partitioner.h"

#define DEBUG_KMEANS		0 || DEBUG

#define KMEANS_ITERATIONS	100

struct KPoint {
	double x, y;
	int cluster;
	bool locked;
	double minDist;
	Vertex* v;

	KPoint(double x, double y) : x(x), y(y), cluster(-1), locked(false), minDist(__DBL_MAX__), v(NULL) {}
	KPoint(double x, double y, Vertex* vrtx) : x(x), y(y), cluster(-1), locked(false), minDist(__DBL_MAX__), v(vrtx) {}

	/**
	* Computes the (square) Euclidean distance between this point and another
	*/
	double distance(KPoint p) {
		double dist = sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
		return dist;
	}
};


class KMeans_Partitioner : public Partitioner{
public:
	KMeans_Partitioner();
	~KMeans_Partitioner();

protected:
	/*
	 *
	 */
	void RunAlgorithm(Solution* solution);

private:
	/*
	 * Performs Lloyd's k-means clustering algorithm on points using centroids as the initial centroids. The
	 * algorithm runs for epocks iterations.
	 */
	void kMeansClustering(std::vector<KPoint>* points, std::vector<KPoint>* centroids, int epochs);
};
