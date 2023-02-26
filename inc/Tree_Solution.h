/*
 * Tree_Solution.h
 *
 * Created by:	Jonathan Diller
 * On: 			Nov 04, 2021
 *
 * Description:
 */

#pragma once

#include "Solution.h"

class Tree_Solution : public Solution {
public:
	Tree_Solution(std::string gaph_path, int m);
	~Tree_Solution();

	/*
	 * Determine the amount of time required for a UAV to run this solution
	 */
	float TimeToRunSolution(bool bLagrangianRelaxation = false) override;
	// Return the distance (in meters) of partition n of this solution
	float DistanceOfPartition(int n) override;
	// Returns the distance of the maximum leg in this solution
	float GetMaxLegDist() override;
private:
	// Determines the minimum time to run this solution given m partitions
	float getMinTreeLength();
};
