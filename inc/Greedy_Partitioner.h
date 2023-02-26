/*
 * Greedy_Partitioner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 05, 2021
 *
 * Description:
 */

#pragma once

#include "Partitioner.h"

#define DEBUG_GREEDY	0 && DEBUG


class Greedy_Partitioner : public Partitioner{
public:
	Greedy_Partitioner();
	~Greedy_Partitioner();

protected:
	/*
	 * Run a greedy partitioner that slowly expands the partitioned space by randomly picking
	 * a vertex from the set of all closest vertices
	 */
	void RunAlgorithm(Solution* solution);

private:
};
