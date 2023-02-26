/*
 * Sweeping_Partitioner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 04, 2021
 *
 * Description:
 */

#pragma once

#include <math.h>

#include "Partitioner.h"
#include "Sweeping_Approach.h"


#define PI		3.14159265


class Sweeping_Partitioner : public Partitioner, public Sweeping_Approach {
public:
	Sweeping_Partitioner();
	~Sweeping_Partitioner();

protected:
	/*
	 * Creates a very basic partition
	 */
	void RunAlgorithm(Solution* solution);

private:
};
