/*
 * Generic_Partitioner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 30, 2021
 *
 * Description:
 */

#pragma once

#include "Partitioner.h"


class Generic_Partitioner : public Partitioner{
public:
	Generic_Partitioner();
	~Generic_Partitioner();

protected:
	/*
	 * Creates a very basic partition
	 */
	void RunAlgorithm(Solution* solution);

private:
};
