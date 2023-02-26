/*
 * Tree_Solver.h
 *
 * Created by:	Jonathan Diller
 * On: 			June 08, 2022
 *
 * Description: Base-class for algorithms that create a tree of the entire space
 * then use that tree to find sub-tours.
 */

#pragma once

#include <stdlib.h>
#include <math.h>
#include <stack>
#include <vector>
#include <list>

#include "defines.h"
#include "Solution.h"
#include "Graph_Theory_Algorithms.h"
#include "UDEdge.h"
#include "ShortCutTree.h"
#include "Roots.h"

#define TS_DEBUG		0 || DEBUG

class Tree_Solver {
public:
	Tree_Solver();
	virtual ~Tree_Solver();

protected:
	// Determine the distance of the given sub-tour
	float getSubtourDist(std::list<Vertex*> &subtour);
	// Determine the distance of the given sub-tour
	void determineTerminalLocation(std::list<Vertex*> &subTour, float fV_b, float fV_u, Vertex* terminal);
	// Determines if c is relatively close to zero. Avoids comparing floating-point numbers to zero
	bool isZero(double c);

private:
};
