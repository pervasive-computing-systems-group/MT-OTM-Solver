/*
 * DEdge.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 02, 2021
 *
 * Description: Artificial edge class. These edges are used to add additional
 * elements to matroids.
 */

#pragma once

#include "Edge.h"
#include "graph_defines.h"

class AEdge : public Edge {
public:
	AEdge(float fWeight);
	~AEdge();

	/*
	 * Returns the weight of this edge, defined here as the Euclidean distance
	 * between two vertices plus some added constant lambda. For many edges, lambda
	 * will be 0.
	 */
	virtual float getWeight();

private:
	// Additional weight constant lambda
	float m_fWeight;
};
