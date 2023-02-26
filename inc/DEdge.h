/*
 * DEdge.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 01, 2021
 *
 * Description: Directed edge class
 */

#pragma once

#include "Edge.h"
#include "Vertex.h"
#include "graph_defines.h"

class DEdge : public Edge {
public:
	DEdge(const Vertex* pFromVertex, const Vertex* pToVertex);
	~DEdge();


	/*
	 * Returns the weight of this edge, defined here as the Euclidean distance
	 * between two vertices plus some added constant lambda. For many edges, lambda
	 * will be 0.
	 */
	virtual float getWeight();

	/*
	 * Set additional weight constant lambda
	 */
	void setLambda(float fLambda);

	// Vertex pointers incident to this edge
	const Vertex* p_oFromVertex;
	const Vertex* p_oToVertex;

private:
	// Additional weight constant lambda
	float m_fLambda;
};
