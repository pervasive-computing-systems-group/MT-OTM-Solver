/*
 * UDEdge.h
 *
 * Created by:	Jonathan Diller
 * On: 			May 17, 2022
 *
 * Description: Undirected edge class
 */

#pragma once

#include "Edge.h"
#include "Vertex.h"
#include "graph_defines.h"

class UDEdge : public Edge {
public:
	UDEdge(const Vertex* pAVertex, const Vertex* pBVertex);
	~UDEdge();

	/*
	 * Returns the weight of this edge, defined here as the Euclidean distance
	 * between two vertices plus some added constant lambda. For many edges, lambda
	 * will be 0.
	 */
	virtual float getWeight();

	// Vertex pointers incident to this edge
	const Vertex* p_oAVertex;
	const Vertex* p_oBVertex;

private:
};
