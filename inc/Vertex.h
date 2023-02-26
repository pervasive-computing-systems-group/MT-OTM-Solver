/*
 * Vertex.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 01, 2021
 *
 * Description: Vertex class, used to build graphs.
 */

#pragma once

#include "graph_defines.h"

class Vertex {
public:
	Vertex();
	Vertex(const Vertex &u);
	Vertex(int id, float x, float y, E_VertexType vType);
	~Vertex();

	int nID;
	float fX, fY;
	E_VertexType eVType;

	// Returns the distance (in meters) from this vertex to v
	float GetDistanceTo(Vertex* v);
private:
};
