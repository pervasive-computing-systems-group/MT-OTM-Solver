#include "UDEdge.h"

UDEdge::UDEdge(const Vertex* pAVertex, const Vertex* pBVertex)
	: p_oAVertex(pAVertex), p_oBVertex(pBVertex)
{
}

UDEdge::~UDEdge() {

}

/*
 * float getWeight() : weight functions
 *
 * Returns the weight of this edge, defined here as the Euclidean distance
 * between two vertices
 */
float UDEdge::getWeight() {
	float a, b;
	a = p_oBVertex->fX - p_oAVertex->fX;
	b = p_oBVertex->fY - p_oAVertex->fY;
	float ret_val = (float)sqrt(a*a + b*b);

	return ret_val;
}
