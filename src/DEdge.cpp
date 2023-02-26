#include "DEdge.h"

DEdge::DEdge(const Vertex* pFromVertex, const Vertex* pToVertex)
	: p_oFromVertex(pFromVertex), p_oToVertex(pToVertex)
{
	m_fLambda = 0;
}

DEdge::~DEdge() {

}

/*
 * float getWeight() : weight functions
 *
 * Returns the weight of this edge, defined here as the Euclidean distance
 * between two vertices plus some added constant lambda. For many edges, lambda
 * will be 0.
 */
float DEdge::getWeight() {
	float a, b, ret_val = m_fLambda;
	a = p_oToVertex->fX - p_oFromVertex->fX;
	b = p_oToVertex->fY - p_oFromVertex->fY;
	ret_val += (float)sqrt(a*a + b*b);

	return ret_val;
}

// Set additional weight constant
void DEdge::setLambda(float fLambda) {
	m_fLambda = fLambda;
}
