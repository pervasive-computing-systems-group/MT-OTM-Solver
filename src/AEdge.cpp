#include "AEdge.h"

AEdge::AEdge(float fWeight) {
	m_fWeight = fWeight;
}

AEdge::~AEdge() {

}

/*
 * float getWeight() : weight functions
 *
 * Returns the weight of this edge, defined here as the Euclidean distance
 * between two vertices plus some added constant lambda. For many edges, lambda
 * will be 0.
 */
float AEdge::getWeight() {
	return m_fWeight;
}
