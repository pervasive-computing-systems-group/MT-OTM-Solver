#include "VertexOnEdge.h"

// Default constructor
VertexOnEdge::VertexOnEdge() :
	m_pUnderlyingEdge(NULL)
{ }

// Basic constructor
VertexOnEdge::VertexOnEdge(Edge* pUnderlyingEdge) :
	m_pUnderlyingEdge(pUnderlyingEdge)
{ }

// Constructor that creates a source vertex and makes dest a destination vertex.
VertexOnEdge::VertexOnEdge(Edge* pUnderlyingEdge, VertexOnEdge* dest) :
	m_pUnderlyingEdge(pUnderlyingEdge), m_vOutEdges(dest->m_vOutEdges)
{
	dest->m_vOutEdges.clear();

	for(auto edge = m_vOutEdges.begin() ; edge != m_vOutEdges.end(); ++edge) {
		edge->pv_from = this;
	}
}

// Destructor
VertexOnEdge::~VertexOnEdge() {

}


/*
 * Add directed edge going out of this vertex
 */
void VertexOnEdge::addDEdgeOut(int id, VertexOnEdge* v_to, float nWeight) {
	// Create bipartite edge
	T_BipartitEdge e;
	e.ID = id;
	e.nWeight = nWeight;
	e.pv_from = this;
	e.pv_to = v_to;

	// Store edge in map of out-going edges
	m_vOutEdges.push_back(e);
}

/*
 * Clear all edges leaving this vertex
 */
void VertexOnEdge::clearEdgeList() {
	m_vOutEdges.clear();
}

/*
 * Add the underlying edge for the vertex
 */
void VertexOnEdge::addUnderlyingEdge(Edge* pUnderlyingEdge) {
	m_pUnderlyingEdge = pUnderlyingEdge;
}

/*
 * Get the list of directed edges going out of this vertex
 */
const std::vector<T_BipartitEdge>& VertexOnEdge::getEdgesList() {
	return m_vOutEdges;
}
