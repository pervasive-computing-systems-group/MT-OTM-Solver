/*
 * VertexOnEdge.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 02, 2021
 *
 * Description: Vertex-on-edge class. This vertex type represents an underlying
 * edge that is part of another, related graph.
 */

#pragma once

#include "graph_defines.h"
#include "Edge.h"
#include "DEdge.h"
#include "Vertex.h"
#include <vector>


class VertexOnEdge : public Vertex {
public:
	VertexOnEdge();
	VertexOnEdge(Edge* pUnderlyingEdge);
	/*
	 * Constructor that creates a source vertex and makes dest a destination vertex.
	 * It does this by splitting away the adjacency list from dest, leaving dest
	 * with no out-bound edges and giving them to the this vertex.
	 */
	VertexOnEdge(Edge* pUnderlyingEdge, VertexOnEdge* dest);
	~VertexOnEdge();

	/*
	 * Add directed edge going out of this vertex
	 */
	void addDEdgeOut(int id, VertexOnEdge* v_to, float nWeight);

	/*
	 * Clear all edges leaving this vertex
	 */
	void clearEdgeList();

	/*
	 * Add the underlying edge for the vertex
	 */
	void addUnderlyingEdge(Edge* pUnderlyingEdge);

	/*
	 * Get the list of directed edges going out of this vertex
	 */
	const std::vector<T_BipartitEdge>& getEdgesList();

	Edge* m_pUnderlyingEdge;
private:
	std::vector<T_BipartitEdge> m_vOutEdges;
};
