/*
 * Graph_Theory_Algorithms.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 22, 2021
 *
 * Description:
 */

#pragma once

#include <vector>
#include <map>
#include <list>
#include <limits>
#include <queue>

#include "graph_defines.h"
#include "Vertex.h"
#include "UDEdge.h"

#define GRAPH_DEBUG		0

class Graph_Theory_Algorithms {
public:
	Graph_Theory_Algorithms();
	~Graph_Theory_Algorithms();

	// Runs DFS to find a path from edge u to edge v in graph g (an n x n adjacency matrix)
	static bool DFS_AdjMtrx(bool** g, int g_size, int u, int v);
	// Runs DFS to find neighborhood of vertex v in an undirected graph defined on an adjacency matrix
	static void DFS_Neighborhood_AdjMtrx(bool** g, bool* nbrhoodList, int g_size, int v);
	//Bellman-Ford algorithm to find shortest paths from vertex s using maps. Returns true if the graph contains a negative weight cycle.
	static bool Bellman_Ford_Maps(std::vector<T_BipartitEdge>& edgeList, VertexOnEdge* s,
			std::map<VertexOnEdge*, float>& costMap, std::map<VertexOnEdge*, VertexOnEdge*>& parentMap);
	// Runs recursive implementation of DFS to find shortest path from vertex u to vertex v in an undirected graph g
	static bool DFS_TopAdjMtrx(bool** g, int* parentList, int g_size, int u, int v);
	// Finds the distance of a Min-Spanning-Tree using Prim's Algorithm on complete graphs
	static float MST_Prims(std::vector<Vertex*>& V);
	// Finds the distance of a Min-Spanning-Tree using Prim's Algorithm on complete graphs with n vertices
	static float MST_Prims(Vertex* V, int n);
	// Finds Min-Spanning-Tree using Prim's Algorithm on complete graphs with n vertices, stores solution in results, returns tree length
	static float MST_Prims(Vertex* V, int n, std::list<UDEdge> &result);
	// Finds the distance of a Min-Spanning-Forest of m trees using Prim's Algorithm on complete graphs with n vertices
	static float MSF_Prims(Vertex* V, int n, int m);
	// Performs the Hungarian Algorithm on cost matrix A to find a minimum cost matching, returns minimum cost
	//  and stores answer in result
	static int Hungarian_Algorithm(std::vector<std::vector<int>>& A, std::vector<std::pair<int, int>>& result);

private:
	// Runs recursive implementation of DFS to determine if a path exists
	static bool runDFS_AdjMtrx(bool** g, bool* visitedList, int g_size, int u, int v);
};
