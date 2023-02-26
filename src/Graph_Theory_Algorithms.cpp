#include "Graph_Theory_Algorithms.h"

Graph_Theory_Algorithms::Graph_Theory_Algorithms()
{
}

Graph_Theory_Algorithms::~Graph_Theory_Algorithms() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * bool DFS_AdjMtrx() : Runs DFS to find a path from edge u to edge v in graph g
 *
 * This expects an n x n adjacency matrix for an undirected graph g where n = g_size.
 * Note that this graph can have both edges (v, u) and (u, v), which are treated as
 * unique, undirected edges. In other words, the algorithm checks for adjacent
 * vertices on both the top and bottom of the adjacency matrix.
 */
bool Graph_Theory_Algorithms::DFS_AdjMtrx(bool** g, int g_size, int u, int v) {
	// Track visited vertices
	bool* visitedList = new bool[g_size];
	for(int i = 0; i < g_size; i++) {
		visitedList[i] = false;
	}

	// Run DFS on graph
	bool path_exists = runDFS_AdjMtrx(g, visitedList, g_size, u, v);

	// Cleanup memory
	delete[] visitedList;

	return path_exists;
}

/*
 * bool runDFS_Neighborhood_AdjMtrx() : Runs recursive implementation of DFS to find neighborhood of
 * vertex v in an undirected graph defined on an adjacency matrix. Stores neighbors in nbrHoodList.
 *
 * This expects an n x n adjacency matrix for an undirected graph g where n = g_size.
 * Note that this graph can have both edges (v, u) and (u, v), which are treated as
 * unique, undirected edges. In other words, the algorithm checks for adjacent
 * vertices on both the top and bottom of the adjacency matrix.
 */
void Graph_Theory_Algorithms::DFS_Neighborhood_AdjMtrx(bool** g, bool* nbrhoodList, int g_size, int v) {
	nbrhoodList[v] = true;
	for(int i = 0; i < g_size; i++) {
		if(g[v][i]) {
			if(!nbrhoodList[i]) {
				DFS_Neighborhood_AdjMtrx(g, nbrhoodList, g_size, i);
			}
		}
	}

	for(int i = 0; i < g_size; i++) {
		if(g[i][v]) {
			if(!nbrhoodList[i]) {
				DFS_Neighborhood_AdjMtrx(g, nbrhoodList, g_size, i);
			}
		}
	}
}

/*
 * Bellman-Ford algorithm to find shortest paths from vertex s using maps. Returns true if
 * the graph contains a negative weight cycle.
 *
 * This implementation of the Bellman-Ford algorithm uses maps where the vertex pointer is
 * the key and the cost/parent of the vertex is the value. This allows the algorithm to run
 * without having to create a new graph where each vertex has an id in {0, 1, ... , n} but
 * also means that the algorithm will run in O(|V||E|log(|V|)) (not |V||E|).
 *
 * We don't assume that the maps have already been configured (i.e. we reset the costs to
 * INF and the parents to NULL)
 *
 * TODO: This isn't efficient, fix it so that it runs in O(|V||E|)
 */
bool Graph_Theory_Algorithms::Bellman_Ford_Maps(std::vector<T_BipartitEdge>& edgeList, VertexOnEdge* s, std::map<VertexOnEdge*, float>& costMap,
		std::map<VertexOnEdge*, VertexOnEdge*>& parentMap) {
	// Set c[v] := INF, for-all v
	for(auto v = costMap.begin(); v != costMap.end(); v++) {
		v->second = std::numeric_limits<float>::max();
	}

	// Set p[v] := NIL, for-all v
	for(auto v = parentMap.begin(); v != parentMap.end(); v++) {
		v->second = NULL;
	}

	// Cost from the source to the source is 0
	costMap.at(s) = 0;

	// Run |V| - 1 times
	for(uint i = 0; i < (uint)(costMap.size() - 1); i++) {
		// for-each edge (u, v)
		for(T_BipartitEdge e : edgeList) {
			// Verify that the path to u isn't INF
			if(costMap.at(e.pv_from) < (std::numeric_limits<float>::max() - 1)) {
				// Check if reaching v from u is cheaper than the previous cost to get to v
				float c_v_from_u = (costMap.at(e.pv_from) + e.nWeight);
				if(c_v_from_u < costMap.at(e.pv_to)) {
					// Found cheaper route to v, update cost and parent
					costMap.at(e.pv_to) = c_v_from_u;
					parentMap.at(e.pv_to) = e.pv_from;
				}
			}
		}
	}

	// Check for negative cycles
	bool neg_cycle = false;

	// for-each edge (u, v)
	for(T_BipartitEdge e : edgeList) {
		// Check if it is still cheaper to get to v from u
		if(costMap.at(e.pv_to) > (costMap.at(e.pv_from) + e.nWeight)) {
			// Then we have a negative cycle
			neg_cycle = true;
			break;
		}
	}

	return neg_cycle;
}

/*
 * bool runDFS_AdjMtrx() : runs recursive implementation of DFS to find shortest path
 * from vertex u to vertex v in an undirected graph g. It stores the parents of each
 * vertex in the parentList array. This function assumes that the parent list has already
 * been initialized to '-1' for parent entries and that the adjacency matrix is filled
 * out on the top diagonal only. Edge entries on the bottom half of the matrix are ignored.
 * We also assume that the parent of u is set to u (ou qualquer opção)
 */
bool Graph_Theory_Algorithms::DFS_TopAdjMtrx(bool** g, int* parentList, int g_size, int u, int v) {
	// base case
	if(u == v) {
		return true;
	}
	else {
		// Recursive step
		for(int i = 0; i < g_size; i++) {
			int a, b;
			// Only search top half of matrix
			if(i < u) {
				a = i;
				b = u;
			}
			else {
				a = u;
				b = i;
			}

			if(g[a][b]) {
				if(parentList[i] == -1) {
					// Discovered new vertex
					parentList[i] = u;

					// Call DFS on i
					if(DFS_TopAdjMtrx(g, parentList, g_size, i, v)) {
						// Found destination, return true
						return true;
					}
				}
			}
		}
	}

	// If we make it this far, we didn't find our target
	return false;
}

// Finds the distance of a Min-Spanning-Tree using Prim's Algorithm
float Graph_Theory_Algorithms::MST_Prims(Vertex* V, int n) {
	if(GRAPH_DEBUG) {
		printf("Running Prim's Algorithm, n = %d\n", n);
	}

	float dist = 0;

	// Create two lists to hold the vertices
	std::list<Vertex*> t_list;
	std::list<Vertex*> g_list;

	// Add all vertices to graph-list
	for(int i = 0; i < n; i++) {
		g_list.push_back(V + i);
	}

	// To start, add a "random" vertex to the tree
	t_list.push_back(g_list.front());
	g_list.pop_front();

	if(GRAPH_DEBUG) {
		printf("Ready to start, |g-list| = %ld, |t-list| = %ld\n", g_list.size(), t_list.size());
	}

	// Run until there are not more vertices in graph-list
	while(!g_list.empty()) {
		// Find shortest edge between tree-list and graph-list
		float shortest_leg = std::numeric_limits<float>::max();
		std::list<Vertex*>::iterator t_it = t_list.begin(), best_v = g_list.begin();
		for (; t_it != t_list.end(); ++t_it) {
			std::list<Vertex*>::iterator g_it = g_list.begin();
			for (; g_it != g_list.end(); ++g_it) {
				float dist_t_to_g = (*t_it)->GetDistanceTo(*g_it);
				if(dist_t_to_g < shortest_leg) {
					shortest_leg = dist_t_to_g;
					best_v = g_it;
				}
			}
		}

		// Sanity Print
		if(GRAPH_DEBUG) {
			printf("Moving %d into tree\n", (*best_v)->nID);
		}

		// Move the next closest vertex into the tree, remove from graph
		Vertex* move_to_tree = *best_v;
		dist += shortest_leg;
		g_list.erase(best_v);
		t_list.push_back(move_to_tree);
	}

	// Sanity Print
	if(GRAPH_DEBUG) {
		printf("Found MST distance of %f\n", dist);
	}

	return dist;
}

// Finds Min-Spanning-Tree using Prim's Algorithm on complete graphs with n vertices, stores solution in results, returns tree length
float Graph_Theory_Algorithms::MST_Prims(Vertex* V, int n, std::list<UDEdge> &result) {
	if(GRAPH_DEBUG) {
		printf("Running Prim's Algorithm, n = %d\n", n);
	}

	float dist = 0;

	// Create two lists to hold the vertices
	std::list<Vertex*> t_list;
	std::list<Vertex*> g_list;

	// Add all vertices to graph-list
	for(int i = 0; i < n; i++) {
		g_list.push_back(V + i);
	}

	// To start, add a "random" vertex to the tree
	t_list.push_back(g_list.front());
	g_list.pop_front();

	if(GRAPH_DEBUG) {
		printf("Ready to start, |g-list| = %ld, |t-list| = %ld\n", g_list.size(), t_list.size());
	}

	// Run until there are not more vertices in graph-list
	while(!g_list.empty()) {
		// Find shortest edge between tree-list and graph-list
		float shortest_leg = std::numeric_limits<float>::max();
		std::list<Vertex*>::iterator best_a = t_list.begin(), best_b = g_list.begin();
		for(std::list<Vertex*>::iterator t_it = t_list.begin(); t_it != t_list.end(); ++t_it) {
			for(std::list<Vertex*>::iterator g_it = g_list.begin(); g_it != g_list.end(); ++g_it) {
				float dist_t_to_g = (*t_it)->GetDistanceTo(*g_it);
				if(dist_t_to_g < shortest_leg) {
					shortest_leg = dist_t_to_g;
					best_a = t_it;
					best_b = g_it;
				}
			}
		}

		// Sanity Print
		if(GRAPH_DEBUG) {
			printf("Moving %d into tree\n", (*best_b)->nID);
		}
		Vertex* vertexA = *best_a;
		Vertex* vertexB = *best_b;

		// Add to distance
		dist += shortest_leg;
		// Move the next closest vertex into the tree, remove from graph
		t_list.push_back(vertexB);
		g_list.erase(best_b);
		// Add this edge to results
		result.push_back(UDEdge(vertexA, vertexB));
	}

	// Sanity Print
	if(GRAPH_DEBUG) {
		printf("Found MST distance of %f\n", dist);
	}

	return dist;
}

// Finds the distance of a Min-Spanning-Forest of m trees using Prim's Algorithm on complete graphs with n vertices
float Graph_Theory_Algorithms::MSF_Prims(Vertex* V, int n, int m) {
	if(GRAPH_DEBUG) {
		printf("Running Prim's Algorithm, n = %d\n", n);
	}

	float dist = 0;

	// Create two lists to hold the vertices
	std::list<Vertex*> t_list;
	std::list<Vertex*> g_list;

	// Create a max-heap to hold the longest legs of the tree
	std::priority_queue<float> edgeMaxHeap;

	// Add all vertices to graph-list
	for(int i = 0; i < n; i++) {
		g_list.push_back(V + i);
	}

	// To start, add a random vertex to the tree
	t_list.push_back(g_list.front());
	g_list.pop_front();

	if(GRAPH_DEBUG) {
		printf("Ready to start, |g-list| = %ld, |t-list| = %ld\n", g_list.size(), t_list.size());
	}

	// Run until there are not more vertices in graph-list
	while(!g_list.empty()) {
		// Find shortest edge between tree-list and graph-list
		float shortest_leg = std::numeric_limits<float>::max();
		std::list<Vertex*>::iterator t_it = t_list.begin(), best_v = g_list.begin();
		for(; t_it != t_list.end(); ++t_it) {
			std::list<Vertex*>::iterator g_it = g_list.begin();
			for(; g_it != g_list.end(); ++g_it) {
				float dist_t_to_g = (*t_it)->GetDistanceTo(*g_it);
				if(dist_t_to_g < shortest_leg) {
					shortest_leg = dist_t_to_g;
					best_v = g_it;
				}
			}
		}

		// Sanity Print
		if(GRAPH_DEBUG) {
			printf("Moving %d into tree\n", (*best_v)->nID);
		}

		// Move the next closest vertex into the tree, remove from graph
		Vertex* move_to_tree = *best_v;
		edgeMaxHeap.push(shortest_leg);
		dist += shortest_leg;
		g_list.erase(best_v);
		t_list.push_back(move_to_tree);
	}

	// Sanity Print
	if(GRAPH_DEBUG) {
		printf("Found MST distance of %f, ", dist);
	}

	// Delete off the longest legs from the tree
	for(int i = 0; i < (m - 1); i++) {
		dist -= edgeMaxHeap.top();
		edgeMaxHeap.pop();
	}

	// Sanity Print
	if(GRAPH_DEBUG) {
		printf("MSF distance of %f\n", dist);
	}

	return dist;
}

/*
 * Performs the Hungarian Algorithm on cost matrix A to find a minimum cost matching,
 * returns minimum cost and stores answer in result. More formally:
 *
 * Given the cost matrix "vector<vector<int>> A {...};", find the maximum matching
 * "vector<pair<int,int>>result;" with all pairs. As well as total cost "int C;"
 * with the minimum assignment cost.
 *
 * Many thanks to Zafar Takhirov for giving such a concise implementation and explaining
 * how it works.
 *
 * http://zafar.cc/2017/7/19/hungarian-algorithm/
 */
int Graph_Theory_Algorithms::Hungarian_Algorithm(std::vector<std::vector<int>>& A, std::vector<std::pair<int, int>>& result) {
	// Given the cost matrix "vector<vector<int>> A {...};"
	// Find the maximum matching "vector<pair<int,int>>result;" with all pairs
	// As well as total cost "int C;" with the minimum assignment cost.
	int n = (int)A.size();
	int m = (int)A.at(0).size();

	std::vector<int> u(n + 1), v(m + 1), p(m + 1), way(m + 1);
	for (int i=1; i<=n; ++i) {
		p[0] = i;
		int j0 = 0;
		std::vector<int> minv (m+1, std::numeric_limits<int>::max());
		std::vector<char> used (m+1, false);
		do {
			used[j0] = true;
			int i0 = p[j0],  delta = std::numeric_limits<int>::max(),  j1;
			for (int j=1; j<=m; ++j)
				if (!used[j]) {
					int cur = A[i0-1][j-1]-u[i0]-v[j];
					if (cur < minv[j])
						minv[j] = cur,  way[j] = j0;
					if (minv[j] < delta)
						delta = minv[j],  j1 = j;
				}
			for (int j=0; j<=m; ++j)
				if (used[j])
					u[p[j]] += delta,  v[j] -= delta;
				else
					minv[j] -= delta;
			j0 = j1;
		} while (p[j0] != 0);
		do {
			int j1 = way[j0];
			p[j0] = p[j1];
			j0 = j1;
		} while (j0);
	}

	for (int i = 1; i <= m; ++i){
	  result.push_back(std::make_pair(p[i], i));
	}

	int C = -v[0];
	return C;
}


//***********************************************************
// Private Member Functions
//***********************************************************

/*
 * bool runDFS_AdjMtrx() : runs recursive implementation of DFS to determine if a path exists
 * from edge u to edge v in an undirected graph defined on an adjacency matrix
 */
bool Graph_Theory_Algorithms::runDFS_AdjMtrx(bool** g, bool* visitedList, int g_size, int u, int v) {
	// base case
	if(u == v) {
		return true;
	}
	else {
		// Recursive step
		visitedList[u] = true;
		for(int i = 0; i < g_size; i++) {
			if(g[u][i]) {
				if(!visitedList[i]) {
//					printf(" (%d, %d)\n", u, i);
					if(runDFS_AdjMtrx(g, visitedList, g_size, i, v)) {
						return true;
					}
				}
			}
		}

		for(int i = 0; i < g_size; i++) {
			if(g[i][u]) {
				if(!visitedList[i]) {
//					printf(" (%d, %d)\n", u, i);
					if(runDFS_AdjMtrx(g, visitedList, g_size, i, v)) {
						return true;
					}
				}
			}
		}
	}

	return false;
}

