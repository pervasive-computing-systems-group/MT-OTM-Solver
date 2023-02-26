#include "Bae_Rathinam.h"

Bae_Rathinam::Bae_Rathinam() {
	// Seed rand()
	srand(time(NULL));
	m_bCreatesTree = true;
}

Bae_Rathinam::~Bae_Rathinam() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Bae-Rathinam algorithm
 *
 * This algorithm finds a 2-approximation solution to the Multiple Depot,
 * Multiple Terminal, Hamiltonian Path Problem (MDMTHPP) for m depots,
 * m terminals, and n destinations to visit. Here we assume that n >= m
 * and weight function w(e) is defined using Euclidean distances.
 */
void Bae_Rathinam::RunAlgorithm(Solution* solution) {
	/// 1. Find optimal constrained forest
	// 1.1 Create digraph G' = (V', E'): V' = V, E' = { D->U, U->U, U->P} for all v in D, U, P
	// Create 2D array, G'
	DEdge*** G_prime = new DEdge**[solution->m_nNumVertices];
	for(int i = 0; i < solution->m_nNumVertices; i++) {
		G_prime[i] = new DEdge*[solution->m_nNumVertices];
	}

	std::vector<DEdge*> E_prime;
	// Add edges to G', starting with edges leaving destination vertices
	for(int i = 0; i < solution->m_nN; i++) {
		for(int j = 0; j < solution->m_nNumVertices; j++) {
			if((i != j) && (solution->m_pVertexData[j].eVType != e_Depot)) {
				G_prime[i][j] = new DEdge((solution->m_pVertexData + i), (solution->m_pVertexData + j));
				E_prime.push_back(G_prime[i][j]);
			}
			else {
				G_prime[i][j] = NULL;
			}
		}
	}

	// Terminals do not have edges going out of them
	for(int i = solution->m_nN; i < (solution->m_nN + solution->m_nM); i++) {
		for(int j = 0; j < solution->m_nNumVertices; j++) {
			G_prime[i][j] = NULL;
		}
	}

	// Add edges from depots to destinations, with no edges from depots to terminals
	for(int i = (solution->m_nN + solution->m_nM); i < solution->m_nNumVertices; i++) {
		for(int j = 0; j < solution->m_nNumVertices; j++) {
			if(solution->m_pVertexData[j].eVType == E_VertexType::e_Destination) {
				G_prime[i][j] = new DEdge((solution->m_pVertexData + i), (solution->m_pVertexData + j));
				E_prime.push_back(G_prime[i][j]);
			}
			else {
				G_prime[i][j] = NULL;
			}
		}
	}

	// Sanity print
//	printf("\nG': \n  ");
//	for(int i = 0; i < solution->m_nNumVertices; i++) {
//		printf("%d\t", i);
//	}
//	printf("\n");
//	for(int i = 0, e = 0; i < solution->m_nNumVertices; i++) {
//		printf("%d|", i);
//		for(int j = 0; j < solution->m_nNumVertices; j++) {
//			if(G_prime[i][j] != NULL) {
//				printf("%d\t", e);
//				e++;
//			}
//			else {
//				printf("-\t");
//			}
//		}
//		printf("\n");
//	}

	// 1.2 Create matroid M'_1 = (E', I'_1)
	// 1.2.1 Define sets S_1 to S_(n + m/2) from E' where e is in S_i if edge e is directed into v_i
	// 1.2.2 Define l_i = 0, u_i = 1 for sets S_1 to S_(n + m/2)
	int l_i = 0;	// l_i and u_i are the same for-all i
	int u_i = 1;
	// We now have a partitioned matroid, M'_1, where A in I'_1 is independent if l_i <= |A ∩ S_i| <= u_i = 1

	// 1.3 Create undirected graph G_ud = (V', E'_ud): E'_ud = E' without direction on edges.
	DEdge*** G_ud = G_prime;

	// 1.4 Create matroid M'_2 = (E', I'_2)
	// Let A in I'_2 be independent if A is a forest in G_ud (no cycles) and no two terminals in G_ud are connected by a path in A

	// 1.5 Define weight function w_λ(E') where w_λ(e) = (n+2m)*max(w(E')) if e is incident to v in D, P, and w(e) otherwise
	// Sanity print
//	printf("\nw(G'): \n");
//	for(int i = 0; i < solution->m_nNumVertices; i++) {
//		for(int j = 0; j < solution->m_nNumVertices; j++) {
//			if(G_prime[i][j] != NULL) {
//				printf("%f\t", G_prime[i][j]->getWeight());
//			}
//			else {
//				printf("0\t\t");
//			}
//		}
//		printf("\n");
//	}

	// Find max weight
	float max_weight = 0;
	for(DEdge* e : E_prime) {
		max_weight = std::max(e->getWeight(), max_weight);
	}

	// Update weight function for all edges incident to either a terminal or depot
	for(DEdge* e : E_prime) {
		if((e->p_oToVertex->eVType == e_Terminal) || (e->p_oToVertex->eVType == e_Depot)
				|| (e->p_oFromVertex->eVType == e_Terminal) || (e->p_oFromVertex->eVType == e_Depot)) {
			e->setLambda(solution->m_nNumVertices * max_weight);
		}
	}

	// Sanity print
//	printf("\nw_λ(G'): \n");
//	for(int i = 0; i < solution->m_nNumVertices; i++) {
//		for(int j = 0; j < solution->m_nNumVertices; j++) {
//			if(G_prime[i][j] != NULL) {
//				printf("%f\t", G_prime[i][j]->getWeight());
//			}
//			else {
//				printf("0\t\t");
//			}
//		}
//		printf("\n");
//	}

	// 1.6 Find minimum k-intersection in M'_1 and M'_2, where k = m + n
	int k = solution->m_nM + solution->m_nN;

	// 1.6.1 Define artificial edge set A, where |A| = k. Define F = A ∪ E'
	std::deque<AEdge*> A;	// F = A ∪ E_prime (is figurative here)

	// 1.6.2 Define w_λ(a) = -1*|E'|*(max(w_λ(E'))), for-all a in A
	// Find new max weight
	max_weight = 0;
	for(DEdge* e : E_prime) {
		max_weight = std::max(e->getWeight(), max_weight);
	}

	// Create artificial edges in A and assign them weights
	float w_of_A = -1 * (float)E_prime.size() * max_weight;
	for(int i = 0; i < k; i++) {
		A.push_back(new AEdge(w_of_A));
	}

	// Sanity print
//	printf("\nk = %d\nw_λ(A): \n", k);
//	for(AEdge* e : A) {
//		printf("%f\t", e->getWeight());
//	}
//	printf("\n");

	// 1.6.3 Define matroids M_1 and M_2 as M'_1 and M'_2, where A_i is independent in M_i if A_i ∩ E' is independent in M'_i
	// 1.6.4 Define initial solution I = A ( = V)
	std::map<Edge*, VertexOnEdge*> V;
	for(Edge* e : A) {
		V.insert(std::pair<Edge*, VertexOnEdge*>(e, new VertexOnEdge(e)));
	}

	std::map<Edge*, VertexOnEdge*> V_prime;
	for(Edge* e : E_prime) {
		V_prime.insert(std::pair<Edge*, VertexOnEdge*>(e, new VertexOnEdge(e)));
	}

	// Sanity print
//	printf("\n |V| = %ld, |V'| = %ld\n", V.size(), V_prime.size());
//	{
//		std::map<Edge*, VertexOnEdge*> T;
//		int j = 0;
//		for(Edge* e : E_prime) {
//			T.insert(std::pair<Edge*, VertexOnEdge*>(e, new VertexOnEdge(e)));
//			j++;
//			if(j == 4) {
//				break;
//			}
//		}
//
//		printf("\nChecking M1, 1\n");
//		printf(" M1 b-swap 1: %d\n", M1_beta_swap(G_prime, solution->m_nNumVertices, T, E_prime[0], E_prime[5], l_i, u_i));
//
//		printf("\nChecking M1, 2\n");
//		printf(" M1 b-swap 2: %d\n", M1_beta_swap(G_prime, solution->m_nNumVertices, T, E_prime[1], E_prime[5], l_i, u_i));
//
//		printf("\nChecking M2, 1\n");
//		printf(" M2 b-swap 1: %d\n", M2_beta_swap(G_prime, solution->m_nNumVertices, T, E_prime[2], E_prime[5],
//				solution->m_nM, solution->m_nN));
//
//		printf("\nChecking M2, 2\n");
//		printf(" M2 b-swap 2: %d\n", M2_beta_swap(G_prime, solution->m_nNumVertices, T, E_prime[3], E_prime[6],
//				solution->m_nM, solution->m_nN));
//
//		printf("\nChecking M2, 3\n");
//		printf(" M2 b-swap 3: %d\n", M2_beta_swap(G_prime, solution->m_nNumVertices, T, E_prime[1], E_prime[12],
//				solution->m_nM, solution->m_nN));
//
//		printf("\nChecking M2, 4\n");
//		printf(" M2 b-swap 4: %d\n", M2_beta_swap(G_prime, solution->m_nNumVertices, T, E_prime[3], E_prime[12],
//				solution->m_nM, solution->m_nN));
//
//		for(std::pair<Edge * const,VertexOnEdge *> pair : T) {
//			delete pair.second;
//		}
//	}

	// 1.6.5 Repeat k-times: (until all artificial edges are deleted from F)
	for(int i = 0; i < k; i++) {
		int id = 0;
		std::vector<T_BipartitEdge> bEdgeList;

		// 1.6.5.1 Create directed bipartite graph G(I) where V = I, V' = F - I
		// Competed previously

		// 1.6.5.2 Create edge (v, v') if I is still independent in M_1 after swapping v and v'. Set w((v, v')) = w_λ(v') - w_λ(v)
		for(const auto& v : V) {
			for(const auto& v_prime : V_prime) {
				// Check if v and v' are a β-swap
				if(M1_beta_swap(G_prime, solution->m_nNumVertices, V, v.first, v_prime.first, l_i, u_i)) {
					// Add new edge to v
					v.second->addDEdgeOut(id, v_prime.second, (v_prime.first->getWeight() - v.first->getWeight()));
					id++;
				}
			}
		}

		// 1.6.5.3 Create edge (v', v) if I is still independent in M_2 after swapping v and v'. Set w((v', v)) = 0
		for(const auto& v_prime : V_prime) {
			for(const auto& v : V) {
				// Check if v and v' are a β-swap in M_2
				if(M2_beta_swap(G_ud, solution->m_nNumVertices, V, v.first, v_prime.first, solution->m_nM, solution->m_nN)) {
					// Add new edge to v'
					v_prime.second->addDEdgeOut(id, v.second, 0);
					id++;
				}
			}
		}

		// 1.6.5.4 Create directed bipartite graph Sp(I) by dividing one of the artificial nodes, w, in I into w_s and w_d.
		AEdge* w;
		VertexOnEdge* w_d;
		VertexOnEdge* w_s;

		// 1.6.5.5 Assign all edges leaving w, (w, v'), to w_s and edging going to w, (v', w), to w_d
		// Find each of the above sets A and V, create the source (w_s) and sink (w_d) vertices
		if(A.size() > 0) {
			// Grab an artificial vertex
			w = A.front();
			// We don't need w in A anymore (will delete later)
			A.pop_front();
			std::map<Edge *,VertexOnEdge *>::iterator w_it = V.find(w);
			if(w_it != V.end()) {
				w_d = w_it->second;
				w_s = new VertexOnEdge(w, w_d);
			}
			else {
				printf("[ERROR] : couldn't find w in V!\n");
				exit(1);
			}
		}
		else {
			printf("[ERROR] : set A is empty!\n");
			exit(1);
		}


		// 1.6.5.6 Find minimum dipath P from w_s to w_d
		// Collect all edges in the bipartite graph
		bEdgeList.insert(bEdgeList.end(), w_s->getEdgesList().begin(), w_s->getEdgesList().end());
		for(const auto& v : V) {
			bEdgeList.insert(bEdgeList.end(), v.second->getEdgesList().begin(), v.second->getEdgesList().end());
		}

		for(const auto& v_prime : V_prime) {
			bEdgeList.insert(bEdgeList.end(), v_prime.second->getEdgesList().begin(), v_prime.second->getEdgesList().end());
		}

		printf("id: %d, |edge-list|: %ld\n", id, bEdgeList.size());

		// Create mappings of vertices with costs and parents
		std::map<VertexOnEdge*, float> cost;
		std::map<VertexOnEdge*, VertexOnEdge*> parentList;
		// Add the source vertex
		cost.insert(std::pair<VertexOnEdge*, float>(w_s, std::numeric_limits<float>::max()));
		parentList.insert(std::pair<VertexOnEdge*, VertexOnEdge*>(w_s, NULL));
		// Add all vertices in V
		for(const auto& v : V) {
			cost.insert(std::pair<VertexOnEdge*, float>(v.second, std::numeric_limits<float>::max()));
			parentList.insert(std::pair<VertexOnEdge*, VertexOnEdge*>(v.second, NULL));
		}
		// Add all vertices in V'
		for(const auto& v_prime : V_prime) {
			cost.insert(std::pair<VertexOnEdge*, float>(v_prime.second, std::numeric_limits<float>::max()));
			parentList.insert(std::pair<VertexOnEdge*, VertexOnEdge*>(v_prime.second, NULL));
		}

		// Run Bellman–Ford algorithm to find shortest path from w_s to w_d
		bool has_negative_cycle = Graph_Theory_Algorithms::Bellman_Ford_Maps(bEdgeList, w_s, cost, parentList);
		// Verify that we didn't create a negative cycle
		if(has_negative_cycle) {
			printf("[ERROR] : graph Sp(I) has a negative cycle!!\n");
			exit(1);
		}

		// 1.6.5.7 Let Σ = {v: v in P ∩ V}, Σ' = {v: v in P ∩ V'}, I' = (I - Σ) ∪ Σ', F' = F - {w}
		std::vector<VertexOnEdge*> sigma;
		std::vector<VertexOnEdge*> sigma_prime;

		// Follow parent list from w_d back to w_s
		VertexOnEdge* v = w_d;
		while(v != w_s) {
			// Move to parent node
			v = parentList.at(v);
			// Verify that there is a continuous path back to w_s
			if(v == NULL) {
				printf("[ERROR] : no path from w_d to w_s\n");
				exit(1);
			}
			else {
				// Check if vertex on path is in V or V'
				if(V_prime.find(v->m_pUnderlyingEdge) != V_prime.end()) {
					// In V', add to Σ'
					sigma_prime.push_back(v);
				}
				else {
					// In V, add to Σ
					sigma.push_back(v);
				}
			}
		}

		// We are done with w_s (aka w), delete it
		V.erase(w_d->m_pUnderlyingEdge);
		delete w_d;

		// Update V (= I' = (I - Σ) ∪ Σ'), V' = ((V' -  Σ') ∪ Σ) - {w}
		// Add elements from Σ to V'
		for(VertexOnEdge* v : sigma) {
			if(v == w_s) {
				// Done with the source, delete it
				delete w_s;
			}
			else {
				// Remove from V, place in V'
				V.erase(v->m_pUnderlyingEdge);
				V_prime.insert(std::pair<Edge*, VertexOnEdge*>(v->m_pUnderlyingEdge, v));
			}
		}

		// Add elements from Σ' to V
		for(VertexOnEdge* v : sigma_prime) {
			// Remove from V', place in V
			V_prime.erase(v->m_pUnderlyingEdge);
			V.insert(std::pair<Edge*, VertexOnEdge*>(v->m_pUnderlyingEdge, v));
		}

		// Clean-up G(I)
		for(auto v : V) {
			v.second->clearEdgeList();
		}
		for(auto v : V_prime) {
			v.second->clearEdgeList();
		}

		// Done with w, delete it
		delete w;

		// Sanity print
		printf(" |V| = %ld, |V'| = %ld\n", V.size(), V_prime.size());
	}

	// 1.7 Return I', the edges in the optimal constrained forest
	// Sanity print
	printf("\nFound constrained forest:\n");
	FILE * pForestFile;
	pForestFile = fopen("forest_output.txt", "w");
	// I' = V
	for(std::pair<Edge * const,VertexOnEdge *> v : V) {
//		printf(" ( %d, %d)\n", ((DEdge*)v.first)->p_oFromVertex->nID, ((DEdge*)v.first)->p_oToVertex->nID);
		fprintf(pForestFile, "%d %d\n", ((DEdge*)v.first)->p_oFromVertex->nID, ((DEdge*)v.first)->p_oToVertex->nID);
	}

	fclose(pForestFile);
	// Create adjacency matrix with the edges found in the k-intersection in M'_1 and M'_2
	printf("Create adj matrix\n");
	// Add edges to adjacency matrix
	for(std::pair<Edge * const,VertexOnEdge *> v : V) {
		int a = ((DEdge*)v.first)->p_oFromVertex->nID;
		int b = ((DEdge*)v.first)->p_oToVertex->nID;
//		printf(" adding ( %d, %d)\n", ((DEdge*)v.first)->p_oFromVertex->nID, ((DEdge*)v.first)->p_oToVertex->nID);

		if(b > a) {
			solution->m_pAdjMatrix[a][b] = true;
		}
		else {
			solution->m_pAdjMatrix[b][a] = true;
		}
	}

	// Clean up memory for V and V'
	for(std::pair<Edge * const,VertexOnEdge *> v : V) {
		delete v.second;
	}
	for(std::pair<Edge * const,VertexOnEdge *> v : V_prime) {
		delete v.second;
	}
	// Clean up edges created in G'
	for(int i = 0; i < solution->m_nNumVertices; i++) {
		for(int j = 0; j < solution->m_nNumVertices; j++) {
			delete G_prime[i][j];
		}
	}
	// Clean up G'
	for(int i = 0; i < solution->m_nNumVertices; i++) {
		delete[] G_prime[i];
	}
	delete[] G_prime;

	// Ensure that there isn't already a partition
	solution->m_mPartitions.clear();
	// Add partitions to solution
	for(int i = 0; i < solution->m_nM; i++) {
		std::vector<Vertex*> partList;
		// Create list to track neighborhood
		bool* nbrhoodList = new bool[solution->m_nNumVertices];
		for(int k = 0; k < solution->m_nNumVertices; k++) {
			nbrhoodList[k] = false;
		}
		// Find all nodes connected to each terminal (all nodes in the partition)
		Graph_Theory_Algorithms::DFS_Neighborhood_AdjMtrx(solution->m_pAdjMatrix, nbrhoodList, solution->m_nNumVertices, (i + solution->m_nN));
		// Add these nodes to the this partition
		for(int j = 0; j < solution->m_nNumVertices; j++) {
			if(nbrhoodList[j]) {
				// Add to partition
				partList.push_back(solution->m_pVertexData + j);
			}
		}
		// Free memory
		delete[] nbrhoodList;

		// Add this partition to the partitioning map
		solution->m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(i, partList));
	}

	// Sanity print
//	for(auto vect: solution->m_mPartitions) {
//		printf("Partition: %d\n ", vect.first);
//		for(auto v : vect.second) {
//			printf("%d ", v->nID);
//		}
//		printf("\n");
//	}
}



//***********************************************************
// Private Member Functions
//***********************************************************

// Determines if edges a and b are a beta swap, as defined in the Bae-Rathinam algorithm
bool Bae_Rathinam::M1_beta_swap(DEdge*** g, int g_size, std::map<Edge*, VertexOnEdge*> set_A, Edge* a, Edge* b, int l_limit, int u_limit) {
	bool ret_val = true;

	// Track how many edges from set A are in each sub-set from g
	int* setCouts = new int[g_size];
	for(int i = 0; i < g_size; i++) {
		setCouts[i] = 0;
	}

	// Check every edge in g
	for(int n = 0; n < g_size; n++) {
		for(int m = 0; m < g_size; m++) {
			if(g[n][m] != NULL) {
				// Skip edge a
				if(g[n][m] != a) {
					// If this is b, assume we swapped-out b into set A
					if(g[n][m] == b) {
						setCouts[m]++;
					}
					else {
						// Check if this edge is in set A
						std::map<Edge*, VertexOnEdge*>::iterator it = set_A.find(g[n][m]);
						if(it != set_A.end()) {
							setCouts[m]++;
						}
					}
				}
			}
		}
	}

	// Verify that swapping a and b didn't violate set constraints
	for(int i = 0; i < g_size; i++) {
		if((setCouts[i] < l_limit) || (setCouts[i] > u_limit)) {
			ret_val = false;
		}
	}

	// Clean-up memory
	delete[] setCouts;

	return ret_val;
}

bool Bae_Rathinam::M2_beta_swap(DEdge*** g, int g_size, std::map<Edge*, VertexOnEdge*> set_A, Edge* a, Edge* b, int m, int n) {
	bool ret_val = true;
	int adjB1 = -1, adjB2 = -1;

	// Recreate our graph only using edges in set A (for ease of DFS)
	bool** G_base = new bool*[g_size];
	for(int i = 0; i < g_size; i++) {
		G_base[i] = new bool[g_size];
	}

	// Add edges to base graph from set A, skip edges a and b
	for(int n = 0; n < g_size; n++) {
		for(int m = 0; m < g_size; m++) {
			if(g[n][m] != NULL) {
				// Skip edge a
				if(g[n][m] == a) {
					G_base[n][m] = false;
				}
				else {
					if(g[n][m] == b) {
						// Record which vertices are connected by adding b and leave b out for now
						G_base[n][m] = false;
						adjB1 = n;
						adjB2 = m;
					}
					else {
						// Check if this edge is in set A
						if(set_A.find(g[n][m]) != set_A.end()) {
							// Mark this edge in G_base
							G_base[n][m] = true;
						}
						else {
							// Edge isn't in A
							G_base[n][m] = false;
						}
					}
				}
			}
			else {
				G_base[n][m] = false;
			}
		}
	}

	// If b is not in g, then we assume b is an artificial edge and can be added to set A
	if(adjB1 == -1) {
//		printf(" b is an artificial edge\n");
		ret_val = true;
	}
	else {
		/// Do DFS to determine if replacing edge a with b create a cycle
		// Search for a path from vertex adjB1 to adjB2
//		printf(" checking for path from %d to %d\n", adjB1, adjB2);
		if(Graph_Theory_Algorithms::DFS_AdjMtrx(G_base, g_size, adjB1, adjB2)) {
			// Path exists from vertex adjB1 to adjB2, adding b will create a cycle.
			ret_val = false;
		}
		else {
			/// Determine if replacing edge a with b creates a walk between two terminals
//			printf(" checking for connected terminals\n");
			// Add b to base graph
			G_base[adjB1][adjB2] = true;
			// Find the neighborhood of each terminal
			int first_terminal = n;
			int first_depot = n + m;
			// Create list to track neighborhood
			bool* nbrhoodList = new bool[g_size];
			// Check the neighborhood of each terminal
			for(int i = first_terminal; i < first_depot; i++) {
				// Reset neighborhood list
				for(int j = 0; j < g_size; j++) {
					nbrhoodList[j] = false;
				}

				// Find neighborhood of i
				Graph_Theory_Algorithms::DFS_Neighborhood_AdjMtrx(G_base, nbrhoodList, g_size, i);
//				printf("  neighbors of %d: ", i);
//				for(int j = 0; j < g_size; j++) {
//					if(nbrhoodList[j]) {
//						printf("%d ", j);
//					}
//				}
//				printf("\n");
				// Check neighborhood for other terminals
				for(int j = i + 1; j < first_depot; j++) {
					if(nbrhoodList[j]) {
						// Found another terminal in neighborhood of i -> there exists a path between two terminals
						ret_val = false;
						break;
					}
				}

				if(!ret_val) {
					// Stop searching neighborhoods!
					break;
				}
			}

			// Clean-up memory for neighborhood list
			delete[] nbrhoodList;
		}
	}

	// Clean-up base-graph memory
	for(int i = 0; i < g_size; i++) {
		delete[] G_base[i];
	}
	delete[] G_base;

	return ret_val;
}

