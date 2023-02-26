#include "ShortCutTree.h"

ShortCutTree::ShortCutTree() {
}

ShortCutTree::~ShortCutTree() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Short-cuts the trees in a constrained forest to make multiple Hamiltonian paths.
 * This is based on the short-cuting algorithm given with the Bae-Rathinam algorithm.
 */
void ShortCutTree::RunAlgorithm(Solution* pathSolution) {
	// TODO: Ensure that pathSolution has a forest!

	// Create parent list and set all parents to -1
	int* parentList = new int[pathSolution->m_nNumVertices];
	for(int i = 0; i < pathSolution->m_nNumVertices; i++) {
		parentList[i] = -1;
	}

	// Run DFS to find path from each depot to each terminal
	for(int i = (pathSolution->m_nM + pathSolution->m_nN); i < pathSolution->m_nNumVertices; i++) {
		// Set parent of each depot as itself
		parentList[i] = i;
		// Run DFS. Should return true
		if(!Graph_Theory_Algorithms::DFS_TopAdjMtrx(pathSolution->m_pAdjMatrix,
				parentList, pathSolution->m_nNumVertices, i, (i - pathSolution->m_nM))) {
			printf("[ERROR] ShortCutTree::RunAlgorithm() : I' does not contain a path from %d to %d!\n", i, (i - pathSolution->m_nM));
			exit(1);
		}
	}

	// Sanity print
	if(T_SHORT_CUT_DEBUG) {
		printf("Parent List:\n  ");
		for(int i = 0; i < pathSolution->m_nNumVertices; i++) {
			printf("%d\t", i);
		}
		printf("\n  ");
		for(int i = 0; i < pathSolution->m_nNumVertices; i++) {
			printf("%d\t", parentList[i]);
		}
		printf("\n");
	}

	// 2.1 Double all edges in I' not on the path between the depot and terminal node of each tree
	// We will clear the entries in the adjacency matrix that are on the path from d to p. Entries
	//  left over are considered doubled edges
	// 2.2 Short-cut all paths
	for(int i = (pathSolution->m_nM + pathSolution->m_nN); i < pathSolution->m_nNumVertices; i++) {
		// Create two stacks, first for initial path and second for short-cut path
		std::stack<int> stack1;
		std::stack<int> stack2;

		if(T_SHORT_CUT_DEBUG)
			printf("Stack of %d:\n |", i);

		// Find path (from terminal to depot)
		int j = (i - pathSolution->m_nM);
		stack1.push(j);
		if(T_SHORT_CUT_DEBUG)
			printf("%d ", j);
		while(j != i) {
			// Grab next vertex
			int next = parentList[j];

			// Remove appropriate edge from the adjacency list
			if(next < j) {
				pathSolution->m_pAdjMatrix[next][j] = false;
			}
			else {
				pathSolution->m_pAdjMatrix[j][next] = false;
			}

			// Push vertex onto stack
			stack1.push(next);
			if(T_SHORT_CUT_DEBUG)
				printf("%d ", next);
			j = next;
		}
		if(T_SHORT_CUT_DEBUG)
			printf("\n");

		// Short-cut doubled edges
		while(!stack1.empty()) {
			int current = stack1.top();
			stack1.pop();

			// Search for double edges connected to our path
			for(int l = 0; l < pathSolution->m_nNumVertices; l++) {
				int a, b;
				if(l < current) {
					a = l;
					b = current;
				}
				else {
					a = current;
					b = l;
				}

				if(pathSolution->m_pAdjMatrix[a][b]) {
					// Found a doubled edge
					pathSolution->m_pAdjMatrix[a][b] = false;

					// Determine which length is shorter, {v1, a, b, v2} or {v1, b, a, v2}
					int v1 = stack1.top(), v2 = stack2.top();
					float v1_a = sqrt(pow((pathSolution->m_pVertexData[a].fX - pathSolution->m_pVertexData[v1].fX), 2)
							+ pow((pathSolution->m_pVertexData[a].fY - pathSolution->m_pVertexData[v1].fY), 2));
					float v1_b = sqrt(pow((pathSolution->m_pVertexData[b].fX - pathSolution->m_pVertexData[v1].fX), 2)
							+ pow((pathSolution->m_pVertexData[b].fY - pathSolution->m_pVertexData[v1].fY), 2));
					float a_b = sqrt(pow((pathSolution->m_pVertexData[b].fX - pathSolution->m_pVertexData[a].fX), 2)
							+ pow((pathSolution->m_pVertexData[b].fY - pathSolution->m_pVertexData[a].fY), 2));
					float b_a = a_b;
					float b_v2 = sqrt(pow((pathSolution->m_pVertexData[v2].fX - pathSolution->m_pVertexData[b].fX), 2)
							+ pow((pathSolution->m_pVertexData[v2].fY - pathSolution->m_pVertexData[b].fY), 2));
					float a_v2 = sqrt(pow((pathSolution->m_pVertexData[v2].fX - pathSolution->m_pVertexData[a].fX), 2)
							+ pow((pathSolution->m_pVertexData[v2].fY - pathSolution->m_pVertexData[a].fY), 2));
					float v1_a_b_v2 = v1_a + a_b + b_v2;
					float v1_b_a_v2 = v1_b + b_a + a_v2;

					// Sanity print
					if(T_SHORT_CUT_DEBUG) {
						printf(" comparing: {%d, %d, %d, %d}, {%d, %d, %d, %d}\n", v1, a, b, v2, v1, b, a, v2);
						printf("            %f\t%f\n", v1_a_b_v2, v1_b_a_v2);
					}

					// Update order
					if(v1_a_b_v2 < v1_b_a_v2) {
						stack1.push(a);
						stack1.push(b);
						l = pathSolution->m_nNumVertices;
					}
					else {
						stack1.push(b);
						stack1.push(a);
						l = pathSolution->m_nNumVertices;
					}
				}
				else if(l == (pathSolution->m_nNumVertices - 1)) {
					// If we made it the whole way through the list adjacency matrix without finding a double edge,
					//  then push current onto stack2.
					stack2.push(current);
				}
			}
		}

		// Done short-cutting, record results
		int a, b;
		a = stack2.top();
		stack2.pop();
		if(T_SHORT_CUT_DEBUG)
			printf("\n** Final path **\n ");
		while(!stack2.empty()) {
			b = stack2.top();

			// Document edge in solution
			if(a < b) {
				pathSolution->m_pAdjMatrix[a][b] = true;
			}
			else {
				pathSolution->m_pAdjMatrix[b][a] = true;
			}
			if(T_SHORT_CUT_DEBUG)
				printf("( %d, %d)\n ", a, b);
			a = b;
			stack2.pop();
		}
		if(T_SHORT_CUT_DEBUG)
			printf("\n\n");
	}

	// Clean-up dynamic memory
	delete[] parentList;
}


//***********************************************************
// Private Member Functions
//***********************************************************
