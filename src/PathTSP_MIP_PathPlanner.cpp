#include "PathTSP_MIP_PathPlanner.h"


//***********************************************************
// Subtour Class Functions
//***********************************************************

Subtour::Subtour() {
}

Subtour::~Subtour() {
}

// Given an integer-feasible solution 'sol', find the smallest
// sub-tour.  Result is returned in 'tour', and length is
// returned in 'tourlenP'.
void Subtour::findsubtour(int n, double** sol, int* tourlenP, int* tour) {
	// Function variables
	int bestind, bestlen;
	int i, node, len, start;
	bestlen = n+1;
	bestind = -1;
	start = 0;
	node = 0;

	// Track which stops have been seen so far
	bool* seen = new bool[n];
	for(i = 0; i < n; i++) {
		seen[i] = false;
	}

	// Find all sub-tours, record which one is the best
	while(start < n) {
		// Find a next node that we haven't seen yet
		for(node = 0; node < n; node++) {
			if (!seen[node])
				break;
		}

		// If we have seen every node, stop algorithm
		if(node == n) {
			break;
		}

		for(len = 0; len < n; len++) {
			// Record this stop and mark it as seen
			tour[start+len] = node;
			seen[node] = true;

			// Find next stop on tour
			for(i = 0; i < n; i++) {
				if(sol[node][i] > 0.5 && !seen[i]) {
					node = i;
					break;
				}
			}

			// Check to see if we have traversed the entire tour
			if(i == n) {
				// Traversed this tour, (maybe) update best tour length found
				len++;
				if(len < bestlen) {
					bestlen = len;
					bestind = start;
				}

				// Update start marker
				start += len;
				break;
			}
		}
	}

	// Update tour array so that best sub-tour is at the beginning
	for(i = 0; i < bestlen; i++) {
		tour[i] = tour[bestind+i];
	}
	*tourlenP = bestlen;

	delete[] seen;
}


//***********************************************************
// SubtourElim Class Functions
//***********************************************************

SubtourElim::SubtourElim(GRBVar** xvars, int xn) {
	vars = xvars;
	n = xn;

}

SubtourElim::~SubtourElim() {}


void SubtourElim::callback() {
	try {
		if(where == GRB_CB_MIPSOL) {
			// Found an integer feasible solution - does it visit every node?
			double **x = new double*[n];
			int *tour = new int[n];
			int i, j, len;

			// Get current solution from model
			for(i = 0; i < n; i++) {
				x[i] = getSolution(vars[i], n);	// Allocates memory!
			}

			// Check length of smallest sub-tour in solution
			sb.findsubtour(n, x, &len, tour);

			// If smallest sub-tour does not contain all stops, create a lazy constraint (cut off this node in the solver)
			if(len < n) {
				// Add sub-tour elimination constraint
				GRBLinExpr expr = 0;
				for (i = 0; i < len; i++) {
					for (j = i+1; j < len; j++) {
						expr += vars[tour[i]][tour[j]];
					}
				}
				addLazy(expr <= len-1);
			}

			// Cleanup memory
			for(i = 0; i < n; i++) {
				delete[] x[i];
			}
			delete[] x;
			delete[] tour;
		}
	}
	catch(GRBException e) {
		fprintf(stderr, "[ERROR] SubtourElim::callback() : Error number %d\n\t%s\n", e.getErrorCode(), e.getMessage().c_str());
		exit(1);
	}
	catch(...) {
		fprintf(stderr, "[ERROR] SubtourElim::callback() : Error during callback\n");
		exit(1);
	}
}



//***********************************************************
// PathTSP_MIP_PathPlanner Class Functions
//***********************************************************

PathTSP_MIP_PathPlanner::PathTSP_MIP_PathPlanner() {
}

PathTSP_MIP_PathPlanner::~PathTSP_MIP_PathPlanner() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Uses a random ordering algorithm to "solve" the Path TSP on each partition in solution
 */
void PathTSP_MIP_PathPlanner::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Running Path-TSP MIP for Path-Planning *\n");
	for(int i = 0; i < solution->m_nM; i++) {
		// Run Path-TSP MIP on this partition
		PathTSPMIP(solution, i);
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************

std::string PathTSP_MIP_PathPlanner::itos(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

//
void PathTSP_MIP_PathPlanner::PathTSPMIP(Solution* solution, int p) {

	// Program variables
	GRBEnv *env = NULL;
	GRBVar **vars = NULL;
	int n = solution->m_mPartitions.at(p).size();

	// Create an N x N array for decision variables
	vars = new GRBVar*[n];
	for(int i = 0; i < n; i++) {
		vars[i] = new GRBVar[n];
	}

	try {
		// Create Gurobi environment
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);

		// Tell the model that we are going to use lazy constraints
		model.set(GRB_IntParam_LazyConstraints, 1);

		// Create binary variables representing edges in graph
		for(int i = 0; i < n; i++) {
			for(int j = 0; j <= i; j++) {
				Vertex* v_i = solution->m_mPartitions.at(p).at(i);
				Vertex* v_j = solution->m_mPartitions.at(p).at(j);
				float dist = v_i->GetDistanceTo(v_j);
				vars[i][j] =
						model.addVar(
								0.0,
								1.0,
								dist,
								GRB_BINARY,
								"x_"+itos(i)+"_"+itos(j));
				vars[j][i] = vars[i][j];
			}
		}

		// Each stop must have an edge in and an edge out (Degree-2 constraint)
		for(int i = 0; i < n; i++) {
			GRBLinExpr expr = 0;
			for(int j = 0; j < n; j++) {
				expr += vars[i][j];
			}
			model.addConstr(expr == 2, "deg2_"+itos(i));
		}

		// No self-loops
		for(int i = 0; i < n; i++) {
			vars[i][i].set(GRB_DoubleAttr_UB, 0);
		}

		// Force v_t -> v_d
		vars[n-2][n-1].set(GRB_DoubleAttr_LB, 1);
		// Block v_d -> v_t
//		vars[n-1][n-2].set(GRB_DoubleAttr_UB, 0);

		// Set callback function
		SubtourElim cb = SubtourElim(vars, n);
		model.setCallback(&cb);

		// Optimize model
		model.optimize();

		// Extract solution
		if(model.get(GRB_IntAttr_SolCount) > 0) {
			double **sol = new double*[n];
			for(int i = 0; i < n; i++) {
				sol[i] = model.get(GRB_DoubleAttr_X, vars[i], n);
			}

			if(DEBUG_MIPTSP) {
				printf("Tour:\n");
			}
			for(int i = 0; i < n; i++) {
				for(int j = 0; j < n; j++) {
					// Check to see if this variable was set
					if(sol[i][j] > 0.5) {
						Vertex* v = solution->m_mPartitions.at(p).at(i);
						Vertex* u = solution->m_mPartitions.at(p).at(j);
						// Verify that we aren't looking at the Depot -> Terminal edge
						if(((v->eVType == e_Depot) && (u->eVType == e_Terminal)) ||
								((u->eVType == e_Depot) && (v->eVType == e_Terminal))) {
							if(DEBUG_MIPTSP) {
								printf(" %d -/> %d", v->nID, u->nID);
							}
						}
						else {
							// Not Depot -> Terminal edge, add this edge to the solution
							int a = v->nID;
							int b = u->nID;

							if(DEBUG_MIPTSP) {
								printf(" %d -> %d", a, b);
							}

							if(a < b) {
								solution->m_pAdjMatrix[a][b] = true;
							}
							else {
								solution->m_pAdjMatrix[b][a] = true;
							}
						}
					}
				}
			}
			if(DEBUG_MIPTSP) {
				printf("\n");
			}

			for(int i = 0; i < n; i++)
				delete[] sol[i];
			delete[] sol;
		}
		else {
			fprintf(stderr, "[ERROR] PathTSP_MIP_PathPlanner::PathTSPMIP() : Couldn't find a solution!\n");
			exit(1);
		}
	}
	catch(GRBException e) {
		fprintf(stderr, "[ERROR] PathTSP_MIP_PathPlanner::PathTSPMIP() : Error number %d\n\t%s\n", e.getErrorCode(), e.getMessage().c_str());
		exit(1);
	}
	catch(...) {
		fprintf(stderr, "[ERROR] PathTSP_MIP_PathPlanner::PathTSPMIP() : Error during callback\n");
		exit(1);
	}

	// Clean-up dynamic memory
	for(int i = 0; i < n; i++) {
		delete[] vars[i];
	}
	delete[] vars;
	delete env;
}
