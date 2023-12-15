#include "PathTSP_LKH.h"

PathTSP_LKH::PathTSP_LKH() {
}

PathTSP_LKH::~PathTSP_LKH() {
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
void PathTSP_LKH::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Running Path-TSP MIP for Path-Planning *\n");
	for(int i = 0; i < solution->m_nM; i++) {
		std::vector<Vertex*> sub_tour;
		// Solve Path-TSP on this partition using the LKH solver
		runLKH_TSP(solution, &solution->m_mPartitions.at(i), &sub_tour);

		// Sanity print
		if(DEBUG_PTSP_LKH)
			printf("Found solution:\n");

		// Store solution
		Vertex* u = NULL;
		for(Vertex* v : sub_tour) {
			if(u != NULL) {
				int a = v->nID;
				int b = u->nID;

				// Sanity print
				if(DEBUG_PTSP_LKH)
					printf("%d - type = %d\n", v->nID, v->eVType);

				if(a < b) {
					solution->m_pAdjMatrix[a][b] = true;
				}
				else {
					solution->m_pAdjMatrix[b][a] = true;
				}
			}
			u = v;
		}
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************

//
//void PathTSP_LKH::PathTSP(Solution* solution, int p) {
//
//
//	/// Run TSP on resulting fixed-HPP
//	std::vector<std::vector<Vertex*>> previous_tours;
//	// For each cluster
//	for(int k = 0; k < solution->m_nM; k++) {
//		// Store pointers to the vertices in this cluster
//		std::vector<Vertex*> cluster;
//		for(KMPoint pnt : points) {
//			if(pnt.cluster == clust_to_part.at(k)) {
//				// Add vertex to the cluster
//				cluster.push_back(solution->m_pVertexData + pnt.vID);
//			}
//		}
//		// Add in the depot/terminal
//		cluster.push_back(solution->GetDepotOfPartion(k));
//		cluster.push_back(solution->GetTerminalOfPartion(k));
//
//		// Sanity print
//		printf("Sub-tour %d:\n", k);
//		for(Vertex* v : cluster) {
//			printf(" %d", v->nID);
//		}
//		printf("\n");
//
//		std::vector<Vertex*> sub_tour;
//
//		runLKH_TSP(solution, &cluster, &sub_tour);
//
//		// Add this sub-tour to the solution
//		previous_tours.push_back(sub_tour);
//	}
//
//	// Sanity print
//	printf("Tours:\n");
//	for(std::vector<Vertex*> sub_tour : previous_tours) {
//		printf(" {");
//		for(Vertex* v : sub_tour) {
//			printf(" %d", v->nID);
//		}
//		printf("}\n");
//	}
//
//
//
//
//
//	// Program variables
//	GRBEnv *env = NULL;
//	GRBVar **vars = NULL;
//	int n = solution->m_mPartitions.at(p).size();
//
//	// Create an N x N array for decision variables
//	vars = new GRBVar*[n];
//	for(int i = 0; i < n; i++) {
//		vars[i] = new GRBVar[n];
//	}
//
//	try {
//		// Create Gurobi environment
//		env = new GRBEnv();
//		GRBModel model = GRBModel(*env);
//
//		// Tell the model that we are going to use lazy constraints
//		model.set(GRB_IntParam_LazyConstraints, 1);
//
//		// Create binary variables representing edges in graph
//		for(int i = 0; i < n; i++) {
//			for(int j = 0; j <= i; j++) {
//				Vertex* v_i = solution->m_mPartitions.at(p).at(i);
//				Vertex* v_j = solution->m_mPartitions.at(p).at(j);
//				float dist = v_i->GetDistanceTo(v_j);
//				vars[i][j] =
//						model.addVar(
//								0.0,
//								1.0,
//								dist,
//								GRB_BINARY,
//								"x_"+itos(i)+"_"+itos(j));
//				vars[j][i] = vars[i][j];
//			}
//		}
//
//		// Each stop must have an edge in and an edge out (Degree-2 constraint)
//		for(int i = 0; i < n; i++) {
//			GRBLinExpr expr = 0;
//			for(int j = 0; j < n; j++) {
//				expr += vars[i][j];
//			}
//			model.addConstr(expr == 2, "deg2_"+itos(i));
//		}
//
//		// No self-loops
//		for(int i = 0; i < n; i++) {
//			vars[i][i].set(GRB_DoubleAttr_UB, 0);
//		}
//
//		// Force v_t -> v_d
//		vars[n-2][n-1].set(GRB_DoubleAttr_LB, 1);
//		// Block v_d -> v_t
////		vars[n-1][n-2].set(GRB_DoubleAttr_UB, 0);
//
//		// Set callback function
//		SubtourElim cb = SubtourElim(vars, n);
//		model.setCallback(&cb);
//
//		// Optimize model
//		model.optimize();
//
//		// Extract solution
//		if(model.get(GRB_IntAttr_SolCount) > 0) {
//			double **sol = new double*[n];
//			for(int i = 0; i < n; i++) {
//				sol[i] = model.get(GRB_DoubleAttr_X, vars[i], n);
//			}
//
//			if(DEBUG_PTSP_LKH) {
//				printf("Tour:\n");
//			}
//			for(int i = 0; i < n; i++) {
//				for(int j = 0; j < n; j++) {
//					// Check to see if this variable was set
//					if(sol[i][j] > 0.5) {
//						Vertex* v = solution->m_mPartitions.at(p).at(i);
//						Vertex* u = solution->m_mPartitions.at(p).at(j);
//						// Verify that we aren't looking at the Depot -> Terminal edge
//						if(((v->eVType == e_Depot) && (u->eVType == e_Terminal)) ||
//								((u->eVType == e_Depot) && (v->eVType == e_Terminal))) {
//							if(DEBUG_PTSP_LKH) {
//								printf(" %d -/> %d", v->nID, u->nID);
//							}
//						}
//						else {
//							// Not Depot -> Terminal edge, add this edge to the solution
//							int a = v->nID;
//							int b = u->nID;
//
//							if(DEBUG_PTSP_LKH) {
//								printf(" %d -> %d", a, b);
//							}
//
//							if(a < b) {
//								solution->m_pAdjMatrix[a][b] = true;
//							}
//							else {
//								solution->m_pAdjMatrix[b][a] = true;
//							}
//						}
//					}
//				}
//			}
//			if(DEBUG_PTSP_LKH) {
//				printf("\n");
//			}
//
//			for(int i = 0; i < n; i++)
//				delete[] sol[i];
//			delete[] sol;
//		}
//		else {
//			fprintf(stderr, "[ERROR] PathTSP_MIP_PathPlanner::PathTSPMIP() : Couldn't find a solution!\n");
//			exit(1);
//		}
//	}
//	catch(GRBException e) {
//		fprintf(stderr, "[ERROR] PathTSP_MIP_PathPlanner::PathTSPMIP() : Error number %d\n\t%s\n", e.getErrorCode(), e.getMessage().c_str());
//		exit(1);
//	}
//	catch(...) {
//		fprintf(stderr, "[ERROR] PathTSP_MIP_PathPlanner::PathTSPMIP() : Error during callback\n");
//		exit(1);
//	}
//
//	// Clean-up dynamic memory
//	for(int i = 0; i < n; i++) {
//		delete[] vars[i];
//	}
//	delete[] vars;
//	delete env;
//}

// Run LKH TSP solver on the give cluster of vertices
void PathTSP_LKH::runLKH_TSP(Solution* solution, std::vector<Vertex*>* cluster, std::vector<Vertex*>* sub_tour) {
	// Sanity print
	if(SANITY_PRINT) {
		printf("Given cluster:\n");
		for(Vertex* v : *cluster) {
			printf("%d - type = %d\n", v->nID, v->eVType);
		}
	}

	// Prep stuff
	int depot_index = cluster->size()-1;
	int terminal_index = cluster->size()-2;
	sub_tour->clear();

	/// List of total-tour minus depot and terminal
	std::list<int> totalPath;

	bool try_again = true;
	double multiplier = 100;
	while(try_again) {
		// Create input file for LKH solver
		solution->PrintLKHData(*cluster, multiplier);

		if(DEBUG_PTSP_LKH)
			printf("Running LKH\n");
		// Run TSP solver on this sub-tour
		std::system("LKH FixedHPP.par");

		if(DEBUG_PTSP_LKH)
			printf("Found the following solution:\n");

		// Open file with results
		std::ifstream file("LKH_output.dat");
		// Remove the first few lines...
		std::string line;
		for(int i = 0; i < 6; i++) {
			std::getline(file, line);
		}

		/// Make list of total-tour minus depot and terminal
		totalPath.clear();

		// Start parsing the data
		for(int i = 0; i < cluster->size(); i++) {
			std::getline(file, line);
			std::stringstream lineStreamN(line);
			// Parse the way-point from the line
			int n;
			lineStreamN >> n;
			totalPath.push_back(n-1);
			if(DEBUG_PTSP_LKH)
				printf(" %d", n-1);
		}
		if(DEBUG_PTSP_LKH)
			printf("\n");

		file.close();

		/// Correct the returned list from the LKH solver
		// Check for weird (easy) edge-cases
		if((totalPath.front() == depot_index) && (totalPath.back() == terminal_index)) {
			// Nothing to fix...
		}
		else if((totalPath.front() == terminal_index) && (totalPath.back() == depot_index)) {
			// Easy fix, just reverse the list
			totalPath.reverse();
		}
		else {
			// Correcting the path will take a little more work...
			// Scan totalPath to determine the given order
			bool reverseList = true;
			{
				std::list<int>::iterator it = totalPath.begin();

				while((*it != depot_index) && (it != totalPath.end())) {
					if(*it == terminal_index) {
						reverseList = false;
					}
					it++;
				}
			}

			// Rotate list so that it starts at the closest-to-depot way-point,
			//  and ends with the closest-to-ideal-stop
			bool rotate_again = true;
			while(rotate_again) {
				if(totalPath.front() == depot_index) {
					// Total path has been corrected
					rotate_again = false;
				}
				else {
					// Keep rotating list
					int temp = totalPath.front();
					totalPath.pop_front();
					totalPath.push_back(temp);
				}
			}

			if(reverseList) {
				// We were given the list "backwards", we need to reverse it
				int temp = totalPath.front();
				totalPath.pop_front();
				totalPath.push_back(temp);

				totalPath.reverse();
			}
		}

		// Verify that the list is correct
		if((totalPath.front() != depot_index) || (totalPath.back() != terminal_index)) {
			// Something went wrong...
			multiplier *= 10;
			if(multiplier < DBL_MAX) {
				try_again = true;
			}
			else {
				// Just give up...
				solution->m_bFeasible = false;
				return;
			}
		}
		else {
			// It worked!
			try_again = false;
		}
	}


	// Sanity print
	if(DEBUG_PTSP_LKH) {
		printf("Fixed total path:\n");
		for(int n : totalPath) {
			printf(" %d", n);
		}
		printf("\n\n");
	}

	// Create an ordered vector based on found path
	for(int n : totalPath) {
		sub_tour->push_back(cluster->at(n));
	}
}
