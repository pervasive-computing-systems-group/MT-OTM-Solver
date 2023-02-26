#include "MIP_FMDMTHPP.h"

MIP_FMDMTHPP::MIP_FMDMTHPP() {
}

MIP_FMDMTHPP::~MIP_FMDMTHPP() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

std::string MIP_FMDMTHPP::itos(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

int MIP_FMDMTHPP::U_d_k(int k) {
	// The math is 1-based but k is 0-based
	k++;
	return (L_M + 2)*(k - 1);
}

int MIP_FMDMTHPP::U_t_k(int k) {
	// The math is 1-based but k is 0-based
	k++;
	return k*(L_M + 1) + (k - 1);
}

/*
 * Formulate the MIP and run it on the Gurobi solver
 */
void MIP_FMDMTHPP::Run_MIP(Solution* solution) {
	N = solution->m_nN;
	M = solution->m_nM;
	L_M =(N - M + 1);
	BIG_M = (M*L_M + 2*M - 1);

	// Gurobi variables
	GRBEnv* env = NULL;
	// Tracks edges between way-points
	GRBVar** X = new GRBVar*[N];
	for(int i = 0; i < N; i++) {
		X[i] = new GRBVar[N];
	}
	// Tracks edges between depots and way-points
	GRBVar** Y = new GRBVar*[M];
	for (int k = 0; k < M; k++) {
		Y[k] = new GRBVar[N];
	}
	// Tracks edges between way-points and terminals
	GRBVar** Z = new GRBVar*[N];
	for (int i = 0; i < N; i++) {
		Z[i] = new GRBVar[M];
	}
	// Used to assign numbers to each location
	GRBVar* U = new GRBVar[N];

	try {
		// Create Gurobi environment
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);

		/// Define variables
		// Create binary decision variables for each X_{i,j}
		for(int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->m_pVertexData + j);
				X[i][j] = model.addVar(0.0, 1.0, dist, GRB_BINARY, "x_"+itos(i)+"_"+itos(j));
			}
		}

		// Create binary decision variables for each Y_{k,i}
		for(int k = 0; k < M; k++) {
			for (int i = 0; i < N; i++) {
				double dist = solution->GetDepotOfPartion(k)->GetDistanceTo(solution->m_pVertexData + i);
				Y[k][i] = model.addVar(0.0, 1.0, dist, GRB_BINARY, "y_"+itos(k)+"_"+itos(i));
			}
		}

		// Create binary decision variables for each Z_{i,k}
		for(int i = 0; i < N; i++) {
			for (int k = 0; k < M; k++) {
				double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->GetTerminalOfPartion(k));
				Z[i][k] = model.addVar(0.0, 1.0, dist, GRB_BINARY, "z_"+itos(i)+"_"+itos(k));
			}
		}

		// Create integer variables for each U_{i}
		for(int i = 0; i < N; i++) {
			U[i] = model.addVar(0.0, BIG_M, 0, GRB_INTEGER, "u_" + itos(i));
		}

		/// Constraints
		// Enforce numbering scheme after depots
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 0;
				expr += U[i] - U_d_k(k)*Y[k][i];
				model.addConstr(expr >= 1, "U_"+itos(i)+"_if_Y"+itos(k)+"_"+itos(i));
			}
		}

		// Enforce numbering scheme before terminals
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 0;
				expr += U_t_k(k)*Z[i][k] - U[i] + BIG_M*(1 - Z[i][k]);
				model.addConstr(expr >= 1, "U_"+itos(i)+"_if_Z"+itos(i)+"_"+itos(k));
			}
		}

		// Enforce numerical sequencing
		for(int i = 0; i < N; i++) {
			for(int j = 0; j < N; j++) {
				GRBLinExpr expr = 0;
				expr += BIG_M*X[i][j] + U[i] - U[j] + 1;
				model.addConstr(expr <= BIG_M, "U_"+itos(i)+"_to_U_"+itos(j));
			}
		}

		// Must have one edge going "in"
		for(int i = 0; i < N; i++) {
			GRBLinExpr expr = 0;
			for(int j = 0; j < N; j++) {
				expr += X[j][i];
			}
			for(int k = 0; k < M; k++) {
				expr += Y[k][i];
			}
			model.addConstr(expr == 1, "in-deg_"+itos(i));
		}

		// Must have one edge going "out"
		for(int i = 0; i < N; i++) {
			GRBLinExpr expr = 0;
			for(int j = 0; j < N; j++) {
				expr += X[i][j];
			}
			for(int k = 0; k < M; k++) {
				expr += Z[i][k];
			}
			model.addConstr(expr == 1, "out-deg_"+itos(i));
		}

		// Ensure that each depot has one tour
		for(int k = 0; k < M; k++) {
			GRBLinExpr expr = 0;
			for(int i = 0; i < N; i++) {
				expr += Y[k][i];
			}
			model.addConstr(expr == 1, "Y_"+itos(k));
		}

		// Ensure that each terminal has one tour
		for(int k = 0; k < M; k++) {
			GRBLinExpr expr = 0;
			for(int i = 0; i < N; i++) {
				expr += Z[i][k];
			}
			model.addConstr(expr == 1, "Z_"+itos(k));
		}

		// Set a time-out limit. If we hit the time-out, then we'll set a MIP-gap limit
		model.set(GRB_DoubleParam_TimeLimit, 200);

		// Optimize model
		model.optimize();

		// Check the status of the model to see if we timed-out
		if(model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
			// Set maximum time-out
			model.set(GRB_DoubleParam_TimeLimit, GRB_INFINITY);
			// Set a MIP-Gap to run for
			model.set(GRB_DoubleParam_MIPGap, MIP_GAP);

			// Run the model again
			model.optimize();
		}

		// Extract solution
		if (model.get(GRB_IntAttr_SolCount) > 0) {
			// Create arrays of doubles to collect results
			double** Xsol = new double*[N];
			double** Ysol = new double*[M];
			double** Zsol = new double*[N];
			double* Usol = new double[N];

			// Extract the results from the solver
			for(int i = 0; i < N; i++) {
				Xsol[i] = model.get(GRB_DoubleAttr_X, X[i], N);
			}
			for(int k = 0; k < M; k++) {
				Ysol[k] = model.get(GRB_DoubleAttr_X, Y[k], N);
			}
			for(int i = 0; i < N; i++) {
				Zsol[i] = model.get(GRB_DoubleAttr_X, Z[i], M);
			}
			Usol = model.get(GRB_DoubleAttr_X, U, N);

			// Print results
			double dist = 0;
			// Find all selected destination -> destination edges
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					if(Xsol[i][j] > 0.5) {
						if(DEBUG_MIP_FMDMTHPP) {
							printf(" %d -> %d\n", i, j);
						}
						dist += (solution->m_pVertexData + i)->GetDistanceTo(solution->m_pVertexData + j);

						// Added edge to solution
						int a = i;
						int b = j;
						if(a < b) {
							solution->m_pAdjMatrix[a][b] = true;
						}
						else {
							solution->m_pAdjMatrix[b][a] = true;
						}
					}
				}
			}

			// Find all selected depot -> destination edges
			for(int k = 0; k < M; k++) {
				for(int i = 0; i < N; i++) {
					if(Ysol[k][i] > 0.5) {
						if(DEBUG_MIP_FMDMTHPP) {
							printf(" %d: %d -> %d\n", k, N + M + k, i);
						}
						dist += solution->GetDepotOfPartion(k)->GetDistanceTo(solution->m_pVertexData + i);

						// Added edge to solution
						int a = i;
						int b = solution->GetDepotOfPartion(k)->nID;
						if(a < b) {
							solution->m_pAdjMatrix[a][b] = true;
						}
						else {
							solution->m_pAdjMatrix[b][a] = true;
						}
					}
				}
			}

			// Find all selected destination -> terminal edges
			for(int i = 0; i < N; i++) {
				for(int k = 0; k < M; k++) {
					if(Zsol[i][k] > 0.5) {
						if(DEBUG_MIP_FMDMTHPP) {
							printf(" %d: %d -> %d\n", k, i, N + k);
						}
						dist += (solution->m_pVertexData + i)->GetDistanceTo(solution->GetTerminalOfPartion(k));

						// Added edge to solution
						int a = i;
						int b = solution->GetTerminalOfPartion(k)->nID;
						if(a < b) {
							solution->m_pAdjMatrix[a][b] = true;
						}
						else {
							solution->m_pAdjMatrix[b][a] = true;
						}
					}
				}
			}
			if(DEBUG_MIP_FMDMTHPP) {
				printf(" dist = %0.3f\n", dist);
				printf("\nU:\n");
				for(int i = 0; i < N; i++) {
					printf(" %d: %.0f\n", i, Usol[i]);
				}
			}

			// Clean-up memory
			for(int i = 0; i < N; i++) {
				delete[] Xsol[i];
				delete[] Zsol[i];
			}
			for(int k = 0; k < M; k++) {
				delete[] Ysol[k];
			}
			delete[] Xsol;
			delete[] Ysol;
			delete[] Zsol;
			delete[] Usol;
		}

	} catch (GRBException& e) {
		fprintf(stderr, "[ERROR] : MIP_FMDMTHPP::Run_MIP(), caught error number: %d\n  msg: %s\n",
				e.getErrorCode(),  e.getMessage().c_str());
		exit(0);
	} catch (...) {
		fprintf(stderr, "[ERROR] : MIP_FMDMTHPP::Run_MIP(), Error during optimization\n");
		exit(0);
	}

	// Clean-up memory
	for(int i = 0; i < N; i++) {
		delete[] X[i];
		delete[] Z[i];
	}
	for (int k = 0; k < M; k++) {
		delete[] Y[k];
	}
	delete[] X;
	delete[] Y;
	delete[] Z;
	delete[] U;
}

//***********************************************************
// Protected Member Functions
//***********************************************************


//***********************************************************
// Private Member Functions
//***********************************************************
