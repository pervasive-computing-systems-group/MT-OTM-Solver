#include "FMdMtHPP_NLP.h"

FMdMtHPP_NLP::FMdMtHPP_NLP() {
}

FMdMtHPP_NLP::~FMdMtHPP_NLP() {
}

//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

/*
 * Runs a hybrid iterative sweeping approach
 */
void FMdMtHPP_NLP::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Running FMdMtHPP NLP *\n\n");

	// Function variables
	N = solution->m_nN;
	M = solution->m_nM;
	L_M =(N - M + 1);
	BIG_M = (M*L_M + 2*M - 1);

	// Gurobi variables
	GRBEnv* env = NULL;
	// Tracks edges between way-points
	GRBVar*** X = new GRBVar**[N];
	for(int i = 0; i < N; i++) {
		X[i] = new GRBVar*[N];
		for(int j = 0; j < N; j++) {
			X[i][j] = new GRBVar[M];
		}
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
	// Velocity for each sub-tour
	GRBVar* V = new GRBVar[M];
	// Length of each sub-tour
	GRBVar* L = new GRBVar[M];
	// Auxiliary variable
	GRBVar* W = new GRBVar[M];

	try {
		// Create Gurobi environment
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);

		/// Define variables
		// Create binary decision variables for each X_{i,j}
		for(int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					X[i][j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "x_"+itos(i)+"_"+itos(j)+"_"+itos(k));
				}
			}
		}

		// Create binary decision variables for each Y_{k,i}
		for(int k = 0; k < M; k++) {
			for (int i = 0; i < N; i++) {
				Y[k][i] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "y_"+itos(k)+"_"+itos(i));
			}
		}

		// Create binary decision variables for each Z_{i,k}
		for(int i = 0; i < N; i++) {
			for (int k = 0; k < M; k++) {
				Z[i][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "z_"+itos(i)+"_"+itos(k));
			}
		}

		// Create integer variables for each U_{i}
		for(int i = 0; i < N; i++) {
			U[i] = model.addVar(0.0, BIG_M, 0, GRB_INTEGER, "u_" + itos(i));
		}

		// Create continuous variables for each V_{k}
		for(int k = 0; k < M; k++) {
			V[k] = model.addVar(0.0, V_MAX, 0, GRB_CONTINUOUS, "v_" + itos(k));
		}

		// Create continuous variables for each L_{k}
		for(int k = 0; k < M; k++) {
			L[k] = model.addVar(0.0, DIST_MAX, 0, GRB_CONTINUOUS, "l_" + itos(k));
		}

		// Create continuous auxiliary variables
		for(int k = 0; k < M; k++) {
			W[k] = model.addVar(0.0, GRB_INFINITY, 0, GRB_CONTINUOUS, "w_" + itos(k));
		}

		/// Objective function
		GRBQuadExpr ObjExpr = 0;
		for(int k = 0; k < M; k++) {
			ObjExpr += W[k];
		}
		model.setObjective(ObjExpr, GRB_MINIMIZE);

		/// Constraints
		// Bound on L_k
		for(int k = 0; k < M; k++) {
			GRBLinExpr expr = 0;
			// Add up length of tour
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					expr += solution->m_pVertexData[i].GetDistanceTo(solution->m_pVertexData + j)*X[i][j][k];
				}
			}
			for(int i = 0; i < N; i++) {
				expr += solution->GetDepotOfPartion(k)->GetDistanceTo(solution->m_pVertexData + i)*Y[k][i];
			}
			for(int i = 0; i < N; i++) {
				expr += solution->m_pVertexData[i].GetDistanceTo(solution->GetTerminalOfPartion(k))*Z[i][k];
			}
			// Subtract L_k
			expr -= L[k];

			model.addConstr(expr <= 0, "L_"+itos(k)+"_geq_dist_"+itos(k));
		}

		// Bound on V_k
		for(int k = 0; k < M; k++) {
			GRBQuadExpr expr = V[k]*V[k] - 2*C4*V[k] + (C2/pow(C3,2))*L[k] + pow(C4,2) - (C1/pow(C3,2));
			model.addQConstr(expr <= 0, "V_"+itos(k)+"_leq_v(d_"+itos(k)+")");
		}

		// Enforce number sequencing (lower bound)
		for(int i = 0; i < N; i++) {
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					GRBLinExpr expr = 1 - BIG_M*(1 - X[i][j][k]) + U[i] - U[j];
					model.addConstr(expr <= 0, "U_"+itos(i)+"_to_U_"+itos(j)+"_LB_on_"+itos(k));
				}
			}
		}

		// Enforce number sequencing (upper bound)
		for(int i = 0; i < N; i++) {
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					GRBLinExpr expr = 1 + BIG_M*(1 - X[i][j][k]) + U[i] - U[j];
					model.addConstr(expr >= 0, "U_"+itos(i)+"_to_U_"+itos(j)+"_UB_on_"+itos(k));
				}
			}
		}

		// Enforce numbering scheme after depots (lower bound)
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 1 - U[i] + U_d_k(k)*Y[k][i];
				model.addConstr(expr <= 0, "U_"+itos(i)+"_if_Y"+itos(k)+"_"+itos(i)+"_LB");
			}
		}

		// Enforce numbering scheme after depots (upper bound)
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 1 + BIG_M*(1 - Y[k][i]) - U[i] + U_d_k(k)*Y[k][i];
				model.addConstr(expr >= 0, "U_"+itos(i)+"_if_Y"+itos(k)+"_"+itos(i)+"_UB");
			}
		}

		// The number of edges going into a stop must equal the number leaving the stop for each tour
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 0;

				// Edges going in
				for(int j = 0; j < N; j++) {
					expr += X[j][i][k];
				}
				expr += Y[k][i];

				// Edges going out
				for(int j = 0; j < N; j++) {
					expr -= X[i][j][k];
				}
				expr -= Z[i][k];

				model.addConstr(expr == 0, "d(IN)_eq_d(OUT)_"+itos(i)+"on"+itos(k));
			}
		}

		// Degree-2 constraint on all way-points
		for(int i = 0; i < N; i++) {
			GRBLinExpr expr = 0;
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					expr += X[i][j][k];
				}
			}
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					expr += X[j][i][k];
				}
			}
			for(int k = 0; k < M; k++) {
				expr += Y[k][i];
			}
			for(int k = 0; k < M; k++) {
				expr += Z[i][k];
			}
			model.addConstr(expr == 2, "deg2_"+itos(i));
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

		// Link W_k with L_k/V_k
		// TODO: Can this be W_k >= L_k/V_k ?
		for(int k = 0; k < M; k++) {
			GRBQuadExpr expr = W[k]*V[k]-L[k];
			model.addQConstr(expr == 0, "W_"+itos(k));
		}

		// Tell Gurobi that this is a non-convex problem
		model.set(GRB_IntParam_NonConvex, 2);

		// Favor RLT and Flow-Cover cuts
		model.set(GRB_INT_PAR_RLTCUTS, "2");
		model.set(GRB_INT_PAR_FLOWCOVERCUTS, "2");

		// Optimize model
		model.optimize();

		// Extract solution
		if (model.get(GRB_IntAttr_SolCount) > 0) {
			// Create arrays of doubles to collect results
			double*** Xsol = new double**[N];
			double** Ysol = new double*[M];
			double** Zsol = new double*[N];
			double* Usol = new double[N];
			double* Lsol = new double[M];
			double* Vsol = new double[M];

			// Extract the results from the solver
			for(int i = 0; i < N; i++) {
				Xsol[i] = new double*[N];
				for(int j = 0; j < N; j++) {
					Xsol[i][j] = model.get(GRB_DoubleAttr_X, X[i][j], M);
				}
			}
			for(int k = 0; k < M; k++) {
				Ysol[k] = model.get(GRB_DoubleAttr_X, Y[k], N);
			}
			for(int i = 0; i < N; i++) {
				Zsol[i] = model.get(GRB_DoubleAttr_X, Z[i], M);
			}
			Usol = model.get(GRB_DoubleAttr_X, U, N);
			Lsol = model.get(GRB_DoubleAttr_X, L, M);
			Vsol = model.get(GRB_DoubleAttr_X, V, M);

			// Print results
			for(int k = 0; k < M; k++) {
				double dist = 0;
				for(int i = 0; i < N; i++) {
					for(int j = 0; j < N; j++) {
						if(Xsol[i][j][k] > 0.5) {
							if(DEBUG_MT_NLP)
								printf(" %d: %d -> %d\n", k, i, j);
							dist += solution->m_pVertexData[i].GetDistanceTo(solution->m_pVertexData + j);

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
				for(int i = 0; i < N; i++) {
					if(Ysol[k][i] > 0.5) {
						if(DEBUG_MT_NLP)
							printf(" %d: %d -> %d\n", k, N + M + k, i);
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
				for(int i = 0; i < N; i++) {
					if(Zsol[i][k] > 0.5) {
						if(DEBUG_MT_NLP)
							printf(" %d: %d -> %d\n", k, i, N + k);
						dist += solution->m_pVertexData[i].GetDistanceTo(solution->GetTerminalOfPartion(k));

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
				if(DEBUG_MT_NLP)
					printf(" %d: dist = %0.3f\n", k, dist);
			}

			if(DEBUG_MT_NLP) {
				printf("\nU:\n");
				for(int i = 0; i < N; i++) {
					printf(" %d: %.0f\n", i, Usol[i]);
				}

				printf("\nTour lengths and Velocities:\n");
				for(int k = 0; k < M; k++) {
					printf(" %d: L:%.3f, V:%.3f\n", k, Lsol[k], Vsol[k]);
				}
			}

			// Clean-up memory
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					delete[] Xsol[i][j];
				}
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
			delete[] Lsol;
			delete[] Vsol;
		}
		else {
			solution->m_bFeasible = false;
		}

	} catch (GRBException& e) {
		printf("Error number: %d\n", e.getErrorCode());
		printf(" %s\n", e.getMessage().c_str());
	} catch (...) {
		printf("Error during optimization");
	}

	// Clean-up memory
	for(int i = 0; i < N; i++) {
		for(int j = 0; j < N; j++) {
			delete[] X[i][j];
		}
		delete[] X[i];
	}
	delete[] X;
	for (int k = 0; k < M; k++) {
		delete[] Y[k];
	}
	delete[] Y;
	for (int i = 0; i < N; i++) {
		delete[] Z[i];
	}
	delete[] Z;
	delete[] U;
	delete[] L;
	delete[] V;
}

//***********************************************************
// Private Member Functions
//***********************************************************


int FMdMtHPP_NLP::U_d_k(int k) {
	// The math is 1-based but k is 0-based
	k++;
	return (L_M + 2)*(k - 1);
}

