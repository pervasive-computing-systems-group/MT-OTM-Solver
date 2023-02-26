#include "MILP_FMDMTHPP.h"

MILP_FMDMTHPP::MILP_FMDMTHPP() {
}

MILP_FMDMTHPP::~MILP_FMDMTHPP() {
}


//***********************************************************
// Public Member Functions
//***********************************************************

std::string MILP_FMDMTHPP::itos(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

int MILP_FMDMTHPP::U_d_k(int k) {
	// The math is 1-based but k is 0-based
	k++;
	return (L_M + 2)*(k - 1);
}

int MILP_FMDMTHPP::U_t_k(int k) {
	// The math is 1-based but k is 0-based
	k++;
	return k*(L_M + 1) + (k - 1);
}

/*
 * Formulate the MIP and run it on the Gurobi solver
 */
void MILP_FMDMTHPP::Run_MILP(Solution* solution) {
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
	// Elastic variables for deviation between tours
	GRBVar* Theta_HI = new GRBVar[M];
	GRBVar* Theta_LOW = new GRBVar[M];

	try {
		// Create Gurobi environment
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);

		/// Define variables
		// Create binary decision variables for each X_{i,j}
		for(int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->m_pVertexData + j);
					X[i][j][k] = model.addVar(0.0, 1.0, dist, GRB_BINARY, "x_"+itos(i)+"_"+itos(j)+"_"+itos(k));
				}
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

		// Create continuous variables for each Theta_HI_{k}
		for(int k = 0; k < M; k++) {
			Theta_HI[k] = model.addVar(0.0, GRB_INFINITY, THETA_HI_COEF, GRB_CONTINUOUS, "theta_hi_" + itos(k));
		}

		// Create continuous variables for each Theta_LOW_{k}
		for(int k = 0; k < M; k++) {
			Theta_LOW[k] = model.addVar(0.0, GRB_INFINITY, THETA_LOW_COEF, GRB_CONTINUOUS, "theta_low_" + itos(k));
		}

		/// Constraints
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


//		// Enforce numerical sequencing
//		for(int i = 0; i < N; i++) {
//			for(int j = 0; j < N; j++) {
//				for(int k = 0; k < M; k++) {
//					GRBLinExpr expr = 0;
//					expr += BIG_M*X[i][j][k] + U[i] - U[j] + 1;
//					model.addConstr(expr <= BIG_M, "U_"+itos(i)+"_to_U_"+itos(j)+"_on_"+itos(k));
//				}
//			}
//		}
//
//		// Enforce numbering scheme after depots
//		for(int i = 0; i < N; i++) {
//			for(int k = 0; k < M; k++) {
//				GRBLinExpr expr = 0;
//				expr += U[i] - U_d_k(k)*Y[k][i];
//				model.addConstr(expr >= 1, "U_"+itos(i)+"_if_Y"+itos(k)+"_"+itos(i));
//			}
//		}
//
//		// Enforce numbering scheme before terminals
//		for(int i = 0; i < N; i++) {
//			for(int k = 0; k < M; k++) {
//				GRBLinExpr expr = 0;
//				expr += U_t_k(k)*Z[i][k] - U[i] + BIG_M*(1 - Z[i][k]);
//				model.addConstr(expr >= 1, "U_"+itos(i)+"_if_Z"+itos(i)+"_"+itos(k));
//			}
//		}


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

		// Enforce equal length tours
		for(int k = 0; k < M; k++) {
			GRBLinExpr expr = 0;
			// LHS
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->m_pVertexData + j);
					expr += dist*X[i][j][k];
				}
			}
			for(int i = 0; i < N; i++) {
				double dist = solution->GetDepotOfPartion(k)->GetDistanceTo(solution->m_pVertexData + i);
				expr += dist*Y[k][i];
			}
			for(int i = 0; i < N; i++) {
				double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->GetTerminalOfPartion(k));
				expr += dist*Z[i][k];
			}
			expr += Theta_LOW[k];
			expr -= Theta_HI[k];

			// - RHS
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					for(int _k = 0; _k < M; _k++) {
						double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->m_pVertexData + j);
						expr -= (1.0/M)*dist*X[i][j][_k];
					}
				}
			}
			for(int i = 0; i < N; i++) {
				for(int _k = 0; _k < M; _k++) {
					double dist = solution->GetDepotOfPartion(k)->GetDistanceTo(solution->m_pVertexData + i);
					expr -= (1.0/M)*dist*Y[_k][i];
				}
			}
			for(int i = 0; i < N; i++) {
				for(int _k = 0; _k < M; _k++) {
					double dist = (solution->m_pVertexData + i)->GetDistanceTo(solution->GetTerminalOfPartion(k));
					expr -= (1.0/M)*dist*Z[i][_k];
				}
			}

			// == 0
			model.addConstr(expr == 0, "Dist_"+itos(k));
		}

		// Set branching decision to Pseudo Reduced Cost Branching (VarBranch  0) GRB_INT_PAR_SOLUTIONLIMIT
		model.set(GRB_IntParam_VarBranch, 0);

		// Set a time-out limit. If we hit the time-out, then we'll set a MIP-gap limit
		model.set(GRB_DoubleParam_TimeLimit, 100);

		// Optimize model
		model.optimize();

		// Check the status of the model to see if we timed-out
		if(model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
			// Set maximum time-out to 900 seconds
			model.set(GRB_DoubleParam_TimeLimit, 10700);
			// Set a MIP-Gap to run for
			model.set(GRB_DoubleParam_MIPGap, MILP_GAP);

			// Run the model again
			model.optimize();

			// Check the status of the model to see if we timed-out again
			if(model.get(GRB_IntAttr_Status) == GRB_TIME_LIMIT) {
				fprintf(stderr, "[ERROR] : MIP_FMDMTHPP::Run_MIP(), couldn't find solution after 10800s (3 hours)\n");
				exit(0);
			}
		}

		// Extract solution
		if (model.get(GRB_IntAttr_SolCount) > 0) {
			// Create arrays of doubles to collect results
			double*** Xsol = new double**[N];
			double** Ysol = new double*[M];
			double** Zsol = new double*[N];
			double* Usol = new double[N];
			double* THsol = new double[M];
			double* TLsol = new double[M];

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
			THsol = model.get(GRB_DoubleAttr_X, Theta_HI, M);
			TLsol = model.get(GRB_DoubleAttr_X, Theta_LOW, M);

			// Print results
			for(int k = 0; k < M; k++) {
				double dist = 0;
				for(int i = 0; i < N; i++) {
					for(int j = 0; j < N; j++) {
						if(Xsol[i][j][k] > 0.5) {
							if(DEBUG_MILP_FMDMTHPP) {
								printf(" %d: %d -> %d\n", k, i, j);
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
				for(int i = 0; i < N; i++) {
					if(Ysol[k][i] > 0.5) {
						if(DEBUG_MILP_FMDMTHPP) {
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
				for(int i = 0; i < N; i++) {
					if(Zsol[i][k] > 0.5) {
						if(DEBUG_MILP_FMDMTHPP) {
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
				if(DEBUG_MILP_FMDMTHPP) {
					printf(" %d: dist = %0.3f\n", k, dist);
				}
			}

			if(DEBUG_MILP_FMDMTHPP) {
				printf("\nU:\n");
				for(int i = 0; i < N; i++) {
					printf(" %d: %.0f\n", i, Usol[i]);
				}

				printf("\nTheta_HI/LOW:\n");
				for(int k = 0; k < M; k++) {
					printf(" %d: H:%.3f, L:%.3f\n", k, THsol[k], TLsol[k]);
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
	delete[] Theta_HI;
	delete[] Theta_LOW;
}

//***********************************************************
// Protected Member Functions
//***********************************************************


//***********************************************************
// Private Member Functions
//***********************************************************
