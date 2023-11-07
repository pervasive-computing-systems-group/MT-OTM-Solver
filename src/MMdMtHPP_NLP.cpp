#include "MMdMtHPP_NLP.h"

MMdMtHPP_NLP::MMdMtHPP_NLP() {
}

MMdMtHPP_NLP::~MMdMtHPP_NLP() {
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
void MMdMtHPP_NLP::RunAlgorithm(Solution* solution) {
	if(SANITY_PRINT)
		printf("\n * Running FMdMtHPP NLP *\n\n");

	/*
	 * Sets:
	 *
	 * Variables:
	 */

	// Function variables
	N = solution->m_nN;
	M = solution->m_nM;
	L_M =(N - M + 1);
	BIG_M = (M*L_M + 2*M - 1);

	// Gurobi variables
	GRBEnv* env = NULL;
	// Tracks edges between way-points
	GRBVar*** E_ijk = new GRBVar**[N];
	for(int i = 0; i < N; i++) {
		E_ijk[i] = new GRBVar*[N];
		for(int j = 0; j < N; j++) {
			E_ijk[i][j] = new GRBVar[M];
		}
	}
	// Tracks edges between depots and way-points
	GRBVar** Dd_ki = new GRBVar*[M];
	for (int k = 0; k < M; k++) {
		Dd_ki[k] = new GRBVar[N];
	}
	// Tracks edges between way-points and terminals
	GRBVar** Dt_jk = new GRBVar*[N];
	for (int i = 0; i < N; i++) {
		Dt_jk[i] = new GRBVar[M];
	}
	// Used to assign numbers to each location
	GRBVar* U_i = new GRBVar[N];
	// Velocity for each sub-tour
	GRBVar* S_k = new GRBVar[M];
	// Length of each sub-tour
	GRBVar* L_k = new GRBVar[M];
	// Start time (depot) of each sub-tour
	GRBVar* Td_k = new GRBVar[M];
	// End time (terminal) of each sub-tour
	GRBVar* Tt_k = new GRBVar[M];
	// Length from depot k to waypoint i
	GRBVar** Ld_ki = new GRBVar*[M];
	for(int k = 0; k < M; k++) {
		Ld_ki[k] = new GRBVar[N];
	}
	// Length from waypoint j to depot k
	GRBVar** Lt_jk = new GRBVar*[N];
	for(int j = 0; j < N; j++) {
		Lt_jk[j] = new GRBVar[M];
	}
	// X-coordinate of depot k
	GRBVar* Xd_k = new GRBVar[M];
	// Y-coordinate of depot k
	GRBVar* Yd_k = new GRBVar[M];
	// X-coordinate of terminal k
	GRBVar* Xt_k = new GRBVar[M];
	// Y-coordinate of terminal k
	GRBVar* Yt_k = new GRBVar[M];

	// Needed if we use a sin function for UGV path
	GRBVar* Zf_d_k = new GRBVar[M];
	GRBVar* Zf_t_k = new GRBVar[M];
	GRBVar* Zr_d_k = new GRBVar[M];
	GRBVar* Zr_t_k = new GRBVar[M];


	try {
		// Create Gurobi environment
		env = new GRBEnv();
		GRBModel model = GRBModel(*env);

		/// Define variables
		// Create binary decision variables for each E_{i,j}
		for(int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					E_ijk[i][j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "E_"+itos(i)+"_"+itos(j)+"_"+itos(k));
				}
			}
		}

		// Create binary decision variables for each Dd_{k,i}
		for(int k = 0; k < M; k++) {
			for (int i = 0; i < N; i++) {
				Dd_ki[k][i] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "Dd_"+itos(k)+"_"+itos(i));
			}
		}

		// Create binary decision variables for each Dt_{i,k}
		for(int j = 0; j < N; j++) {
			for (int k = 0; k < M; k++) {
				Dt_jk[j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "Dt_"+itos(j)+"_"+itos(k));
			}
		}

		// Create integer variables for each U_{i}
		for(int i = 0; i < N; i++) {
			U_i[i] = model.addVar(0.0, BIG_M, 0, GRB_INTEGER, "U_" + itos(i));
		}

		// Create continuous variables for each S_{k}
		for(int k = 0; k < M; k++) {
			S_k[k] = model.addVar(0.0, V_MAX, 0, GRB_CONTINUOUS, "S_" + itos(k));
		}

		// Create continuous variables for each L_{k}
		for(int k = 0; k < M; k++) {
			L_k[k] = model.addVar(0.0, DIST_MAX, 0, GRB_CONTINUOUS, "L_" + itos(k));
		}

		// Create start time variable for each sub-tour
		for(int k = 0; k < M; k++) {
			Td_k[k] = model.addVar(0.0, GRB_INFINITY, 0, GRB_CONTINUOUS, "Td_" + itos(k));
		}

		// Create end time variable for each sub-tour
		for(int k = 0; k < M; k++) {
			Tt_k[k] = model.addVar(0.0, GRB_INFINITY, 0, GRB_CONTINUOUS, "Tt_" + itos(k));
		}

		// Create continuous variables for each Ld_{ki}
		for(int k = 0; k < M; k++) {
			for(int i = 0; i < N; i++) {
				Ld_ki[k][i] = model.addVar(0.0, GRB_INFINITY, 0, GRB_CONTINUOUS, "Ld_"+itos(k)+"_"+itos(i));
			}
		}

		// Create continuous variables for each Lt_{jk}
		for(int j = 0; j < N; j++) {
			for(int k = 0; k < M; k++) {
				Lt_jk[j][k] = model.addVar(0.0, GRB_INFINITY, 0, GRB_CONTINUOUS, "Lt_"+itos(j)+"_"+itos(k));
			}
		}

		// Create depot x-coord variables
		for(int k = 0; k < M; k++) {
			Xd_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Xd_" + itos(k));
		}

		// Create depot y-coord variables
		for(int k = 0; k < M; k++) {
			Yd_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Yd_" + itos(k));
		}

		// Create depot x-coord variables
		for(int k = 0; k < M; k++) {
			Xt_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Xt_" + itos(k));
		}

		// Create depot y-coord variables
		for(int k = 0; k < M; k++) {
			Yt_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Yt_" + itos(k));
		}


		/// Objective function
		GRBQuadExpr ObjExpr = Tt_k[M-1];
		model.setObjective(ObjExpr, GRB_MINIMIZE);


		/// Constraints
		// Bound on L_k
		for(int k = 0; k < M; k++) {
			GRBQuadExpr expr = 0;
			// Add up length of tour
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					expr += solution->m_pVertexData[i].GetDistanceTo(solution->m_pVertexData + j)*E_ijk[i][j][k];
				}
			}
			for(int i = 0; i < N; i++) {
				expr += Ld_ki[k][i]*Dd_ki[k][i];
			}
			for(int j = 0; j < N; j++) {
				expr += Lt_jk[j][k]*Dt_jk[j][k];
			}
			// Subtract L_k
			expr -= L_k[k];

			model.addQConstr(expr == 0, "L_"+itos(k)+"_geq_dist_"+itos(k));
		}

		// Bound on each Ld_ki. For each depot k ...
		for(int k = 0; k < M; k++) {
			// For each waypoint i
			for(int i = 0; i < N; i++) {
				double x_i = solution->m_pVertexData[i].fX;
				double y_i = solution->m_pVertexData[i].fY;
				GRBQuadExpr expr = 0;
				expr += Xd_k[k]*Xd_k[k] - 2*x_i*Xd_k[k] + x_i*x_i;
				expr += Yd_k[k]*Yd_k[k] - 2*y_i*Yd_k[k] + y_i*y_i;
				expr -= Ld_ki[k][i]*Ld_ki[k][i];
				// Add constraint to solver
				model.addQConstr(expr == 0, "Ld_"+itos(k)+"_"+itos(i));
			}
		}

		// Bound on each Lt_jk. For each waypoint i ...
		for(int j = 0; j < N; j++) {
			// For each terminal k
			for(int k = 0; k < M; k++) {
				double x_j = solution->m_pVertexData[j].fX;
				double y_j = solution->m_pVertexData[j].fY;
				GRBQuadExpr expr = 0;
				expr += Xt_k[k]*Xt_k[k] - 2*x_j*Xt_k[k] + x_j*x_j;
				expr += Yt_k[k]*Yt_k[k] - 2*y_j*Yt_k[k] + y_j*y_j;
				expr -= Lt_jk[j][k]*Lt_jk[j][k];
				// Add constraint to solver
				model.addQConstr(expr == 0, "Lt_"+itos(j)+"_"+itos(k));
			}
		}


		// Constraints based on the ground vehicle movement function p_d(f)
		switch(solution->m_tBSTrajectory.pd_type) {
		case E_TrajFuncType::e_StraightLine:
			// For each depot k
			for(int k = 0; k < M; k++) {
				// Constrain X-coord
				GRBLinExpr x_expr = 0;
				x_expr += Td_k[k]*solution->m_tBSTrajectory.mX - Xd_k[k];
				// Add constraint to solver
				model.addConstr(x_expr == 0, "Xd_"+itos(k));
				// Constrain Y-coord
				GRBLinExpr y_expr = 0;
				y_expr += Td_k[k]*solution->m_tBSTrajectory.mY - Yd_k[k];
				// Add constraint to solver
				model.addConstr(y_expr == 0, "Yd_"+itos(k));
			}

			// For each terminal k
			for(int k = 0; k < M; k++) {
				// Constrain X-coord
				GRBLinExpr x_expr = 0;
				x_expr += Tt_k[k]*solution->m_tBSTrajectory.mX - Xt_k[k];
				// Add constraint to solver
				model.addConstr(x_expr == 0, "Xt_"+itos(k));
				// Constrain Y-coord
				GRBLinExpr y_expr = 0;
				y_expr += Tt_k[k]*solution->m_tBSTrajectory.mY - Yt_k[k];
				// Add constraint to solver
				model.addConstr(y_expr == 0, "Yt_"+itos(k));
			}

			break;

		case E_TrajFuncType::e_Sinusoidal:
			// Add auxiliary variables
			for(int k = 0; k < M; k++) {
				Zf_d_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Zf_d_k" + itos(k));
				Zf_t_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Zf_t_k" + itos(k));
				Zr_d_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Zr_d_k" + itos(k));
				Zr_t_k[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "Zr_t_k" + itos(k));
			}

			// For each depot k
			for(int k = 0; k < M; k++) {
				/// x = a t, y = b sin((2pi t)/c)
				// Constrain X-coord
				GRBLinExpr x_expr = 0;
				x_expr += Td_k[k]*solution->m_tBSTrajectory.mX - Xd_k[k];
				// Add constraint to solver
				model.addConstr(x_expr == 0, "Xd_"+itos(k));

				/// Constrain Y-coord
				// Zf_d_k = (2pi t)/c
				GRBLinExpr Zf_d_expr = 0;
				Zf_d_expr += (2*PI*Td_k[k])/solution->m_tBSTrajectory.mC - Zf_d_k[k];
				model.addConstr(Zf_d_expr == 0, "Zf_d_expr_"+itos(k));
				// Zr_d_k = y/b
				GRBLinExpr Zr_d_expr = 0;
				Zr_d_expr += Yd_k[k]/solution->m_tBSTrajectory.mY - Zr_d_k[k];
				model.addConstr(Zr_d_expr == 0, "Zr_d_expr_"+itos(k));
				// Add actual y constraint
				model.addGenConstrSin(Zf_d_k[k], Zr_d_k[k], "Yd_"+itos(k));
			}

			// For each terminal k
			for(int k = 0; k < M; k++) {
				/// x = a t, y = b sin((2pi t)/c)
				// Constrain X-coord
				GRBLinExpr x_expr = 0;
				x_expr += Tt_k[k]*solution->m_tBSTrajectory.mX - Xt_k[k];
				// Add constraint to solver
				model.addConstr(x_expr == 0, "Xt_"+itos(k));

				/// Constrain Y-coord
				// Zf_d_k = (2pi t)/c
				GRBLinExpr Zf_t_expr = 0;
				Zf_t_expr += (2*PI*Tt_k[k])/solution->m_tBSTrajectory.mC - Zf_t_k[k];
				model.addConstr(Zf_t_expr == 0, "Zf_t_expr_"+itos(k));
				// Zr_d_k = y/b
				GRBLinExpr Zr_t_expr = 0;
				Zr_t_expr += Yt_k[k]/solution->m_tBSTrajectory.mY - Zr_t_k[k];
				model.addConstr(Zr_t_expr == 0, "Zr_t_expr_"+itos(k));
				// Add actual y constraint
				model.addGenConstrSin(Zf_t_k[k], Zr_t_k[k], "Yt_"+itos(k));
			}

			break;

		default:
			// Un-implemented UGV function p_d()
			printf("[ERROR] : MMdMtHPP_NLP::RunAlgorithm : Given non-linear UGV, expected linear\n");
			exit(0);
		}

		// Bound on launch (depot) time - start at k=1
		for(int k = 1; k < M; k++) {
			GRBLinExpr expr = Tt_k[k-1] + BATTERY_SWAP_TIME - Td_k[k];
			model.addConstr(expr <= 0, "Td_"+itos(k)+"_geq_Tt_"+itos(k-1));
		}

		// Bound on land (terminal) time
		for(int k = 0; k < M; k++) {
			GRBQuadExpr expr = Td_k[k]*S_k[k] + L_k[k] - Tt_k[k]*S_k[k];
			model.addQConstr(expr == 0, "Tt_"+itos(k)+"_eq_Td_"+itos(k-1));
		}

		// Bound on the first launch (depot) time
		{
			GRBLinExpr expr = Td_k[0];
			model.addConstr(expr >= 0, "Td_0_geq_0");
		}

		// Bound on S_k
		for(int k = 0; k < M; k++) {
			GRBQuadExpr expr = S_k[k]*S_k[k] - 2*C4*S_k[k] + (C2/pow(C3,2))*L_k[k] + pow(C4,2) - (C1/pow(C3,2));
			model.addQConstr(expr <= 0, "V_"+itos(k)+"_leq_v(d_"+itos(k)+")");
		}

		// Enforce number sequencing (lower bound)
		for(int i = 0; i < N; i++) {
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					GRBLinExpr expr = 1 - BIG_M*(1 - E_ijk[i][j][k]) + U_i[i] - U_i[j];
					model.addConstr(expr <= 0, "U_"+itos(i)+"_to_U_"+itos(j)+"_LB_on_"+itos(k));
				}
			}
		}

		// Enforce number sequencing (upper bound)
		for(int i = 0; i < N; i++) {
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					GRBLinExpr expr = 1 + BIG_M*(1 - E_ijk[i][j][k]) + U_i[i] - U_i[j];
					model.addConstr(expr >= 0, "U_"+itos(i)+"_to_U_"+itos(j)+"_UB_on_"+itos(k));
				}
			}
		}

		// Enforce numbering scheme after depots (lower bound)
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 1 - U_i[i] + U_d_k(k)*Dd_ki[k][i];
				model.addConstr(expr <= 0, "U_"+itos(i)+"_if_Y"+itos(k)+"_"+itos(i)+"_LB");
			}
		}

		// Enforce numbering scheme after depots (upper bound)
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 1 + BIG_M*(1 - Dd_ki[k][i]) - U_i[i] + U_d_k(k)*Dd_ki[k][i];
				model.addConstr(expr >= 0, "U_"+itos(i)+"_if_Y"+itos(k)+"_"+itos(i)+"_UB");
			}
		}

		// The number of edges going into a stop must equal the number leaving the stop for each tour
		for(int i = 0; i < N; i++) {
			for(int k = 0; k < M; k++) {
				GRBLinExpr expr = 0;

				// Edges going in
				for(int j = 0; j < N; j++) {
					expr += E_ijk[j][i][k];
				}
				expr += Dd_ki[k][i];

				// Edges going out
				for(int j = 0; j < N; j++) {
					expr -= E_ijk[i][j][k];
				}
				expr -= Dt_jk[i][k];

				model.addConstr(expr == 0, "d(IN)_eq_d(OUT)_"+itos(i)+"on"+itos(k));
			}
		}

		// Degree-2 constraint on all way-points
		for(int i = 0; i < N; i++) {
			GRBLinExpr expr = 0;
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					expr += E_ijk[i][j][k];
				}
			}
			for(int j = 0; j < N; j++) {
				for(int k = 0; k < M; k++) {
					expr += E_ijk[j][i][k];
				}
			}
			for(int k = 0; k < M; k++) {
				expr += Dd_ki[k][i];
			}
			for(int k = 0; k < M; k++) {
				expr += Dt_jk[i][k];
			}
			model.addConstr(expr == 2, "deg2_"+itos(i));
		}

		// Ensure that each depot has one tour
		for(int k = 0; k < M; k++) {
			GRBLinExpr expr = 0;
			for(int i = 0; i < N; i++) {
				expr += Dd_ki[k][i];
			}
			model.addConstr(expr == 1, "Dd_"+itos(k));
		}

		// Ensure that each terminal has one tour
		for(int k = 0; k < M; k++) {
			GRBLinExpr expr = 0;
			for(int i = 0; i < N; i++) {
				expr += Dt_jk[i][k];
			}
			model.addConstr(expr == 1, "Dt_"+itos(k));
		}


		/// Gurobi settings
		// Tell Gurobi that this is a non-convex problem
		model.set(GRB_IntParam_NonConvex, 2);

//		// Favor RLT and Flow-Cover cuts
//		model.set(GRB_INT_PAR_RLTCUTS, "2");
//		model.set(GRB_INT_PAR_FLOWCOVERCUTS, "2");


		/// Run model
		model.optimize();


		/// Extract solution
		if (model.get(GRB_IntAttr_SolCount) > 0) {
			// Reset the solution
			solution->m_Td_k.clear();
			solution->m_Tt_k.clear();

			// Create arrays of doubles to collect results
			double*** Eijk_sol = new double**[N];
			double** Dd_ki_sol = new double*[M];
			double** Dt_jk_sol = new double*[N];
			double* Ui_sol = new double[N];
			double* Lk_sol = new double[M];
			double* Sk_sol = new double[M];
			double* Td_k_sol = new double[M];
			double* Tt_k_sol = new double[M];
			double** Ld_ki_sol = new double*[M];
			double** Lt_jk_sol = new double*[N];
			double* Xd_k_sol = new double[M];
			double* Yd_k_sol = new double[M];
			double* Xt_k_sol = new double[M];
			double* Yt_k_sol = new double[M];

			// Extract the results from the solver
			for(int i = 0; i < N; i++) {
				Eijk_sol[i] = new double*[N];
				for(int j = 0; j < N; j++) {
					Eijk_sol[i][j] = model.get(GRB_DoubleAttr_X, E_ijk[i][j], M);
				}
			}
			for(int k = 0; k < M; k++) {
				Dd_ki_sol[k] = model.get(GRB_DoubleAttr_X, Dd_ki[k], N);
			}
			for(int j = 0; j < N; j++) {
				Dt_jk_sol[j] = model.get(GRB_DoubleAttr_X, Dt_jk[j], M);
			}
			Ui_sol = model.get(GRB_DoubleAttr_X, U_i, N);
			Lk_sol = model.get(GRB_DoubleAttr_X, L_k, M);
			Sk_sol = model.get(GRB_DoubleAttr_X, S_k, M);
			Td_k_sol = model.get(GRB_DoubleAttr_X, Td_k, M);
			Tt_k_sol = model.get(GRB_DoubleAttr_X, Tt_k, M);
			for(int k = 0; k < M; k++) {
				Ld_ki_sol[k] = model.get(GRB_DoubleAttr_X, Ld_ki[k], N);
			}
			for(int k = 0; k < M; k++) {
				Lt_jk_sol[k] = model.get(GRB_DoubleAttr_X, Ld_ki[k], N);
			}
			Xd_k_sol = model.get(GRB_DoubleAttr_X, Xd_k, M);
			Yd_k_sol = model.get(GRB_DoubleAttr_X, Yd_k, M);
			Xt_k_sol = model.get(GRB_DoubleAttr_X, Xt_k, M);
			Yt_k_sol = model.get(GRB_DoubleAttr_X, Yt_k, M);


			// Print results
			for(int k = 0; k < M; k++) {
				double dist = 0;

				if(DEBUG_MUGV_NLP) {
					printf(" Xd_%d,Yd_%d = (%0.3f,%0.3f)\n",k,k,Xd_k_sol[k],Yd_k_sol[k]);
					printf(" Xt_%d,Yt_%d = (%0.3f,%0.3f)\n",k,k,Xt_k_sol[k],Yt_k_sol[k]);
					printf(" Td_%d,Tt_%d = (%0.3f,%0.3f)\n",k,k,Td_k_sol[k],Tt_k_sol[k]);
					printf(" calc-dist = %0.3f, L_%d = %0.3f\n", dist, k, Lk_sol[k]);
				}

				// We solved for the times and depot locations... store those
				solution->m_Td_k.push_back(Td_k_sol[k]);
				solution->m_Tt_k.push_back(Tt_k_sol[k]);
				solution->GetDepotOfPartion(k)->fX = Xd_k_sol[k];
				solution->GetDepotOfPartion(k)->fY = Yd_k_sol[k];
				solution->GetTerminalOfPartion(k)->fX = Xt_k_sol[k];
				solution->GetTerminalOfPartion(k)->fY = Yt_k_sol[k];

				for(int i = 0; i < N; i++) {
					for(int j = 0; j < N; j++) {
						if(Eijk_sol[i][j][k] > 0.5) {
							if(DEBUG_MUGV_NLP)
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
					if(Dd_ki_sol[k][i] > 0.5) {
						if(DEBUG_MUGV_NLP)
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
					if(Dt_jk_sol[i][k] > 0.5) {
						if(DEBUG_MUGV_NLP)
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

			}

			if(DEBUG_MUGV_NLP) {
				printf("\nU:\n");
				for(int i = 0; i < N; i++) {
					printf(" %d: %.0f\n", i, Ui_sol[i]);
				}

				printf("\nTour lengths and Velocities:\n");
				for(int k = 0; k < M; k++) {
					printf(" %d: L_k:%.3f, V_k:%.3f\n", k, Lk_sol[k], Sk_sol[k]);
				}
			}

			// Clean-up memory
			for(int i = 0; i < N; i++) {
				for(int j = 0; j < N; j++) {
					delete[] Eijk_sol[i][j];
				}
				delete[] Eijk_sol[i];
				delete[] Dt_jk_sol[i];
			}
			for(int k = 0; k < M; k++) {
				delete[] Dd_ki_sol[k];
			}
			delete[] Eijk_sol;
			delete[] Dd_ki_sol;
			delete[] Dt_jk_sol;
			delete[] Ui_sol;
			delete[] Lk_sol;
			delete[] Sk_sol;
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
			delete[] E_ijk[i][j];
		}
		delete[] E_ijk[i];
	}
	delete[] E_ijk;
	for (int k = 0; k < M; k++) {
		delete[] Dd_ki[k];
	}
	delete[] Dd_ki;
	for (int i = 0; i < N; i++) {
		delete[] Dt_jk[i];
	}
	delete[] Dt_jk;
	delete[] U_i;
	delete[] L_k;
	delete[] S_k;
	delete[] Td_k;
	delete[] Tt_k;
	for(int k = 0; k < M; k++) {
		delete[] Ld_ki[k];
	}
	delete[] Ld_ki;
	for(int j = 0; j < N; j++) {
		delete[] Lt_jk[j];
	}
	delete[] Lt_jk;
	delete[] Xd_k;
	delete[] Yd_k;
	delete[] Xt_k;
	delete[] Yt_k;
}

//***********************************************************
// Private Member Functions
//***********************************************************


int MMdMtHPP_NLP::U_d_k(int k) {
	// The math is 1-based but k is 0-based
	k++;
	return (L_M + 2)*(k - 1);
}

