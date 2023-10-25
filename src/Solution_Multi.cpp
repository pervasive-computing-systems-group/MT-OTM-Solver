#include "Solution_Multi.h"

/*
 * Solution constructor. Creates a solution based on the graph at gaph_path with m pairs
 * of depots and terminals. If an initial completion time guess, ct_guess, is given then
 * the future base station positions are based off of this guess.
 */
Solution_Multi::Solution_Multi(std::string graph_path, int m, int numUAVs, double ct_guess) {
	double* A = new double[m];
	for(int i = 0; i < m; i++) {
		A[i] = ct_guess/(double)m;
	}

	classConstructor(graph_path, m, numUAVs, A);
	delete[] A;
}


Solution_Multi::Solution_Multi(std::string graph_path, int m, int numUAVs, double* A) {
	classConstructor(graph_path, m, numUAVs, A);
}

Solution_Multi::~Solution_Multi() { }


//***********************************************************
// Public Member Functions
//***********************************************************

/*
 * Returns the time required to run this multi-UAV solution. The result is only valid if the solution is
 * consistent, which isn't guaranteed.
 */
double Solution_Multi::TimeToRunMultiSolution() {
	double ret_val = std::numeric_limits<double>::max();

	if(!m_bFeasible || !m_bSolved) {
		// Problem is not feasible and/or not solved
		ret_val = std::numeric_limits<double>::max();
	}
	else if(m_Tt_k.size() > 0) {
		return m_Tt_k.back();
	}
	else {
		// Find the partition with the farthest out terminal
		double max_x = 0;
		for(int i = 0; i < m_nM; i++) {
			if(GetTerminalOfPartion(i)->fX > max_x) {
				max_x = GetTerminalOfPartion(i)->fX;
			}
		}

		// Return the time required to reach the last base station
		ret_val = max_x/m_tBSTrajectory.mX;
	}

	return ret_val;
}

/*
 * Returns the actual time to run subtour m based on the distance of the subtour and the given velocity flag
 */
double Solution_Multi::GetSubtourTime(int m, E_VelocityFlag fixedVelocityFlag) {
	double ret_val = 0;

	if(!m_bFeasible || !m_bSolved) {
		// Problem is not feasible and/or not solved
		ret_val = std::numeric_limits<double>::max();
	}
	else {
		double dist = DistanceOfPartition(m);

		if(fixedVelocityFlag == E_VelocityFlag::e_FixedOpt) {
			// Fix velocity at the optimal velocity (accept max distance)
			if(dist <= DIST_MAX) {
				// Fly at max speed
				ret_val = dist * (1.0/V_OPT);
			}
			else {
				ret_val = std::numeric_limits<double>::max();
			}
		}
		else if(fixedVelocityFlag == E_VelocityFlag::e_FixedMax) {
			// Fix velocity at max velocity (only accept optimal distances)
			if(dist <= DIST_OPT) {
				// Fly at max speed
				ret_val = dist * (1.0/V_MAX);
			}
			else {
				// This is too long to fly!
				ret_val = std::numeric_limits<double>::max();
			}

		}
		else if(fixedVelocityFlag == E_VelocityFlag::e_FixedPowMin) {
			// Fix velocity at what minimizes energy consumption (prob. a bad idea..)
			if(dist <= DIST_POW_MIN) {
				// Fly at power-opt. speed
				ret_val = dist * (1.0/V_POW_MIN);
			}
			else {
				// This is too long to fly!
				ret_val = std::numeric_limits<double>::max();
			}

		}
		else if(fixedVelocityFlag == E_VelocityFlag::e_NotFixed) {
			// Velocity isn't fixed, modulate velocity
			if(m_bInheritedSpeeds) {
				// Go the inherited speed
				double v = m_vSpeeds.at(m);
				ret_val = dist * (1.0/v);
			}
			else if(dist > DIST_MAX) {
				// This is too long to fly!
				ret_val = std::numeric_limits<double>::max();
			}
			else if(dist <= DIST_OPT) {
				// Fly at max speed
				ret_val = dist * (1.0/V_MAX);
			}
			else {
				// Determine fastest speed to move through this leg
				float v = GetMaxVelocity(dist);
				ret_val = dist * (1.0/v);
			}
		}
		else {
			// Something went wrong here...
			printf("[ERROR] : Solution_Multi::GetSubtourTime() Unexpected velocity flag, %d\n", fixedVelocityFlag);
			exit(1);
		}
	}

	return ret_val;
}


//***********************************************************
// Private Member Functions
//***********************************************************

void Solution_Multi::classConstructor(std::string graph_path, int m, int numUAVs, double* A) {
	m_pAdjMatrix = NULL;
	m_nM = m;
	m_nNumUAVs = numUAVs;
	m_bHasTree = false;
	m_bSolved = false;
	m_bPartitioned = false;
	m_bSetup = false;
	m_bFeasible = true;
	m_bInheritedSpeeds = false;

	// Open file with coordinates
	std::ifstream file(graph_path);
	std::string line;
	// Grab first line with n (number of vertices in search-space graph)
	std::getline(file, line);
	std::stringstream lineStreamN(line);
	// Parse n and R from line
	int n;
	lineStreamN >> n;
	m_nN = n;
	float r;
	lineStreamN >> r;
	m_fR = r;
	m_nNumVertices = (2 * m_nM) + m_nN;
	if(DEBUG_MULT_SOLUTION)
		printf("Graph of n: %d, m: %d, R: %f\n", m_nN, m_nM, m_fR);

	// Create vertices of SSG
	m_pVertexData = new Vertex[m_nNumVertices];

	// Parse coordinates of each destination from file
	float x, y;
	for(int i = 0; i < m_nN; i++) {
		std::getline(file, line);
		std::stringstream lineStream(line);
		lineStream >> x >> y;

		// Configure vertex
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = x;
		m_pVertexData[i].fY = y;
		m_pVertexData[i].eVType = E_VertexType::e_Destination;
	}

	// Read trajectory of base station (start x, y and trajectory vector x_1, y_1)
	std::getline(file, line);
	std::stringstream lineStreamBSxy(line);
	lineStreamBSxy >> x >> y;
	float x_1, y_1;
	std::getline(file, line);
	std::stringstream lineStreamBSTraj(line);
	lineStreamBSTraj >> x_1 >> y_1;

	// Save base station info in base station trajectory struct
	m_tBSTrajectory.configTrajectory(x,y,x_1,y_1,E_TrajFuncType::e_StraightLine);

	// Done reading data, close file
	file.close();

	// Find the additional terminals and depots based on the base stations trajectory.
	place_depotsAndTerminals(A);

	// Sanity print
	if(DEBUG_MULT_SOLUTION) {
		printf("|G| = %d, G:\n", m_nNumVertices);
		for(int j = 0; j < m_nNumVertices; j++) {
			printf(" %d: (%.3f, %.3f), type: %c\n", m_pVertexData[j].nID, m_pVertexData[j].fX, m_pVertexData[j].fY,
					(m_pVertexData[j].eVType == e_Destination) ? 'U' : ((m_pVertexData[j].eVType == e_Terminal) ? 'P' : 'D'));
		}
	}

	if(DEBUG_MULT_SOLUTION)
		printf("Base station: %.3f, %.3f, going: %.3f, %.3f\n\n", m_tBSTrajectory.x, m_tBSTrajectory.y,
				m_tBSTrajectory.mX, m_tBSTrajectory.mY);

	// Allocate memory for adjacency matrix
	m_pAdjMatrix = new bool*[m_nNumVertices];
	for(int i = 0; i < m_nNumVertices; i++) {
		m_pAdjMatrix[i] = new bool[m_nNumVertices];
		for(int j = 0; j < m_nNumVertices; j++) {
			m_pAdjMatrix[i][j] = false;
		}
	}

	// Create single partition with all vertices
	std::vector<Vertex*> partList;
	for(int i = 0; i < m_nNumVertices; i++) {
		// Add to partition
		partList.push_back(m_pVertexData + i);
	}
	// Add this partition to the partitioning map
	m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(0, partList));

	// Solution has been set-up
	m_bSetup = true;
}

void Solution_Multi::place_depotsAndTerminals(double* A) {
	// Add depot and terminal vertices based on base station trajectory using predicted "step-sizes"
	// of the base station for each partition
	// TODO: Fix for non-linear setups
	if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		printf("[ERROR] : Solution_Multi::place_depotsAndTerminals : function does not work for non-linear inputs!\n");

		exit(0);
	}

	float battery_change_step_x = m_tBSTrajectory.mX * BATTERY_SWAP_TIME;
	float battery_change_step_y = m_tBSTrajectory.mY * BATTERY_SWAP_TIME;

	// Track the position of the base station while we step through sub-tours
	float* current_loc_x = new float[m_nNumUAVs];
	float* current_loc_y = new float[m_nNumUAVs];

	// Each UAV starts at the same starting point
	for(int i = 0; i < m_nNumUAVs; i++) {
		current_loc_x[i] = m_tBSTrajectory.x;
		current_loc_y[i] = m_tBSTrajectory.y;
	}

	// By convention, we first add the terminals and then add the depots
	for(int i = m_nN, j = 0; i < (m_nM + m_nN); i++, j++) {
		// Rotate through the UAVs
		int nUAV_ID = j%m_nNumUAVs;

		// Add depot
		int depot_id = i + m_nM;
		m_pVertexData[depot_id].nID = depot_id;
		m_pVertexData[depot_id].fX = current_loc_x[nUAV_ID];
		m_pVertexData[depot_id].fY = current_loc_y[nUAV_ID];
		m_pVertexData[depot_id].eVType = E_VertexType::e_Depot;
		if(DEBUG_MULT_SOLUTION)
			printf(" depot at x:%f, y:%f\n", m_pVertexData[depot_id].fX, m_pVertexData[depot_id].fY);

		if(DEBUG_MULT_SOLUTION)
			printf(" time for subtour:%f\n", A[j]);

		// Move forward
		float tour_duration_step_x = (A[j] * m_tBSTrajectory.mX);
		float tour_duration_step_y = (A[j] * m_tBSTrajectory.mY);
		current_loc_x[nUAV_ID] += tour_duration_step_x;
		current_loc_y[nUAV_ID] += tour_duration_step_y;
		if(DEBUG_MULT_SOLUTION)
			printf(" move step by x:%f, y:%f\n", tour_duration_step_x, tour_duration_step_y);

		// Add terminal
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = current_loc_x[nUAV_ID];
		m_pVertexData[i].fY = current_loc_y[nUAV_ID];
		m_pVertexData[i].eVType = E_VertexType::e_Terminal;
		if(DEBUG_MULT_SOLUTION)
			printf(" terminal at x:%f, y:%f\n", m_pVertexData[i].fX, m_pVertexData[i].fY);

		// Update for the next depot
		current_loc_x[nUAV_ID] += battery_change_step_x;
		current_loc_y[nUAV_ID] += battery_change_step_y;
	}

	delete[] current_loc_x;
	delete[] current_loc_y;
}

