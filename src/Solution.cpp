#include "Solution.h"

Solution::Solution() {
}

/*
 * Solution constructor. Creates a solution based on the graph at gaph_path with m pairs
 * of depots and terminals. If an initial completion time guess, ct_guess, is given then
 * the future base station positions are based off of this guess.
 */
Solution::Solution(std::string graph_path) {
	m_pAdjMatrix = NULL;
	m_nM = 1;
	m_bHasTree = false;
	m_bSolved = false;
	m_bPartitioned = false;
	m_bSetup = false;
	m_bFeasible = false;
	m_bInheritedSpeeds = true;

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
	if(DEBUG_SOLUTION)
		printf("Building graph of n: %d, m: %d, R: %f\n", m_nN, m_nM, m_fR);

	// Create vertices of SSG
	m_pVertexData = new Vertex[m_nNumVertices];

	// Parse coordinates of each destination from file
	float x, y;
	float fISX = 0;
	float fISY = 0;
	int i = 0;
	for(; i < m_nN; i++) {
		std::getline(file, line);
		std::stringstream lineStream(line);
		lineStream >> x >> y;

		// Configure vertex
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = x;
		m_pVertexData[i].fY = y;
		m_pVertexData[i].eVType = E_VertexType::e_Destination;
		if(fISX < m_pVertexData[i].fX) {
			fISX = m_pVertexData[i].fX;
		}
	}

	// Create an "ideal-stop" vertex
	m_pVertexData[i].nID = i;
	m_pVertexData[i].fX = fISX;
	m_pVertexData[i].fY = fISY;
	m_pVertexData[i].eVType = E_VertexType::e_Terminal;

	if(DEBUG_SOLUTION) {
		printf(" Added \"ideal-stop\" %d, at (%f, %f)\n", m_pVertexData[i].nID, m_pVertexData[i].fX, m_pVertexData[i].fY);
	}

	// Increment i
	i++;

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

	// Create initial base station
	if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		// The following assumes a linear UGV... hard fail
		printf("[ERROR] : Solution::Solution : Given non-linear UGV, expected linear\n");
		exit(0);
	}
	m_pVertexData[i].nID = i;
	m_pVertexData[i].fX = m_tBSTrajectory.x;
	m_pVertexData[i].fY = m_tBSTrajectory.y;
	m_pVertexData[i].eVType = E_VertexType::e_Depot;

	if(DEBUG_SOLUTION) {
		printf(" Added base station %d, at (%f, %f)\n", m_pVertexData[i].nID, m_pVertexData[i].fX, m_pVertexData[i].fY);
	}

	// Done reading data, close file
	file.close();

	// Sanity print
	if(DEBUG_SOLUTION) {
		printf("|G| = %d, G:\n", m_nNumVertices);
		for(int j = 0; j < m_nNumVertices; j++) {
			printf(" %d: (%.3f, %.3f), type: %c\n", m_pVertexData[j].nID, m_pVertexData[j].fX, m_pVertexData[j].fY,
					(m_pVertexData[j].eVType == e_Destination) ? 'U' : ((m_pVertexData[j].eVType == e_Terminal) ? 'P' : 'D'));
		}
	}

	if(DEBUG_SOLUTION)
		printf("Base station: %.3f, %.3f, going: %.3f, %.3f\n\n",m_tBSTrajectory.x, m_tBSTrajectory.y,
				m_tBSTrajectory.mX, m_tBSTrajectory.mY);

	// Allocate memory for adjacency matrix
	m_pAdjMatrix = new bool*[m_nNumVertices];
	for(int i = 0; i < m_nNumVertices; i++) {
		m_pAdjMatrix[i] = new bool[m_nNumVertices];
		for(int j = 0; j < m_nNumVertices; j++) {
			m_pAdjMatrix[i][j] = false;
		}
	}
}

/*
 * Solution constructor. Creates a solution based on the graph at gaph_path with m pairs
 * of depots and terminals. If an initial completion time guess, ct_guess, is given then
 * the future base station positions are based off of this guess.
 */
Solution::Solution(std::string graph_path, int m, float ct_guess) {
	m_pAdjMatrix = NULL;
	m_nM = m;
	m_bHasTree = false;
	m_bSolved = false;
	m_bPartitioned = false;
	m_bSetup = false;

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
	if(DEBUG_SOLUTION)
		printf("Graph of n: %d, m: %d, R: %f\n", m_nN, m_nM, m_fR);

	// Create vertices of SSG
	m_pVertexData = new Vertex[m_nNumVertices];

	// Parse coordinates of each destination from file
	float x, y;
	int i = 0;
	for(; i < m_nN; i++) {
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

	// Add depot and terminal vertices based on base station trajectory using predicted "step-sizes"
	// of the base station for each partition
	if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		// The following assumes a linear UGV... hard fail
		printf("[ERROR] : Solution::Solution : Given non-linear UGV, expected linear\n");
		exit(0);
	}
	float tour_duration_step_x = 0;
	float tour_duration_step_y = 0;
	float battery_change_ts = BATTERY_SWAP_TIME;
	float battery_change_step_x = m_tBSTrajectory.mX * battery_change_ts;
	float battery_change_step_y = m_tBSTrajectory.mY * battery_change_ts;

	if(ct_guess >= 0) {
		// Determine approximate base station step size per partition based off of given completion time guess
		float time_less_battery_swap = ct_guess - (battery_change_ts * (m_nM - 1));
		float time_per_step = time_less_battery_swap / (float)m_nM;

		tour_duration_step_x = time_per_step * m_tBSTrajectory.mX;
		tour_duration_step_y = time_per_step * m_tBSTrajectory.mY;
	}
	else {
		/*
		 * Determine approximate base station step size per partition based off of base station trajectory
		 *
		 * For now, we are looking at the initial position of the base station and adding the some
		 * guess of the time to cover 1/m of the SSG. We multiply this duration by the base station
		 * trajectory vector to get the location of the first terminal (future base station location).
		 * For every following future base station locations add to the previous estimate the product
		 * of v_bs and some given battery change-time constant. For the corresponding terminals we
		 * again add the 1/m * v_bs to the new starting base station location.
		 */
//		tour_duration_step_x = (float)m_nN/m_nM * m_tBSTrajectory.mX;
//		tour_duration_step_y = (float)m_nN/m_nM * m_tBSTrajectory.mY;

		// Use the min-spanning-forest as an initial guess
		float base_line_time = this->GetMinSpanningForestRT();

		// Determine approximate base station step size per partition based off of give completion time guess
		float time_less_battery_swap = base_line_time - (battery_change_ts * (m_nM - 1));
		float time_per_step = time_less_battery_swap / (float)m_nM;

		tour_duration_step_x = time_per_step * m_tBSTrajectory.mX;
		tour_duration_step_y = time_per_step * m_tBSTrajectory.mY;
	}

	// Find the additional terminals and depots based on the base stations trajectory.
	// First terminal location
	float terminal_location_x = m_tBSTrajectory.x + tour_duration_step_x;
	float terminal_location_y = m_tBSTrajectory.y + tour_duration_step_y;

	// Create total graph G = U ∪ D ∪ P (destinations + terminals and depots)
	// By convention, we first add the terminals and then add the depots
	for(; i < (m_nM + m_nN); i++) {
		// Configure vertex
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = terminal_location_x;
		m_pVertexData[i].fY = terminal_location_y;
		m_pVertexData[i].eVType = E_VertexType::e_Terminal;

		// Update for the next terminal
		terminal_location_x += battery_change_step_x + tour_duration_step_x;
		terminal_location_y += battery_change_step_y + tour_duration_step_y;
	}

	// The first depot is the base stations starting point
	m_pVertexData[i].nID = i;
	m_pVertexData[i].fX = m_tBSTrajectory.x;
	m_pVertexData[i].fY = m_tBSTrajectory.y;
	m_pVertexData[i].eVType = E_VertexType::e_Depot;
	i++;

	// Add additional depot locations
	float depot_location_x = m_tBSTrajectory.x + tour_duration_step_x + battery_change_step_x;
	float depot_location_y = m_tBSTrajectory.y + tour_duration_step_y + battery_change_step_y;
	for(; i < m_nNumVertices; i++) {
		// Configure vertex
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = depot_location_x;
		m_pVertexData[i].fY = depot_location_y;
		m_pVertexData[i].eVType = E_VertexType::e_Depot;

		// Update for the next terminal
		depot_location_x += battery_change_step_x + tour_duration_step_x;
		depot_location_y += battery_change_step_y + tour_duration_step_y;
	}

	// Sanity print
	if(DEBUG_SOLUTION) {
		printf("|G| = %d, G:\n", m_nNumVertices);
		for(int j = 0; j < m_nNumVertices; j++) {
			printf(" %d: (%.3f, %.3f), type: %c\n", m_pVertexData[j].nID, m_pVertexData[j].fX, m_pVertexData[j].fY,
					(m_pVertexData[j].eVType == e_Destination) ? 'U' : ((m_pVertexData[j].eVType == e_Terminal) ? 'P' : 'D'));
		}
	}

	if(DEBUG_SOLUTION)
		printf("Base station: %.3f, %.3f, going: %.3f, %.3f\n\n",m_tBSTrajectory.x, m_tBSTrajectory.y,
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


Solution::Solution(std::string graph_path, int m, float ct_guess, float* A) {
	m_pAdjMatrix = NULL;
	m_nM = m;
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
	if(DEBUG_SOLUTION)
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

	// Create initial base station
	if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		// The following assumes a linear UGV... hard fail
		printf("[ERROR] : Solution::Solution : Given non-linear UGV, expected linear\n");
		exit(0);
	}

	// Add depot and terminal vertices based on base station trajectory using predicted "step-sizes"
	// of the base station for each partition

	float battery_change_ts = BATTERY_SWAP_TIME;
	float battery_change_step_x = m_tBSTrajectory.mX * battery_change_ts;
	float battery_change_step_y = m_tBSTrajectory.mY * battery_change_ts;

	// Determine approximate base station step size per partition based off of given completion time guess
	float time_less_battery_swap = ct_guess - (battery_change_ts * (m_nM - 1));

	// Find the additional terminals and depots based on the base stations trajectory.
	// First terminal location

	// Create total graph G = U ∪ D ∪ P (destinations + terminals and depots)
	float current_location_x = m_tBSTrajectory.x;
	float current_location_y = m_tBSTrajectory.y;

	// By convention, we first add the terminals and then add the depots
	for(int i = m_nN, j = 0; i < (m_nM + m_nN); i++, j++) {
		// Add depot
		int depot_id = i + m_nM;
		m_pVertexData[depot_id].nID = depot_id;
		m_pVertexData[depot_id].fX = current_location_x;
		m_pVertexData[depot_id].fY = current_location_y;
		m_pVertexData[depot_id].eVType = E_VertexType::e_Depot;
		if(DEBUG_SOLUTION)
			printf(" depot at x:%f, y:%f\n", m_pVertexData[depot_id].fX, m_pVertexData[depot_id].fY);

		// Move forward
		float time_per_step = time_less_battery_swap * A[j];
		float tour_duration_step_x = (time_per_step * m_tBSTrajectory.mX);
		float tour_duration_step_y = (time_per_step * m_tBSTrajectory.mY);
		current_location_x += tour_duration_step_x;
		current_location_y += tour_duration_step_y;
		if(DEBUG_SOLUTION)
			printf(" move step by x:%f, y:%f\n", tour_duration_step_x, tour_duration_step_y);

		// Add terminal
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = current_location_x;
		m_pVertexData[i].fY = current_location_y;
		m_pVertexData[i].eVType = E_VertexType::e_Terminal;
		if(DEBUG_SOLUTION)
			printf(" terminal at x:%f, y:%f\n", m_pVertexData[i].fX, m_pVertexData[i].fY);

		// Update for the next depot
		current_location_x += battery_change_step_x;
		current_location_y += battery_change_step_y;
	}

	// Sanity print
	if(DEBUG_SOLUTION) {
		printf("|G| = %d, G:\n", m_nNumVertices);
		for(int j = 0; j < m_nNumVertices; j++) {
			printf(" %d: (%.3f, %.3f), type: %c\n", m_pVertexData[j].nID, m_pVertexData[j].fX, m_pVertexData[j].fY,
					(m_pVertexData[j].eVType == e_Destination) ? 'U' : ((m_pVertexData[j].eVType == e_Terminal) ? 'P' : 'D'));
		}
	}

	if(DEBUG_SOLUTION)
		printf("Base station: %.3f, %.3f, going: %.3f, %.3f\n\n",m_tBSTrajectory.x, m_tBSTrajectory.y,
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

Solution::Solution(Solution* cp_solution, float ct, float* A) {
	m_nN = cp_solution->m_nN;
	m_nM = cp_solution->m_nM;
	m_fR = cp_solution->m_fR;
	m_bHasTree = cp_solution->m_bHasTree;
	m_bSolved = cp_solution->m_bSolved;
	m_bPartitioned = cp_solution->m_bPartitioned;
	m_nNumVertices = cp_solution->m_nNumVertices;
	m_bFeasible = true;
	m_bInheritedSpeeds = false;

	// Allocate memory and copy over adjacency matrix
	m_pAdjMatrix = new bool*[m_nNumVertices];
	for(int i = 0; i < m_nNumVertices; i++) {
		m_pAdjMatrix[i] = new bool[m_nNumVertices];
		for(int j = 0; j < m_nNumVertices; j++) {
			m_pAdjMatrix[i][j] = cp_solution->m_pAdjMatrix[i][j];
		}
	}

	// Create vertices of SSG
	m_pVertexData = new Vertex[m_nNumVertices];
	// Copy over way-point data
	for(int i = 0; i < m_nN; i++) {
		// Configure vertex
		m_pVertexData[i].nID = cp_solution->m_pVertexData[i].nID;
		m_pVertexData[i].fX = cp_solution->m_pVertexData[i].fX;
		m_pVertexData[i].fY = cp_solution->m_pVertexData[i].fY;
		m_pVertexData[i].eVType = E_VertexType::e_Destination;
	}

	// Add base station info in base station trajectory struct
	m_tBSTrajectory.x = cp_solution->m_tBSTrajectory.x;
	m_tBSTrajectory.y = cp_solution->m_tBSTrajectory.y;
	m_tBSTrajectory.mX = cp_solution->m_tBSTrajectory.mX;
	m_tBSTrajectory.mY = cp_solution->m_tBSTrajectory.mY;

	// Save base station info in base station trajectory struct
	m_tBSTrajectory.configTrajectory(cp_solution->m_tBSTrajectory.x,
			cp_solution->m_tBSTrajectory.y,cp_solution->m_tBSTrajectory.mX,
			cp_solution->m_tBSTrajectory.mY,E_TrajFuncType::e_StraightLine);

	// Create initial base station
	if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
		// The following assumes a linear UGV... hard fail
		printf("[ERROR] : Solution::Solution : Given non-linear UGV, expected linear\n");
		exit(0);
	}

	if(DEBUG_SOLUTION)
		printf("Graph of n: %d, m: %d, R: %f\n", m_nN, m_nM, m_fR);

	// Add depot and terminal vertices based on base station trajectory using predicted "step-sizes"
	// of the base station for each partition
	float battery_change_ts = BATTERY_SWAP_TIME;
	float battery_change_step_x = m_tBSTrajectory.mX * battery_change_ts;
	float battery_change_step_y = m_tBSTrajectory.mY * battery_change_ts;

	// Determine approximate base station step size per partition based off of given completion time guess
	float time_less_battery_swap = ct - (battery_change_ts * (m_nM - 1));

	// Find the additional terminals and depots based on the base stations trajectory.
	// First terminal location

	// Create total graph G = U ∪ D ∪ P (destinations + terminals and depots)
	float current_location_x = m_tBSTrajectory.x;
	float current_location_y = m_tBSTrajectory.y;

	// By convention, we first add the terminals and then add the depots
	for(int i = m_nN, j = 0; i < (m_nM + m_nN); i++, j++) {
		// Add depot
		int depot_id = i + m_nM;
		m_pVertexData[depot_id].nID = depot_id;
		m_pVertexData[depot_id].fX = current_location_x;
		m_pVertexData[depot_id].fY = current_location_y;
		m_pVertexData[depot_id].eVType = E_VertexType::e_Depot;
		if(DEBUG_SOLUTION)
			printf(" depot at x:%f, y:%f\n", m_pVertexData[depot_id].fX, m_pVertexData[depot_id].fY);

		// Move forward
		float time_per_step = time_less_battery_swap * A[j];
		float tour_duration_step_x = (time_per_step * m_tBSTrajectory.mX);
		float tour_duration_step_y = (time_per_step * m_tBSTrajectory.mY);
		current_location_x += tour_duration_step_x;
		current_location_y += tour_duration_step_y;
		if(DEBUG_SOLUTION)
			printf(" move step by x:%f, y:%f\n", tour_duration_step_x, tour_duration_step_y);

		// Add terminal
		m_pVertexData[i].nID = i;
		m_pVertexData[i].fX = current_location_x;
		m_pVertexData[i].fY = current_location_y;
		m_pVertexData[i].eVType = E_VertexType::e_Terminal;
		if(DEBUG_SOLUTION)
			printf(" terminal at x:%f, y:%f\n", m_pVertexData[i].fX, m_pVertexData[i].fY);

		// Update for the next depot
		current_location_x += battery_change_step_x;
		current_location_y += battery_change_step_y;
	}

	// Sanity print
	if(DEBUG_SOLUTION) {
		printf("|G| = %d, G:\n", m_nNumVertices);
		for(int j = 0; j < m_nNumVertices; j++) {
			printf(" %d: (%.3f, %.3f), type: %c\n", m_pVertexData[j].nID, m_pVertexData[j].fX, m_pVertexData[j].fY,
					(m_pVertexData[j].eVType == e_Destination) ? 'U' : ((m_pVertexData[j].eVType == e_Terminal) ? 'P' : 'D'));
		}
	}

	if(DEBUG_SOLUTION)
		printf("Base station: %.3f, %.3f, going: %.3f, %.3f\n\n",m_tBSTrajectory.x, m_tBSTrajectory.y,
				m_tBSTrajectory.mX, m_tBSTrajectory.mY);

	if(m_bPartitioned) {
		// Copy over any partitioning data
		for(auto p : cp_solution->m_mPartitions) {
			std::vector<Vertex*> partList;
			for(Vertex* v : p.second) {
				partList.push_back(v);
			}
			// Add this partition to the partitioning map
			m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(p.first, partList));
		}
	}
	else {
		// Just create single partition with all vertices
		std::vector<Vertex*> partList;
		for(int i = 0; i < m_nNumVertices; i++) {
			// Add to partition
			partList.push_back(m_pVertexData + i);
		}
		// Add this partition to the partitioning map
		m_mPartitions.insert(std::pair<int, std::vector<Vertex*>>(0, partList));
	}

	// Solution has been set-up
	m_bSetup = true;

}

Solution::~Solution() {
	// Clear vertex list
	delete[] m_pVertexData;
	if(m_pAdjMatrix != NULL) {
		// Clear adjacency matrix
		for(int i = 0; i < m_nNumVertices; i++) {
			delete[] m_pAdjMatrix[i];
		}
		delete[] m_pAdjMatrix;
	}
}


//***********************************************************
// Public Member Functions
//***********************************************************

// Build complete solution
bool Solution::BuildCompleteSolution(std::vector<std::list<Vertex*>>& tours, std::vector<float>& speeds) {
	// Verify that we should be completing this solution
	if(this->m_bSetup) {
		// Already complete solution... return false!
		return false;
	}

	// Clear the old adjacency matrix
	if(m_pAdjMatrix != NULL) {
		// Clear adjacency matrix
		for(int i = 0; i < m_nNumVertices; i++) {
			delete[] m_pAdjMatrix[i];
		}
		delete[] m_pAdjMatrix;
	}

	/// Use tours to create new vertices
	// Determine new value for M
	m_nM = (int)tours.size();
	m_nNumVertices =  (2 * m_nM) + m_nN;
	m_bFeasible = true;

	// Allocate memory for new adjacency matrix
	m_pAdjMatrix = new bool*[m_nNumVertices];
	for(int i = 0; i < m_nNumVertices; i++) {
		m_pAdjMatrix[i] = new bool[m_nNumVertices];
		for(int j = 0; j < m_nNumVertices; j++) {
			m_pAdjMatrix[i][j] = false;
		}
	}

	// Create vertices
	Vertex* pVertexData = new Vertex[m_nNumVertices];

	// Copy over way-point data and solution into adjacency matrix
	std::vector<std::list<Vertex *>>::iterator lst = tours.begin();
	for(int i = 0; lst != tours.end(); lst++, i++) {
		int prev_wp = -1;
		for(Vertex* v : *lst) {
			// Determine vertex ID
			int id = (v->eVType == E_VertexType::e_Depot) ? (m_nN + m_nM + i) :
					((v->eVType == E_VertexType::e_Terminal) ? (m_nN + i) : v->nID);

			if(prev_wp != -1) {
				// Add last_wp -> id to adjacency matrix
				int a = (prev_wp < id) ? prev_wp : id;
				int b = (prev_wp >= id) ? prev_wp : id;
				m_pAdjMatrix[a][b] = true;
			}

			// Copy over data for this vertex
			pVertexData[id].nID = id;
			pVertexData[id].fX = v->fX;
			pVertexData[id].fY = v->fY;
			pVertexData[id].eVType = v->eVType;

			// Update the previous waypoint
			prev_wp = id;
		}

		// Record sub-tour's designated speed
		m_vSpeeds.push_back(speeds[i]);
	}

	// Clean-up vertex memory
	delete[] m_pVertexData;
	// Assign new vertex array
	m_pVertexData = pVertexData;

	// Sanity print
	if(DEBUG_SOLUTION) {
		printf("Reformulated solution: n = %d, m = %d, |G| = %d\nG:\n", m_nN, m_nM, m_nNumVertices);
		for(int j = 0; j < m_nNumVertices; j++) {
			printf(" %d: (%.3f, %.3f), type: %c\n", m_pVertexData[j].nID, m_pVertexData[j].fX, m_pVertexData[j].fY,
					(m_pVertexData[j].eVType == e_Destination) ? 'U' : ((m_pVertexData[j].eVType == e_Terminal) ? 'P' : 'D'));
		}
		printf("\nAdjacency Matrix:\n");
		for(int i = 0; i < m_nNumVertices; i++) {
			for(int j = 0; j < m_nNumVertices; j++) {
				printf(" %d", m_pAdjMatrix[i][j]);
			}
			printf("\n");
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

	return true;
}

/*
 * Prints out the adjacency matrix contained in this solution (or nothing if adjacency matrix was not set-up)
 */
void Solution::PrintSolution() {
	// Verify that the adjacency matrix was created
	if((m_pAdjMatrix == NULL) || !m_bSolved || !m_bSetup) {
		printf("\nSolution does not contain an adjMatrix!\n");
	}
	else {
		// Create file with graph G
		FILE * pGraphFile;
		pGraphFile = fopen("graph_output.txt", "w");
		fprintf(pGraphFile, "%d %d\n", m_nM, m_nN);
		// Print the coordinates of each vertex
		for(int j = 0; j < m_nNumVertices; j++) {
			fprintf(pGraphFile, "%f %f\n", m_pVertexData[j].fX, m_pVertexData[j].fY);
		}
		fclose(pGraphFile);

		// Save results to file
		FILE * pPathFile;
		pPathFile = fopen("path_output.txt", "w");

		// Print adjacency matrix entries
		if(PRINT_SOLUTION) {
			printf("\nadjMatrix : \n  ");
			for(int i = 0; i < m_nNumVertices; i++) {
				printf("%d\t", i);
			}
			printf("\n");
		}

		for(int i = 0; i < m_nNumVertices; i++) {
			if(PRINT_SOLUTION)
				printf("%d|", i);
			for(int j = 0; j < m_nNumVertices; j++) {
				if(m_pAdjMatrix[i][j]) {
					if(PRINT_SOLUTION)
						printf("1\t");
					fprintf(pPathFile, "%d %d\n", i, j);
				}
				else {
					if(PRINT_SOLUTION)
						printf("-\t");
				}
			}
			if(PRINT_SOLUTION)
				printf("\n");
		}

		// Close path file
		fclose(pPathFile);
	}
}

// Prints out the data to be used in an AMPL model. This will look like a large 2D array of absolute distances
void Solution::PrintAMPLData() {
	// Print results to file
	FILE * pDataFile;
	char buff[100];
    sprintf(buff, "data.txt");
    printf("%s\n", buff);
    pDataFile = fopen(buff, "w");

    printf(" Adding NxN matrix of destination data\n");
	fprintf(pDataFile, "param d1 :\n");
	for(int i = 0; i < m_nN; i++) {
		fprintf(pDataFile, "\t%d\t", i + 1);
	}
	fprintf(pDataFile, ":=\n");
	for(int i = 0; i < m_nN; i++) {
		fprintf(pDataFile, "%d", i + 1);
		for(int j = 0; j < m_nN; j++) {
			fprintf(pDataFile, "\t%.5f", (m_pVertexData + i)->GetDistanceTo(m_pVertexData + j) );
		}
		fprintf(pDataFile, "\n");
	}

    printf(" Adding MxN matrix for depot data\n");
	fprintf(pDataFile, "\nparam d2 :\n");
	for(int i = 0; i < m_nN; i++) {
		fprintf(pDataFile, "\t%d\t", i + 1);
	}
	fprintf(pDataFile, ":=\n");
	for(int i = 0; i < m_nM; i++) {
		fprintf(pDataFile, "%d", i + 1);
		for(int j = 0; j < m_nN; j++) {
			fprintf(pDataFile, "\t%.5f", GetDepotOfPartion(i)->GetDistanceTo(m_pVertexData + j) );
		}
		fprintf(pDataFile, "\n");
	}

    printf(" Adding NxM matrix for terminal data\n");
	fprintf(pDataFile, "\nparam d3 :\n");
	for(int i = 0; i < m_nM; i++) {
		fprintf(pDataFile, "\t%d\t", i + 1);
	}
	fprintf(pDataFile, ":=\n");
	for(int i = 0; i < m_nN; i++) {
		fprintf(pDataFile, "%d", i + 1);
		for(int j = 0; j < m_nM; j++) {
			fprintf(pDataFile, "\t%.5f", this->GetTerminalOfPartion(j)->GetDistanceTo(m_pVertexData + i) );
		}
		fprintf(pDataFile, "\n");
	}

	fclose(pDataFile);
}

// Prints out the data to be used in the TSPLIB file format for the LKH solver
void Solution::PrintLKHData() {
    printf("Creating LKH Data Files\n");

	// Print solver parameter file
    printf(" Creating parameter file\n");
	FILE * pParFile;
	char buff1[100];
    sprintf(buff1, "FixedHPP.par");
    printf("  %s\n", buff1);
    pParFile = fopen(buff1, "w");

	fprintf(pParFile, "PROBLEM_FILE = FixedHPP.tsp\n");
	fprintf(pParFile, "COMMENT Fixed Hamiltonian Path Problem\n");
	fprintf(pParFile, "TOUR_FILE = LKH_output.dat\n");

    printf("  Done!\n");
	fclose(pParFile);

	// Print node data to file
    printf(" Creating node data file\n");
	FILE * pDataFile;
	char buff2[100];
    sprintf(buff2, "FixedHPP.tsp");
    printf("  %s\n", buff2);
    pDataFile = fopen(buff2, "w");

    printf("  Adding graph specification data\n");
	fprintf(pDataFile, "NAME : FixedHPP \n");
	fprintf(pDataFile, "COMMENT : Fixed Hamiltonian Path Problem \n");
	fprintf(pDataFile, "TYPE : TSP \n");
	fprintf(pDataFile, "DIMENSION : %d \n", this->m_nN);
	fprintf(pDataFile, "EDGE_WEIGHT_TYPE : EXPLICIT \n");
	fprintf(pDataFile, "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n");

    printf("  Adding NxN weights matrix\n");
	fprintf(pDataFile, "EDGE_WEIGHT_SECTION\n");
	for(int i = 0; i < m_nN; i++) {
		for(int j = 0; j < m_nN; j++) {
			fprintf(pDataFile, "%.5f\t", (m_pVertexData + i)->GetDistanceTo(m_pVertexData + j) );
		}
		fprintf(pDataFile, "\n");
	}

    printf("  Done!\n");
	fprintf(pDataFile, "EOF\n");
	fclose(pDataFile);
}


// Prints out the data to be used in the TSPLIB file format for the LKH solver where the weight between a and b is 0
void Solution::PrintLKHDataFHPP(int a, int b) {
	if(DEBUG_SOLUTION)
		printf("Creating LKH Data Files\n a = %d, b = %d\n", a, b);

	// Print solver parameter file
	if(DEBUG_SOLUTION)
		printf(" Creating parameter file\n");
	FILE * pParFile;
	char buff1[100];
    sprintf(buff1, "FixedHPP.par");
	if(DEBUG_SOLUTION)
		printf("  %s\n", buff1);
    pParFile = fopen(buff1, "w");

	fprintf(pParFile, "PROBLEM_FILE = FixedHPP.tsp\n");
	fprintf(pParFile, "COMMENT Fixed Hamiltonian Path Problem\n");
	fprintf(pParFile, "TOUR_FILE = LKH_output.dat\n");

	if(DEBUG_SOLUTION)
		printf("  Done!\n");
	fclose(pParFile);

	// Print node data to file
	if(DEBUG_SOLUTION)
		printf(" Creating node data file\n");
	FILE * pDataFile;
	char buff2[100];
    sprintf(buff2, "FixedHPP.tsp");
	if(DEBUG_SOLUTION)
		printf("  %s\n", buff2);
    pDataFile = fopen(buff2, "w");

	if(DEBUG_SOLUTION)
		printf("  Adding graph specification data\n");
	fprintf(pDataFile, "NAME : FixedHPP \n");
	fprintf(pDataFile, "COMMENT : Fixed Hamiltonian Path Problem \n");
	fprintf(pDataFile, "TYPE : TSP \n");
	fprintf(pDataFile, "DIMENSION : %d \n", this->m_nN);
	fprintf(pDataFile, "EDGE_WEIGHT_TYPE : EXPLICIT \n");
	fprintf(pDataFile, "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n");

	if(DEBUG_SOLUTION)
		printf("  Adding NxN weights matrix\n");
	fprintf(pDataFile, "EDGE_WEIGHT_SECTION\n");
	for(int i = 0; i < m_nN; i++) {
		for(int j = 0; j < m_nN; j++) {
			if((i == a && j == b) || (i == b && j == a)) {
				if(DEBUG_SOLUTION)
					printf("   Found a and b\n");
				fprintf(pDataFile, "%f\t", 0.0);
			}
			else {
				if((i == a) || (i == b) || (j == a) || (i == b)) {
					fprintf(pDataFile, "%f\t", (m_pVertexData + i)->GetDistanceTo(m_pVertexData + j)*1000 );
				}
				else {
					fprintf(pDataFile, "%f\t", (m_pVertexData + i)->GetDistanceTo(m_pVertexData + j) );
				}
			}
		}
		fprintf(pDataFile, "\n");
	}

	if(DEBUG_SOLUTION)
		printf("  Done!\n");
	fprintf(pDataFile, "EOF\n");
	fclose(pDataFile);
}

// Clears the adjacency matrix (resets all to false) (if adjacency matrix was created)
void Solution::ClearAdjacencyMatrix() {
	// Verify that the adjacency matrix was created
	if(m_pAdjMatrix == NULL) {
		if(SANITY_PRINT)
			printf("\nSolution does not contain an adjMatrix!\n");
	}
	else {
		// Reset all entries to false
		for(int i = 0; i < m_nNumVertices; i++) {
			for(int j = 0; j < m_nNumVertices; j++) {
				m_pAdjMatrix[i][j] = false;
			}
		}
	}

	m_bHasTree = false;
}

// Determine the amount of time required for a UAV to run this solution
float Solution::TimeToRunSolution(E_VelocityFlag fixedVelocityFlag, bool bLagrangianRelaxation) {
	if(!m_bSolved || !m_bFeasible) {
		// We didn't solve this problem yet!!
		return std::numeric_limits<float>::max();
	}

	if(SANITY_PRINT)
		printf("\nCalculating Distance %d partitions:\n", m_nM);

	float t_tot = 0;

	// Add up the time to complete each partitioned path
	for(int i = 0; i < m_nM; i++) {
		float legDist = DistanceOfPartition(i);
		if(SANITY_PRINT)
			printf(" legDist = %f\n", legDist);

		// Check to see if we are fixing the velocity
		if(fixedVelocityFlag == E_VelocityFlag::e_FixedOpt) {
			// Fix velocity at the optimal velocity (accept max distance)
			if(legDist <= DIST_MAX) {
				// Fly at max speed
				float t = legDist * (1.0/V_OPT);
				t_tot += t;
				if(1)
					printf(" go v = %f, t = %f\n", V_OPT, t);
			}
			else {
				// This is too long to fly!
				if(1)
					printf(" TOO FAR! dist = %f\n", legDist);
				return std::numeric_limits<float>::max();
			}
		}
		else if(fixedVelocityFlag == E_VelocityFlag::e_FixedMax) {
			// Fix velocity at max velocity (only accept optimal distances)
			if(legDist <= DIST_OPT) {
				// Fly at max speed
				float t = legDist * (1.0/V_MAX);
				t_tot += t;
				if(1)
					printf(" go v = %f, t = %f\n", V_OPT, t);
			}
			else {
				// This is too long to fly!
				if(1)
					printf(" TOO FAR! dist = %f\n", legDist);
				return std::numeric_limits<float>::max();
			}
		}
		else if(fixedVelocityFlag == E_VelocityFlag::e_FixedPowMin) {
			// Fix velocity at max velocity (only accept optimal distances)
			if(legDist <= DIST_POW_MIN) {
				// Fly at max speed
				float t = legDist * (1.0/V_POW_MIN);
				t_tot += t;
				if(1)
					printf(" go v = %f, t = %f\n", V_POW_MIN, t);
			}
			else {
				// This is too long to fly!
				if(1)
					printf(" TOO FAR! dist = %f\n", legDist);
				return std::numeric_limits<float>::max();
			}
		}
		// Velocity isn't fixed, modulate velocity
		else if(fixedVelocityFlag == E_VelocityFlag::e_NotFixed) {
			if(m_bInheritedSpeeds) {
				// Go the inherited speed
				float v = this->m_vSpeeds.at(i);
				float t = legDist * (1.0/v);
				t_tot += t;
				if(SANITY_PRINT)
					printf(" go v = %f, t = %f\n", v, t);
			}
			else if(legDist > DIST_MAX) {
				if(bLagrangianRelaxation) {
					// Relax the max-distance constraint (used for training)
					float t = legDist * (1.0/V_OPT);
					// Add penalty for over-shooting
					t += (legDist - DIST_MAX) * (1.0/V_MAX);
					t_tot += t;

					if(SANITY_PRINT)
						printf(" TOO FAR! t = %f\n", t);
				}
				else {
					// This is too long to fly!
					return std::numeric_limits<float>::max();
				}
			}
			else if(legDist <= DIST_OPT) {
				// Fly at max speed
				float t = legDist * (1.0/V_MAX);
				t_tot += t;
				if(SANITY_PRINT)
					printf(" go V_MAX = %f, t = %f\n", V_MAX, t);
			}
			else {
				// Determine fastest speed to move through this leg
				float v = GetMaxVelocity(legDist);

				float t = legDist * (1.0/v);
				t_tot += t;

				if(SANITY_PRINT)
					printf(" go v = %f, t = %f\n", v, t);
			}
		}
		else {
			// Something went wrong here...
			printf("[ERROR] : Solution::TimeToRunSolution() unexpected velocity flag, %d\n", fixedVelocityFlag);
			exit(1);
		}
	}

	t_tot += BATTERY_SWAP_TIME * (m_nM - 1);

	return t_tot;
}

// Return the distance (in meters) of partition n of this solution
float Solution::DistanceOfPartition(int n) {
	float dist = 0;

	if(!m_bSolved || (m_pAdjMatrix == NULL)) {
		// We didn't solve this problem yet!!
		dist = std::numeric_limits<float>::max();
	}
	else {
		// Create parent list and set all parents to -1
		int* parentList = new int[m_nNumVertices];
		for(int i = 0; i < m_nNumVertices; i++) {
			parentList[i] = -1;
		}

		// Run DFS to find path from depot to terminal
		int t = (m_nN + n), d = (m_nN + n + m_nM);
		// Set parent of each depot as itself
		parentList[d] = d;
		// Run DFS. Should return true
		if(!Graph_Theory_Algorithms::DFS_TopAdjMtrx(m_pAdjMatrix, parentList, m_nNumVertices, d, t)) {
			printf("[ERROR] Solution::DistanceOfPartition() : I' does not contain a path from %d to %d!\n", d, t);
			exit(1);
		}

		if(DEBUG_SOLUTION)
			printf("Calculating distance\n");
		int u = t, v = parentList[t];
		while(u != v) {
			if(DEBUG_SOLUTION)
				printf(" adding %d -> %d\n", u, v);
			dist += (m_pVertexData + u)->GetDistanceTo(m_pVertexData + v);
			u = v;
			v = parentList[v];
		}
	}

	return dist;
}

// Returns the distance of the maximum leg in this solution
float Solution::GetMaxLegDist() {
	float dist = 0;

	// Make sure we solved this solution
	if(!m_bSolved || !m_bFeasible) {
		// We didn't solve this problem yet!!
		dist = std::numeric_limits<float>::max();
	}
	else {
		for(int i = 0; i < m_nM; i++) {
			float temp = DistanceOfPartition(i);
			if(temp > dist) {
				dist = temp;
			}
		}
	}

	return dist;
}

// Returns the distance of the given tour of vertices found in lst
float Solution::GetTourDist(std::list<Vertex*> &lst) {
	float dist = 0;

	// Iterate through the list and find the distance between each given vertex
	std::list<Vertex *>::iterator it = lst.begin();
	Vertex * previous = (*it);
	it++;

	if(DEBUG_SOLUTION)
		printf("\nTour distance, list contains: %d ", previous->nID);

	for(; it != lst.end(); it++) {
		Vertex* next = (*it);

		if(DEBUG_SOLUTION)
			printf("%d ", next->nID);

		dist += previous->GetDistanceTo(next);
		previous = next;
	}

	if(DEBUG_SOLUTION)
		printf("\n");

	return dist;
}

// Return a pointer to the depot vertex of partition p
Vertex* Solution::GetDepotOfPartion(int p) {
	// Verify that they didn't ask for something out of bounds
	if(p >= m_nM) {
		printf("[ERROR] : Asked for a depot in partition p >= m!\n");
		p = m_nM - 1;
	}

	return m_pVertexData + m_nN + m_nM + p;
}

// Return a pointer to the terminal vertex of partition p
Vertex* Solution::GetTerminalOfPartion(int p) {
	// Verify that they didn't ask for something out of bounds
	if(p >= m_nM) {
		printf("[ERROR] : Asked for a terminal in partition p >= m!\n");
		p = m_nM - 1;
	}

	return m_pVertexData + m_nN + p;
}

// Determines if this solution is feasible given n, m, and radios R
bool Solution::IsFeasible() {
	// Determine the minimum size of the largest partition
	float minMaxPartSize = GetMinSpanningForestDistance() / (float)m_nM;

	return (DIST_MAX >= minMaxPartSize) && m_bFeasible;
}

// Returns the distance of a minimum spanning constrained forest containing m trees.
float Solution::GetMinSpanningForestDistance(bool findForest) {
	/*
	 * General idea here: The forest  must connect every depot and terminal to at least
	 * one destination vertex, therefore the forest will contain (n-1) - m edges of length
	 * R plus the shortest distance from each bs-stop to the closest way-point. The (n-1) - m
	 * comes from the fact that a min-spanning tree in the tessellation graph will have (n-1)
	 * edges, but we need to cut this tree into m pieces, so we do this by removing m of the
	 * edges. The returned distance is theoretical only (based on the above math), the actual
	 * minimum spanning constrained forest is never found.
	 */


	if(m_bSetup && (m_pAdjMatrix != NULL)) {
		// Find l_1
		float l_1 = std::numeric_limits<float>::max();
		Vertex* depot = GetDepotOfPartion(0);
		int closest_v = 0;
		// Run through all vertices, find closest to the depot
		for(int j = 0; j < m_nN; j++) {
			// Check distance to depot
			float dist_to_v = depot->GetDistanceTo(m_pVertexData + j);
			if(dist_to_v < l_1) {
				// Update shortest length value
				l_1 = dist_to_v;
				closest_v = j;
			}
		}
		if(findForest) {
			// Add these edges to the graph
			m_pAdjMatrix[closest_v][depot->nID] = true;
		}

		// Find l_2
		float l_2 = m_fR * (m_nN - m_nM);

		// Find l_3_naught
		float alpha_naught = l_1;
		float beta_naught = std::numeric_limits<float>::max();
		// Run through all vertices, find best beta, if it exists
		for(int j = 0; j < m_nN; j++) {
			if((m_pVertexData + j)->fX > 0) {
				if((m_pVertexData + j)->fY < beta_naught) {
					beta_naught = (m_pVertexData + j)->fY;
				}
			}
		}
		float l_3_naught = std::min(alpha_naught,beta_naught);

		/*
		 * Find l_3_m
		 *
		 * First, find point p_b_m. This is the minimum distance that the base-station could travel by
		 * the time the UAV finishes searching the entire search space
		 */

		if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
			// The following assumes a linear UGV... hard fail
			printf("[ERROR] : Solution::GetMinSpanningForestDistance : Given non-linear UGV, expected linear\n");
			exit(0);
		}

		float p_b_m = m_tBSTrajectory.mX * (l_1 + l_2 + 2*(m_nM - 1)*l_3_naught) / V_MAX;
		// Create a temporary vertex at point p_b_m
		Vertex v_temp;
		v_temp.fX = p_b_m;
		v_temp.fY = 0;
		// Find alpha
		float alpha_m = std::numeric_limits<float>::max();
		// Run through all vertices, find closest to v_temp. This distance is alpha_m
		for(int j = 0; j < m_nN; j++) {
			// Check distance to depot
			float dist_to_v = v_temp.GetDistanceTo(m_pVertexData + j);
			if(dist_to_v < alpha_m) {
				// Update shortest length value
				alpha_m = dist_to_v;
			}
		}
		// Find beta_m, if it exists
		float beta_m = std::numeric_limits<float>::max();
		// Run through all vertices, find best beta
		for(int j = 0; j < m_nN; j++) {
			if((m_pVertexData + j)->fX > v_temp.fX) {
				if((m_pVertexData + j)->fY < beta_m) {
					beta_m = (m_pVertexData + j)->fY;
				}
			}
		}
		float l_3_m = std::min(alpha_m, beta_m);

		// Find d_bl, the minimum distance that the UAV must travel to complete the search
		float d_bl = l_1 + l_2 + 2*(m_nM - 1)*l_3_naught + l_3_m;

		return d_bl;
	}
	else {
		return std::numeric_limits<float>::max();
	}
}

int root(int id[10], int x)
{
    while(id[x] != x)
    {
        id[x] = id[id[x]];
        x = id[x];
    }
    return x;
}

void union1(int id[10], int x, int y)
{
    int p = root(id, x);
    int q = root(id, y);
    id[p] = id[q];
}

/*
 * Returns the distance of a minimum spanning forest containing m trees. This should be used on
 * random graph-types!
 */
float Solution::GetMinSpanningForestDistanceRND() {
	// Initially, id[i] = i;
	int id[10], nodes, edges;
	// Edges array, <weight, <v, u>>
	std::pair <long long, std::pair<int, int> > p[10];
	int x, y;
	long long cost, minimumCost = 0;
	// Sort the edges in the ascending order
	sort(p, p + edges);
	for(int i = 0; i < edges; ++i) {
		// Selecting edges one by one in increasing order from the beginning
		x = p[i].second.first;
		y = p[i].second.second;
		cost = p[i].first;
		// Check if the selected edge is creating a cycle or not
		if(root(id, x) != root(id, y)) {
			minimumCost += cost;
			union1(id, x, y);
		}
	}
	return minimumCost;

	return 0;
}

/*
 * Returns the baseline time to run through the minimum spanning constrained forest
 */
float Solution::GetMinSpanningForestRT(bool findForest) {
	float time = 0;

	if(SANITY_PRINT)
		printf("Calculating Min-Spanning-Constrained-Forest Runtime for m = %d\n", m_nM);

	// We assume an even split of the tree
	float legDist = GetMinSpanningForestDistance(findForest) / (float)m_nM;

	if(SANITY_PRINT)
		printf(" legDist = %f\n", legDist);

	// Determine time per-leg based on distance/velocity constraint
	if(legDist > DIST_MAX) {
		if(SANITY_PRINT)
			printf(" TOO FAR!\n");

		return std::numeric_limits<float>::max();
	}
	else if(legDist <= DIST_OPT) {
		// Fly at max speed
		time = legDist * (1.0/V_MAX);

		if(SANITY_PRINT)
			printf(" go V_MAX = %f, t = %f\n", V_MAX, time);
	}
	else {
		// Determine fastest speed to move through this leg

		// Approximate inverse function of d(v)
		float v =  GetMaxVelocity(legDist);

		time = legDist * (1.0/v);

		if(SANITY_PRINT)
			printf(" go v = %f, t = %f\n", v, time);
	}

	// We do this amount of time per-leg
	time = time * (float)m_nM;
	// Add in the battery swap time
	time += BATTERY_SWAP_TIME * (float)(m_nM - 1);
	if(SANITY_PRINT)
		printf(" total time = %f\n", time);

	return time;
}

bool Solution::CorrectSolution(E_VelocityFlag velocityFlag) {
	bool ret_val = true;

	// Verify that the partitions have been setup
	if(!m_bPartitioned || !m_bFeasible) {
		ret_val = false;
	}
	else {
		if(SANITY_PRINT)
			printf("\nCorrecting Solution\n");

		// Correct each partition in turn
		for(int i = 0; i < m_nM; i++) {
			/// Correct the position of the depot
			if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
				// The following assumes a linear UGV... hard fail
				printf("[ERROR] : Solution::CorrectSolution : Given non-linear UGV, expected linear\n");
				exit(0);
			}

			if(i > 0) {
				GetDepotOfPartion(i)->fX = GetTerminalOfPartion(i-1)->fX + BATTERY_SWAP_TIME*this->m_tBSTrajectory.mX;
			}

			/// Correct the position of the terminal
			// Create parent list and set all parents to -1
			int* parentList = new int[m_nNumVertices];
			for(int i = 0; i < m_nNumVertices; i++) {
				parentList[i] = -1;
			}

			// Run DFS. Should return true
			int t = GetTerminalOfPartion(i)->nID, d = GetDepotOfPartion(i)->nID;
			parentList[d] = d;
			if(!Graph_Theory_Algorithms::DFS_TopAdjMtrx(m_pAdjMatrix, parentList, m_nNumVertices, d, t)) {
				printf(" I' does not contain a path from %d to %d!\n", d, t);
				m_bFeasible = false;
				ret_val = false;
			}
			else {
				// Add the stops on this tour (in correct order) to a list
				bool foundEnd = false;
				std::list<Vertex*> pathList;
				{
					// Start with ideal-stop
					int last = GetTerminalOfPartion(i)->nID;

					// Walk through path backwards
					while(parentList[last] != GetDepotOfPartion(i)->nID) {
						// Grab next stop
						int next = parentList[last];
						// Add to stack
						pathList.push_front(m_pVertexData + next);
						// Update
						last = next;
					}
					pathList.push_front(GetDepotOfPartion(i));
				}

				// Sanity print
				if(DEBUG_SOLUTION) {
					printf("Sub-tour %d:\n", i);
					for(Vertex* v : pathList) {
						printf(" %d", v->nID);
					}
					printf("\n");
				}

				// Determine the actual terminal position
				bool run_again = true;
				while(run_again) {
					float old_dist = DistanceOfPartition(i);
					if(old_dist > DIST_MAX) {
						run_again = false;
						m_bFeasible = false;
						ret_val = false;
					}
					else {
						float fV_u = GetMaxVelocity(old_dist, velocityFlag);
						determineTerminalLocation(pathList, fV_u, GetTerminalOfPartion(i));
						float new_dist = DistanceOfPartition(i);
						if(DEBUG_SOLUTION)
							printf(" old_dist: %f, velocity: %f, new_dist: %f, difference: %f\n", old_dist,
									fV_u, new_dist, abs(old_dist - new_dist));
						if(abs(old_dist - new_dist) <= ST_DIST_TOLERANCE) {
							run_again = false;
						}
					}
				}
			}
		}
	}

	return ret_val;
}


// Does the math to determine the maximum velocity the UAV can go for distance dist
double Solution::GetMaxVelocity(double dist, E_VelocityFlag velocityFlag) {
	if(velocityFlag == e_FixedMax) {
		return V_MAX;
	}
	else if(velocityFlag == e_FixedOpt) {
		return V_OPT;
	}
	else if(velocityFlag == e_FixedPowMin) {
		return V_OPT;
	}
	else if(velocityFlag == e_NotFixed) {
		if(dist <= DIST_OPT) {
			return V_MAX;
		}
		else {
			/*
			 * Function for velocity to distance:
			 * d(v) = 99,792v / (0.07v^3 + 0.0391v^2 - 13.196v + 390.95)
			 *
			 * Polynomial approximation of d(v):
			 * y = -12.357x^2 + 317.158x + 1431.037
			 *
			 * Inverse of polynomial approximation of d(v):
			 * v(d)= sqrt(36791437195 − 10557000d) / 10557 + 129221 / 10557
			 */

			// Approximate inverse function of d(v)
			return sqrt(36791437195.0 - 10557000.0 * dist) / 10557.0 + 129221.0/10557.0;
		}
	}
	else {
		// Something went wrong here...
		printf("[ERROR] : Solution::GetMaxVelocity() unexpected velocity flag, %d\n", velocityFlag);
		exit(1);
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************


/*
 * Determine the location of the terminal using sub-tour subTour and UAV velocity magnitude fV_u.
 *
 * This function assumes that the first vertex on the the list is the depot of the sub-tour. We
 * also assume that the base station's velocity vector only points in the positive x-direction
 * (i.e. y_b = 0).
 */
void Solution::determineTerminalLocation(std::list<Vertex*> &subTour, float fV_u, Vertex* terminal) {
	// Do some fun math here...

	if(subTour.size() > 1) {
		/// Determine distance from depot to UAV location
		float dist = GetTourDist(subTour);

		// Time to reach last way-point
		float t_l = dist/fV_u;

		// Sanity print
		if(DEBUG_SOLUTION)
			printf("  Found subTour dist = %f, time = %f\n", dist, t_l);

		if(m_tBSTrajectory.pd_type != E_TrajFuncType::e_StraightLine) {
			// The following assumes a linear UGV... hard fail
			printf("[ERROR] : Solution::determineTerminalLocation : Given non-linear UGV, expected linear\n");
			exit(0);
		}

		// New x coord of depot
		float x_b = subTour.front()->fX + m_tBSTrajectory.mX*t_l;
		// New x, y coords of UAV
		float x_u = subTour.back()->fX;
		float y_u = subTour.back()->fY;
		// Depot position vector relative to UAV
		float p_x = x_b - x_u;
		float p_y = -y_u;
		// Depot velocity vector
		float v_x = m_tBSTrajectory.mX;
		float v_y = 0;
		// UAV speed
		float s = fV_u;

		// The math to find the collision time is: || P + V*t_c || = s*t_c . This turns into a polynomial
		// First coefficient: V.V - s^2
		float a = (v_x*v_x + v_y*v_y - s*s);
		// Second coefficient: 2(P.V)
		float b = 2*(p_x*v_x + p_y*v_y);
		// Third coefficient: P.P
		float c = p_x*p_x + p_y*p_y;

		// Find roots of function
		Roots roots;
		roots.FindRoots(a, b, c);

		// Time till collision
		float t_c = 0;

		// Grab correct root
		if(roots.root1 >= 0) {
			t_c = roots.root1;
		}
		else if(roots.root2 >= 0) {
			t_c = roots.root2;
		}
		else {
			// Something isn't right...
			fprintf(stderr, "[ERROR] Solution : determineTerminalLocation() Did not find positive roots\n");
			// Hard-fail
			exit(1);
		}

		// Assign found values to new terminal
		float x_depot = x_b + t_c*m_tBSTrajectory.mX;
		if(DEBUG_SOLUTION)
			printf(" Depot moves from (%f, %f) to (%f, %f)\n", subTour.front()->fX, 0.0, x_depot, 0.0);
		terminal->fX = x_depot;
		terminal->fY = 0;
		terminal->nID = -1;
		terminal->eVType = E_VertexType::e_Terminal;
	}
	else {
		// Single depot is not a path!
		fprintf(stderr, "[ERROR] Solution_: determineTerminalLocation() was not given a path!\n");
		// Hard-fail
		exit(1);
	}
}

// Compares A and B to determine if they are within some tolerance of each other
bool Solution::compareFloatsAB(float A, float B) {
	return abs(A-B) < 0.05;
}
