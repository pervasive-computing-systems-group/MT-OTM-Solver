#include "Tree_Solver.h"

Tree_Solver::Tree_Solver() {
}

Tree_Solver::~Tree_Solver() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************


// Determine the distance of the given sub-tour
float Tree_Solver::getSubtourDist(std::list<Vertex*> &subtour) {
	// Determine length of new sub-tour
	std::__cxx11::list<Vertex *>::iterator prev = subtour.begin();
	std::__cxx11::list<Vertex *>::iterator next = subtour.begin();
	next++;

	// Walk the path up to the last stop
	float sub_dist = 0;
	do {
		sub_dist += (*prev)->GetDistanceTo(*next);
		prev = next;
		next++;
	} while(next != subtour.end());

	return sub_dist;
}

/*
 * Determine the location of the terminal using sub-tour subTour, base station velocity
 * magnitude fV_b, and UAV velocity magnitude fV_u.
 *
 * This function assumes that the current location of the base station (depot) is
 * the first vertex on the the list sub-tour. We also assume that the base station's
 * velocity vector only points in the positive x-direction (i.e. y_b = 0).
 */
void Tree_Solver::determineTerminalLocation(std::list<Vertex*> &subTour, float fV_b, float fV_u, Vertex* terminal) {
	// Do some fun math here...

	if(subTour.size() > 1) {
		/// Determine distance from depot to UAV location
		float dist = getSubtourDist(subTour);

		// Time to reach last way-point
		float t_l = dist/fV_u;

		// Sanity print
		if(TS_DEBUG)
			printf("  Found subTour dist = %f, time = %f\n", dist, t_l);

		// New x coord of depot
		float x_b = subTour.front()->fX + fV_b*t_l;
		// New x, y coords of UAV
		float x_u = subTour.back()->fX;
		float y_u = subTour.back()->fY;
		// Depot position vector relative to UAV
		float p_x = x_b - x_u;
		float p_y = -y_u;
		// Depot velocity vector
		float v_x = fV_b;
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
			fprintf(stderr, "[ERROR] TotalPath_Follower : determineTerminalLocation() Did not find positive roots\n");
			// Hard-fail
			exit(1);
		}

		// Assign found values to new terminal
		float x_depot = x_b + t_c*fV_b;
		if(TS_DEBUG)
			printf(" Depot moves from (%f, %f) to (%f, %f)\n", subTour.front()->fX, 0.0, x_depot, 0.0);
		terminal->fX = x_depot;
		terminal->fY = 0;
		terminal->nID = -1;
		terminal->eVType = E_VertexType::e_Terminal;
	}
	else {
		// Single depot is not a path!
		fprintf(stderr, "[ERROR] TotalPath_Follower : determineTerminalLocation() was not given a path!\n");
		// Hard-fail
		exit(1);
	}
}


//***********************************************************
// Private Member Functions
//***********************************************************


