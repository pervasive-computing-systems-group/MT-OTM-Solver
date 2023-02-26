#include "Sweeping_Approach.h"

Sweeping_Approach::Sweeping_Approach() {
	m_pA = NULL;
	m_pB = NULL;
	m_pC = NULL;
}

Sweeping_Approach::~Sweeping_Approach() {
}


//***********************************************************
// Public Member Functions
//***********************************************************


//***********************************************************
// Protected Member Functions
//***********************************************************

// Determines which column vertex v belongs in
int Sweeping_Approach::getColumn(Vertex* v, float R) {
	// Find the closest point, (x_, y_), from v on the straight line connecting reference vertices A and B
	float m = (m_pB->fY - m_pA->fY)/(m_pB->fX - m_pA->fX);
	float a;
	float b;
	float c;

	if(abs(m) < 0.005) {
		// Slop is 0!
		a = 0;
		b = 1;
		c = -1*m_pA->fY;
	}
	else {
		a = -1*m;
		b = 1;
		c = m*m_pA->fX - m_pA->fY;
	}

	// Formula to find (x_, y_): https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	float x_ = (b*(b*v->fX - a*v->fY) - a*c)/(a*a + b*b);
	float y_ = (a*(-1*b*v->fX + a*v->fY) - b*c)/(a*a + b*b);
	float v_dist = sqrt(pow(x_ - v->fX, 2) + pow(y_ - v->fY, 2));
	float index = v_dist / (R*sin(60*PI/180));

	// Determine if we need to change signs
	if(abs(m) < 0.005) {
		if(y_ > v->fY) {
			index *= -1;
		}
	}
	else {
		if(x_ > v->fX) {
			index *= -1;
		}
	}

	return (int)roundf(index);
}

// Determines which column vertex v belongs in based on given A and B
int Sweeping_Approach::getColumn(Vertex* v, Vertex* pA, Vertex* pB, float R) {
	// Find the closest point, (x_, y_), from v on the straight line connecting reference vertices A and B
	float m = (pB->fY - pA->fY)/(pB->fX - pA->fX + 0.00005);
	float a = -1*m;
	float b = 1;
	float c = m*pA->fX - pA->fY;

	// Formula to find (x_, y_): https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	float x_ = (b*(b*v->fX - a*v->fY) - a*c)/(a*a + b*b);
	float y_ = (a*(-1*b*v->fX + a*v->fY) - b*c)/(a*a + b*b);
	float v_dist = sqrt(pow(x_ - v->fX, 2) + pow(y_ - v->fY, 2));
	float index = v_dist / (R*sin(60*PI/180));

	if(x_ > v->fX) {
		// Change signs
		index *= -1;
	}

	return (int)roundf(index);

}

// Determines which column vertex v belongs in
int Sweeping_Approach::getRow(Vertex* v, float R) {
	// Find the reference point, (x_r, y_r), on the ab leg of the triangle to base a line through vertex C
	float m_r = (m_pB->fY - m_pA->fY)/(m_pB->fX - m_pA->fX + 0.00005);
	float a_r = -1*m_r;
	float b_r = 1;
	float c_r = m_r*m_pA->fX - m_pA->fY;

	// Formula to find (x_r, y_r)
	float x_r = (b_r*(b_r*m_pC->fX - a_r*m_pC->fY) - a_r*c_r)/(a_r*a_r + b_r*b_r);
	float y_r = (a_r*(-1*b_r*m_pC->fX + a_r*m_pC->fY) - b_r*c_r)/(a_r*a_r + b_r*b_r);


	// Find the closest point, (x_, y_), from v on the straight line connecting reference point (x_r, y_r) and vertex C
	float m = (m_pC->fY - y_r)/(m_pC->fX - x_r + 0.00005);
	float a = -1*m;
	float b = 1;
	float c = m*x_r - y_r;

	// Formula to find (x_, y_)
	float x_ = (b*(b*v->fX - a*v->fY) - a*c)/(a*a + b*b);
	float y_ = (a*(-1*b*v->fX + a*v->fY) - b*c)/(a*a + b*b);
	float v_dist = sqrt(pow(x_ - v->fX, 2) + pow(y_ - v->fY, 2));
	float index = v_dist/(R*0.5);

	if(y_ > v->fY) {
		// Change signs
		index *= -1;
	}

	return (int)roundf(index);
}

// Determines which row vertex v belongs in
int Sweeping_Approach::getRow(Vertex* v, Vertex* pA, Vertex* pB, Vertex* pC, float R) {
	// Find the reference point, (x_r, y_r), on the ab leg of the triangle to base a line through vertex C
	float m_r = (pB->fY - pA->fY)/(pB->fX - pA->fX + 0.00005);
	float a_r = -1*m_r;
	float b_r = 1;
	float c_r = m_r*pA->fX - pA->fY;

	// Formula to find (x_r, y_r)
	float x_r = (b_r*(b_r*pC->fX - a_r*pC->fY) - a_r*c_r)/(a_r*a_r + b_r*b_r);
	float y_r = (a_r*(-1*b_r*pC->fX + a_r*pC->fY) - b_r*c_r)/(a_r*a_r + b_r*b_r);


	// Find the closest point, (x_, y_), from v on the straight line connecting reference point (x_r, y_r) and vertex C
	float m = (pC->fY - y_r)/(pC->fX - x_r + 0.00005);
	float a = -1*m;
	float b = 1;
	float c = m*x_r - y_r;

	// Formula to find (x_, y_)
	float x_ = (b*(b*v->fX - a*v->fY) - a*c)/(a*a + b*b);
	float y_ = (a*(-1*b*v->fX + a*v->fY) - b*c)/(a*a + b*b);
	float v_dist = sqrt(pow(x_ - v->fX, 2) + pow(y_ - v->fY, 2));
	float index = v_dist/(R*0.5);

	if(y_ > v->fY) {
		// Change signs
		index *= -1;
	}

	return (int)roundf(index);

}

// Runs through the given solution and finds A, B, and C of a reference triangle for the tessellation graph
void Sweeping_Approach::findReferenceTriangle(Solution* solution, float R) {
	// Start searching for three vertices that are all next to each other
	do {
		// Grab the first reference vertex
		m_pA = solution->m_pVertexData + (rand() % solution->m_nN);

		// Search for two adjacent vertices that form a reference triangle
		m_pB = NULL;
		m_pC = NULL;
		for(int i = 1; i < solution->m_nN; i++) {
			if(((solution->m_pVertexData + i) != m_pA) && (m_pA->GetDistanceTo(solution->m_pVertexData + i) < (R + EPSILON_R))) {
				// Found a vertex one-hop away from a
				if(m_pB == NULL) {
					// Make this vertex b
					m_pB = (solution->m_pVertexData + i);
				}
				else {
					// Verify that this vertex is also one-hop away from b
					if(m_pB->GetDistanceTo(solution->m_pVertexData + i) < (R + EPSILON_R)) {
						// Found vertex c
						m_pC = (solution->m_pVertexData + i);
					}
				}
			}
		}
	} while((m_pB == NULL) || (m_pC == NULL));
}

// Determines the orientation of A, B and C. Assigns these pointers to pMinV, pMidV, and pTopV
void Sweeping_Approach::determineTriangleOrientation(Vertex* &pMinV, Vertex* &pMidV, Vertex* &pTopV) {
	if((m_pA == NULL) || (m_pB == NULL) || (m_pC == NULL)) {
		printf("[ERROR] : determineTriangleOrientation(), asked for orientation but A, B or C is NULL!");
		exit(0);
	}

	// Determine the orientation of the reference triangle
	if((m_pA->fY < m_pB->fY) && (m_pA->fY < m_pC->fY)) {
		// A is the lowest
		pMinV = m_pA;
		if(m_pB->fY < m_pC->fY) {
			pMidV = m_pB;
			pTopV = m_pC;
		}
		else {
			pMidV = m_pC;
			pTopV = m_pB;
		}
	}
	else if((m_pB->fY < m_pA->fY) && (m_pB->fY < m_pC->fY)) {
		// B is the lowest
		pMinV = m_pB;
		if(m_pA->fY < m_pC->fY) {
			pMidV = m_pA;
			pTopV = m_pC;
		}
		else {
			pMidV = m_pC;
			pTopV = m_pA;
		}
	}
	else {
		// C is the lowest
		pMinV = m_pC;
		if(m_pA->fY < m_pB->fY) {
			pMidV = m_pA;
			pTopV = m_pB;
		}
		else {
			pMidV = m_pB;
			pTopV = m_pA;
		}
	}
}

//***********************************************************
// Private Member Functions
//***********************************************************
