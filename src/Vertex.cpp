#include "Vertex.h"

// Default constructor
Vertex::Vertex() {
	nID = -1;
	fX = -1;
	fY = -1;
	eVType = e_Destination;
}

Vertex::Vertex(const Vertex &u) {
	nID = u.nID;
	fX = u.fX;
	fY = u.fY;
	eVType = u.eVType;
}

// Full implementation constructor
Vertex::Vertex(int id, float x, float y, E_VertexType vType) {
	nID = id;
	fX = x;
	fY = y;
	eVType = vType;
}

// Destructor
Vertex::~Vertex() { }

// Returns the distance (in meters) from this vertex to v
float Vertex::GetDistanceTo(Vertex* v) {
	return sqrt(pow((fX - v->fX), 2) + pow((fY - v->fY), 2));
}
