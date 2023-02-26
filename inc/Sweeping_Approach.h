/*
 * Sweeping_Approach.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 14, 2021
 *
 * Description:
 */

#pragma once

#include <math.h>

#include "Vertex.h"
#include "Solution.h"


#define PI		3.14159265

// Radios of camera FOV
#define EPSILON_R	0.1

struct T_ColumnV {
	Vertex* pV;

	T_ColumnV() {
		pV = NULL;
	}

	T_ColumnV(Vertex* v) {
		pV = v;
	}
};

struct T_RowV {
	Vertex* pV;

	T_RowV() {
		pV = NULL;
	}

	T_RowV(Vertex* v) {
		pV = v;
	}
};


// Operator overload to compare two T_ColumnV in the same column
inline bool operator<(const T_ColumnV &a, const T_ColumnV &b) {
	return a.pV->fY < b.pV->fY;
}

// Operator overload to compare two T_RowV in the same row
inline bool operator<(const T_RowV &a, const T_RowV &b) {
	return a.pV->fX < b.pV->fX;
}


class Sweeping_Approach {
public:
	Sweeping_Approach();
	~Sweeping_Approach();

protected:
	// Determines which column vertex v belongs in
	int getColumn(Vertex* v, float R);
	// Determines which column vertex v belongs in based on given A and B
	int getColumn(Vertex* v, Vertex* pA, Vertex* pB, float R);
	// Determines which row vertex v belongs in
	int getRow(Vertex* v, float R);
	// Determines which row vertex v belongs in
	int getRow(Vertex* v, Vertex* pA, Vertex* pB, Vertex* pC, float R);
	// Runs through the given solution and finds A, B, and C of a reference triangle for the tessellation graph
	void findReferenceTriangle(Solution* solution, float R);
	// Determines the orientation of A, B and C. Assigns these pointers to pMinV, pMidV, and pTopV
	void determineTriangleOrientation(Vertex* &pMinV, Vertex* &pMidV, Vertex* &pTopV);

	// The vertices of the reference triangle
	Vertex* m_pA;
	Vertex* m_pB;
	Vertex* m_pC;

private:

};
