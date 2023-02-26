/*
 * graph_defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 01, 2021
 *
 * Description: Header file to hold graph related defines.
 */

#pragma once

#include <math.h>
#include <stdio.h>

//class Vertex;
class VertexOnEdge;

enum E_VertexType {
	e_Depot,
	e_Terminal,
	e_Destination
};

struct T_BipartitEdge {
	int ID;
	float nWeight;
	VertexOnEdge* pv_from;
	VertexOnEdge* pv_to;
};
