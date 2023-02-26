/*
 * Edge.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 01, 2021
 *
 * Description: Base-class for graph edge classes.
 */

#pragma once

class Edge {
public:
	Edge();
	virtual ~Edge();
	virtual float getWeight() = 0;

private:
};
