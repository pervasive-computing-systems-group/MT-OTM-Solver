/*
 * Bae_Rathinam.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 30, 2021
 *
 * Description:
 */

#pragma once

#include "Partitioner.h"


class Bae_Rathinam : public Partitioner{
public:
	Bae_Rathinam();
	~Bae_Rathinam();

protected:
	/*
	 * Run the Bae-Rathinam algorithm, which finds the optimal constrained forest
	 * for the underlying graph with m trees (m is the number of depot/terminal pairs).
	 */
	void RunAlgorithm(Solution* solution);

private:
	// Determines if edges a and b are a beta swap in Matroid 1, as defined in the Bae-Rathinam algorithm
	bool M1_beta_swap(DEdge*** g, int g_size, std::map<Edge*, VertexOnEdge*> set_A, Edge* a, Edge* b, int l_limit, int u_limit);
	// Determines if edges a and b are a beta swap in Matroid 2, as defined in the Bae-Rathinam algorithm
	bool M2_beta_swap(DEdge*** g, int g_size, std::map<Edge*, VertexOnEdge*> set_A, Edge* a, Edge* b, int m, int n);
};
