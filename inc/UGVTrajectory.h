/*
 * UGVTrajectory.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 24, 2023
 *
 * Description: Class that holds the trajectory of the UGV. This is p_d(t) from the paper.
 */

#pragma once

#include "defines.h"


class UGVTrajectory {
public:
	UGVTrajectory();
	UGVTrajectory(const UGVTrajectory& traj);

	void configTrajectory(float f_x, float f_y, float f_mX, float f_mY, E_TrajFuncType type);
	// Determines the closest point along the trajectory of the UGV (x_u, y_u) to the given point (x_g, y_g)
	void ClostestPoint(double x_g, double y_g, double* x_u, double* y_u);
	// Gets point along the trajectory of the UGV (x_u, y_u) at time t
	void getPosition(double t, double* x_u, double* y_u);
	/*
	 * Returns the time associated that the UGV will be at a give x,y coordiante. We assume
	 * that the trajectory function is invertible (i.e. there is a unique solution for any
	 * input). Function will exit(0) if the given point is not reasonably close to the actual
	 * trajectory.
	 */
	double getTimeAt(double x, double y);

	float x;
	float y;
	float mX;
	float mY;
	E_TrajFuncType pd_type;

private:
};

