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

	float x;
	float y;
	float mX;
	float mY;
	E_TrajFuncType pd_type;

private:
};

