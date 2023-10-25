#include "UGVTrajectory.h"

UGVTrajectory::UGVTrajectory() {
	x = 0;
	y =0;
	mX = 0;
	mY =0;
	pd_type = E_TrajFuncType::e_StraightLine;
}

void UGVTrajectory::configTrajectory(float f_x, float f_y, float f_mX, float f_mY, E_TrajFuncType type) {
	x = f_x;
	y = f_y;
	mX = f_mX;
	mY = f_mY;
	pd_type = type;
}

UGVTrajectory::UGVTrajectory(const UGVTrajectory& traj) {
	x = traj.x;
	y = traj.y;
	mX = traj.mX;
	mY = traj.mY;
	pd_type = traj.pd_type;
}
