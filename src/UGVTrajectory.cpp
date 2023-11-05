#include "UGVTrajectory.h"

UGVTrajectory::UGVTrajectory() {
	x = 0;
	y =0;
	mX = 0;
	mY =0;
	pd_type = E_TrajFuncType::e_StraightLine;
}

UGVTrajectory::UGVTrajectory(const UGVTrajectory& traj) {
	x = traj.x;
	y = traj.y;
	mX = traj.mX;
	mY = traj.mY;
	pd_type = traj.pd_type;
}

void UGVTrajectory::configTrajectory(float f_x, float f_y, float f_mX, float f_mY, E_TrajFuncType type) {
	x = f_x;
	y = f_y;
	mX = f_mX;
	mY = f_mY;
	pd_type = type;
}

// Determines the closest point along the trajectory of the UGV (x_u, y_u) to the given point (x_g, y_g)
void UGVTrajectory::ClostestPoint(double x_g, double y_g, double* x_u, double* y_u) {
	// This depends on the trajectory function..
	switch(pd_type) {
	case E_TrajFuncType::e_StraightLine:
		// Straight line -> should be on x-axis -> just return x_g
		*x_u = x_g;
		*y_u = 0;
		break;

	default:
		// Not implemented for this trajectory type...
		fprintf(stderr, "[ERROR] : UGVTrajectory::ClostestPoint : Function not implemented for give trajectory type: %d\n", pd_type);
		exit(0);
	}
}

// Gets point along the trajectory of the UGV (x_u, y_u) at time t
void UGVTrajectory::getPosition(double t, double* x_u, double* y_u) {
	// This depends on the trajectory function..
	switch(pd_type) {
	case E_TrajFuncType::e_StraightLine:
		// Straight line -> should be on x-axis
		*y_u = 0;
		// x = t*v_x
		*x_u = t*mX;

		break;

	default:
		// Not implemented for this trajectory type...
		fprintf(stderr, "[ERROR] : UGVTrajectory::getPosition() : Function not implemented for give trajectory type: %d\n", pd_type);
		exit(0);
	}
}

/*
 * Returns the time associated that the UGV will be at a give x,y coordiante. We assume
 * that the trajectory function is invertible (i.e. there is a unique solution for any
 * input). Function will exit(0) if the given point is not reasonably close to the actual
 * trajectory.
 */
double UGVTrajectory::getTimeAt(double x, double y) {
	double time = 0;

	// This depends on the trajectory function..
	switch(pd_type) {
	case E_TrajFuncType::e_StraightLine:
		// Straight line -> x,y coord should be on x-axis
		if(!isZero(y)) {
			fprintf(stderr,"[ERROR] UGVTrajectory::getTimeAt() : Linear UGV but asked for time at non-zero y position\n");
			exit(0);
		}

		// x = t*v_x => t = x/v_x
		time = x/mX;

		break;

	default:
		// Not implemented for this trajectory type...
		fprintf(stderr, "[ERROR] : UGVTrajectory::getTimeAt() : Function not implemented for give trajectory type: %d\n", pd_type);
		exit(0);
	}


	return time;
}
