#include "UGVTrajectory.h"

UGVTrajectory::UGVTrajectory() {
	x = 0;
	y =0;
	mX = 0;
	mY =0;
	mC = 0;
	pd_type = E_TrajFuncType::e_StraightLine;
}

UGVTrajectory::UGVTrajectory(const UGVTrajectory& traj) {
	x = traj.x;
	y = traj.y;
	mX = traj.mX;
	mY = traj.mY;
	mC = traj.mC;
	pd_type = traj.pd_type;
}

void UGVTrajectory::configTrajectory(float f_x, float f_y, float f_mX, float f_mY, E_TrajFuncType type) {
	x = f_x;
	y = f_y;
	mX = f_mX;
	mY = f_mY;
	mC = 0;
	pd_type = type;
}

void UGVTrajectory::configTrajectory(float f_x, float f_y, float f_mA, float f_mB, float f_mC, E_TrajFuncType type) {
	x = f_x;
	y = f_y;
	mX = f_mA;
	mY = f_mB;
	mC = f_mC;
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

	case E_TrajFuncType::e_Sinusoidal:
	{
		// Math...
		double t_m = x_g/mX;
		double half_period = mC/2;
		double tacking_time = std::max(t_m-half_period, 0.0);

		// Verify that we aren't too far off on the left of the UGV
		if(tacking_time > t_m+half_period) {
			// Too far off... return origin
			*x_u = 0;
			*y_u = 0;
		}
		else {
			double min_dist = 100000000000000000.0;
			while(tacking_time <= t_m + half_period) {
				double x_next, y_next;
				getPosition(tacking_time, &x_next, &y_next);

				double dist = std::sqrt(std::pow(x_next - x_g, 2) + std::pow(y_next - y_g, 2));
				if(dist < min_dist) {
					min_dist=dist;
					*x_u = x_next;
					*y_u = y_next;
				}
				tacking_time++;
			}
		}
	}

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

	case E_TrajFuncType::e_Sinusoidal:
		// x = a t, y = b sin((2pi t)/c)
		*x_u = mX*t;
		*y_u = mY*std::sin((2*PI*t)/mC);

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

	case E_TrajFuncType::e_Sinusoidal:
	{
		// x = a*t -> t = x/a
		time = x/mX;

		// Verify that y is reasonably to the correct position
		double correct_y = mY*std::sin((2*PI*time)/mC);
		if(std::abs(correct_y - y) > 2) {
			// This position isn't along the path...
			fprintf(stderr,"[ERROR] UGVTrajectory::getTimeAt() : Point (%f,%f) not along sinusoidal UGV path\n", x, y);
			exit(1);
		}
	}

		break;

	default:
		// Not implemented for this trajectory type...
		fprintf(stderr, "[ERROR] : UGVTrajectory::getTimeAt() : Function not implemented for give trajectory type: %d\n", pd_type);
		exit(0);
	}


	return time;
}
