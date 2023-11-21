/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 05, 2021
 *
 * Description: Header file for general project defines
 */

#pragma once

#include <math.h>
#include <stdio.h>
#include <sstream>

// Project-wide debug flag
#define DEBUG			0
#define SANITY_PRINT	0 || DEBUG

// The length of each triangle side
#define PI			3.14159265

// UAV mobility parameters
#define DIST_MAX	3441.0
#define DIST_OPT	3000.0
#define DIST_POW_MIN	2312.1
#define V_MAX		19.0
#define V_OPT		14.0
#define V_POW_MIN	7.5
#define BATTERY_SWAP_TIME	90.0 // in seconds
// Speed function
#define C1 36791437195.0
#define C2 10557000.0
#define C3 10557.0
#define C4 12.24

// Fixed-Velocity flags
enum E_VelocityFlag {
	e_NotFixed = 0,
	e_FixedMax = 1,
	e_FixedOpt = 2,
	e_FixedPowMin = 3,
	e_Default = 10
};

// UGV trajectory function type
enum E_TrajFuncType {
	e_StraightLine = 0,
	e_Sinusoidal = 1
};

// Determines if f is relatively close to zero. Avoids comparing floating-point numbers to zero
bool isZero(double f);
// Takes in an integer and returns a string of said integer
std::string itos(int i);
// Determines if a is "roughly" the same as b. Avoids comparing floating-point numbers
bool equalFloats(double a, double b);
