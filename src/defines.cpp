#include <defines.h>

// Determines if f is relatively close to zero. Avoids comparing floating-point numbers to zero
bool isZero(double f) {
	return (f < 0.0001) && (f > -0.0001);
}

std::string itos(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

// Determines if a is "roughly" the same as b. Avoids comparing floating-point numbers
bool equalFloats(double a, double b) {
	double diff = a - b;
	return (diff < 0.001) && (diff > -0.001);
}
