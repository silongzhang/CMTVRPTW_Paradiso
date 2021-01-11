#pragma once

#include"Enumeration_TimePath.h"

class timeSortCriterion {
public:
	bool operator()(const double lhs, const double rhs) const {
		return lessThanReal(lhs, rhs, PPM);
	}
};

class outputRSFC {
public:
	double objective;
};

