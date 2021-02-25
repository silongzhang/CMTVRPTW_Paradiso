#pragma once

#include"TOPTW.h"

class Parameter_BP {
public:
	double weightLB;
	double weightDepth;
};

class BBNODE {
	friend bool operator<(const BBNODE& lhs, const BBNODE& rhs);
public:
	int depth;
	double priority;

	Parameter_TOPTW_CG parameter;
	Solution_TOPTW_CG solution;
	TOPTW_CG model;

	void reviseParameter();
	void solve(ostream& output);
	void setPriority(double weightLB, double weightDepth) { priority = weightLB * solution.objective + weightDepth * depth; }
};

BBNODE generateRootNode(const Data_Input_VRPTW& inputVRPTW, const Parameter_BP& parameter);

