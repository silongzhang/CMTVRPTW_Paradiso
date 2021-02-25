#pragma once

#include"TOPTW.h"

class BBNODE {
public:
	int depth;
	double priority;

	Parameter_TOPTW_CG parameter;
	Solution_TOPTW_CG solution;
	TOPTW_CG model;

	void reviseParameter();
	void solve(ostream& output);
};

BBNODE generateRootNode(const Data_Input_VRPTW& inputVRPTW);

