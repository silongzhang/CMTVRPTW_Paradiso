#pragma once

#include"BP.h"

class Parameter_BC {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Label_TimePath> columnPool;
	double weightLB;
	double weightDepth;
	bool allowPrintLog;
};

class Info_BC {
public:
	int prunedInfeasibility;
	int prunedInteger;
	int prunedBound;
	int branched;
};

class Parameter_VRPTW_BC {
public:
	vector<pair<vector<int>, bool>> branchOnArcs;
};

class Solution_VRPTW_BC {
public:
	bool explored;									// Whether this node has been explored.
	bool feasible;									// Whether the LP formulation corresponding to this node is feasible.
	bool integer;									// Whether the optimal solution corresponding to this node is an integer solution.
	double objective;								// The optimal objective value corresponding to this node.
	double UB_Integer_Value;						// The best (smallest) upperbound found so far.
	vector<Label_TimePath> UB_Integer_Solution;		// The best integer solution found so far.
};


