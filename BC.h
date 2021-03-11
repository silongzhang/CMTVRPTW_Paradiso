#pragma once

#include"BP.h"

class Parameter_VRPTW_BC {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Label_TimePath> columnPool;
	double weightLB;
	double weightDepth;
	bool allowPrintLog;
};

class Input_VRPTW_BC {
public:
	set<double, timeSortCriterion> timeSet;
	set<tuple<int, int, int>> tripletSet;
	vector<pair<vector<int>, double>> SFCSet;
	vector<pair<vector<int>, bool>> branchOnArcs;
};

class Info_VRPTW_BC {
public:
	int prunedInfeasibility;
	int prunedInteger;
	int prunedBound;
	int branched;
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

class NODE_VRPTW_BC {
	friend bool operator<(const NODE_VRPTW_BC& lhs, const NODE_VRPTW_BC& rhs);
public:
	int depth;
	double priority;

	Input_VRPTW_BC input;
	Solution_VRPTW_BC solution;

	void solve(const Parameter_VRPTW_BC& parameter, ostream& output);
	void setPriority(double weightLB, double weightDepth) { priority = weightLB * solution.objective + weightDepth * depth; }
};

void setConstraintsPartition_VRPTW_BC(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray X);
void addConstraintsTime_VRPTW_BC(const Parameter_VRPTW_BC& parameter, IloModel model, IloNumVarArray X, 
	const set<double, timeSortCriterion>& additionalTimes);
void addConstraintsSR_VRPTW_BC(const Parameter_VRPTW_BC& parameter, IloModel model, IloNumVarArray X,
	const set<tuple<int, int, int>>& additionalTriplets);
void addConstraintsSFC_VRPTW_BC(IloModel model, IloNumVarArray X, const vector<pair<vector<int>, double>>& SFCSet);
void addConstraintsBranchOnArcs_VRPTW_BC(IloModel model, IloNumVarArray X, const vector<pair<vector<int>, bool>>& branchOnArcs);

