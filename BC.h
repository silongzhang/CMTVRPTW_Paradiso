#pragma once

#include"BP.h"
#include"CuttingPlane.h"

class Parameter_VRPTW_BC {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Label_TimePath> columnPool;
	double weightLB;
	double weightDepth;
	bool ArcFlowRatherThanBP;					// true: solve TOPTW by the arc-flow model; false: by BP.
	bool allowPrintLog;
};

class ConstraintSet {
public:
	set<double, timeSortCriterion> timeSet;
	set<tuple<int, int, int>> tripletSet;
	vector<pair<vector<int>, double>> SFCSet;
};

class Info_VRPTW_BC {
public:
	int prunedInfeasibility;
	int prunedInteger;
	int prunedBound;
	int branched;
};

class Input_VRPTW_BC {
public:
	vector<pair<vector<int>, bool>> branchOnArcs;
};

class Solution_VRPTW_BC {
public:
	bool feasible;									// Whether the LP formulation corresponding to this node is feasible.
	bool integer;									// Whether the optimal solution corresponding to this node is an integer solution.
	double objective;								// The optimal objective value corresponding to this node.
	double UB_Integer_Value;						// The upperbound.
	vector<Label_TimePath> UB_Integer_Solution;		// The integer solution.
	vector<int> Indices_UB_Integer_Solution;		// Indices of routes of the integer solution.

	vector<vector<double>> visitArcs;				// 0 <= visitArcs[i][j] <=1, the times at which the arc (i, j) has been visited.
};

class NODE_VRPTW_BC {
	friend bool operator<(const NODE_VRPTW_BC& lhs, const NODE_VRPTW_BC& rhs);
public:
	int depth;
	double priority;

	Input_VRPTW_BC input;
	Solution_VRPTW_BC solution;

	void solve(const Parameter_VRPTW_BC& parameter, ConstraintSet& constraints, ostream& output);
	void setPriority(double weightLB, double weightDepth) { priority = weightLB * solution.objective + weightDepth * depth; }
	void getIntegerSolution(const Parameter_VRPTW_BC& parameter, const IloCplex& cplex, const IloNumVarArray& X);
	void getVisitArcs(const Parameter_VRPTW_BC& parameter, const IloCplex& cplex, const IloNumVarArray& X);
};

void setConstraintsPartition_VRPTW_BC(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray X);
void addConstraintsTime_VRPTW_BC(const Parameter_VRPTW_BC& parameter, IloModel model, IloNumVarArray X, 
	const set<double, timeSortCriterion>& additionalTimes);
void addConstraintsSR_VRPTW_BC(const Parameter_VRPTW_BC& parameter, IloModel model, IloNumVarArray X,
	const set<tuple<int, int, int>>& additionalTriplets);
void addConstraintsSFC_VRPTW_BC(IloModel model, IloNumVarArray X, const vector<pair<vector<int>, double>>& SFCSet);
void addConstraintsBranchOnArcs_VRPTW_BC(IloModel model, IloNumVarArray X, const vector<pair<vector<int>, bool>>& branchOnArcs);
NODE_VRPTW_BC initBCNode(const Parameter_VRPTW_BC& parameter);
vector<int> traverse(const Parameter_VRPTW_BC& parameter, int tail, int head);
NODE_VRPTW_BC childNode(const Parameter_VRPTW_BC& parameter, const NODE_VRPTW_BC& rhs);
void printBranchParameter(const NODE_VRPTW_BC& worker);
NODE_VRPTW_BC BCAlgorithm(const Parameter_VRPTW_BC& parameter, ostream& output);

