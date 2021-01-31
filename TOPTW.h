#pragma once

#include"ESPPRC.h"
#include"VRPTW.h"
#include"RSFC.h"

class Parameter_TOPTW_CG {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Route_VRPTW> initialRoutes;
	bool allowPrintLog;

	unordered_map<int, bool> branchOnVertices;			// [i, true] means vertex i must be visited; false means vertex i cannot be visited.
	unordered_map<pair<int, int>, bool> branchOnArcs;	// [(i, j), true] means arc (i, j) must be visited if i and j are visited; 
														// false means (i, j) cannot be visited.
	pair<int, bool> branchOnVehicleNumber;				// [m, true] means the number of vehicles must be greater than or equal to m;
														// false means the number of vehicles must be less than or equal to m.
	int numArtificial;									// The number of artificial variables.

	void reviseInputVRPTW();
};

class Solution_TOPTW_CG {
public:
	bool explored;
	bool feasible;
	bool integer;
	double LB_Linear;
	double UB_Integer;
	vector<Route_VRPTW> integerSolution;
};

class TOPTW_CG {
private:
	vector<Route_VRPTW> columns;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW& rhs, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void InitiateRMP(const vector<Route_VRPTW>& initialRoutes, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void columnGeneration(const Parameter_TOPTW_CG& parameter, Solution_TOPTW_CG& solution, ostream& output);
	void getIntegerSolution(const IloCplex& cplex, const IloNumVarArray& X, Solution_TOPTW_CG& solution);
};

class BBNODE {
private:
	Parameter_TOPTW_CG parameter;
	Solution_TOPTW_CG solution;
	TOPTW_CG model;
public:
	// Revise the input parameter.

	// Solve the model.

};

void setRangeArray(const Parameter_TOPTW_CG& parameter, IloModel& modelRMP, IloRangeArray& constraintRMP);
void renewReducedCost(Data_Input_ESPPRC& inputESPPRC, const Parameter_TOPTW_CG& parameter, const IloNumArray& dualValue);
bool isBool(const IloCplex& cplex, const IloNumVarArray& X);
bool isFeasible(const Parameter_TOPTW_CG& parameter, const IloCplex& cplex, const IloNumVarArray& X);

