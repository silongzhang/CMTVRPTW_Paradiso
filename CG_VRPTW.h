#pragma once

#include"ESPPRC.h"

typedef Label_ESPPRC Route_VRPTW;
typedef Consumption_ESPPRC Consumption_VRPTW;
typedef Cost_ESPPRC Cost_VRPTW;

class Solution_VRPTW {
private:
	vector<pair<double, Route_VRPTW>> routes;
	double cost;
public:
	vector<pair<double, Route_VRPTW>> getRoutes() const { return routes; }
	double getCost() const { return cost; }
	void reset() { routes.clear(); cost = 0; }
	void addRoute(const double weight, const Route_VRPTW &rhs) { routes.push_back(make_pair(weight, rhs)); cost += weight * rhs.getRealCost(); }
};


class Parameter_CG_VRPTW {
public:
	// Whether the solution can be fractional.
	bool canBeFractional;
	// Threshold value for solving the subproblem (ESPPRC) in an optimal style.
	double thresholdPercentNegArcs;
	// Whether printing is allowed.
	bool allowPrintLog;
};


class CG_VRPTW {
private:
	list<Route_VRPTW> columns;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW &rhs, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X);
	void InitiateRMP(const vector<Route_VRPTW> &initialRoutes, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X);
	Solution_VRPTW getSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X);
	Solution_VRPTW getAnIntegralSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X);
	Solution_VRPTW columnGeneration(const Data_Input_ESPPRC &inputESPPRC, const vector<Route_VRPTW> &initialRoutes, 
		const Parameter_CG_VRPTW &prm, ostream &output);
};


unordered_map<int, int> getCount(const vector<int> &vec);
double solveModel(IloCplex &solver);


