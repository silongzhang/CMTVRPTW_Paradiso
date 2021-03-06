#pragma once

#include"ESPPRC.h"

typedef Label_ESPPRC Route_VRPTW;
typedef Consumption_ESPPRC Consumption_VRPTW;
typedef Cost_ESPPRC Cost_VRPTW;

class Solution_VRPTW_CG {
private:
	vector<pair<double, Route_VRPTW>> routes;
	double cost;
	Data_Input_ESPPRC input;
public:
	vector<pair<double, Route_VRPTW>> getRoutes() const { return routes; }
	double getCost() const { return cost; }
	Data_Input_ESPPRC getInput() const { return input; }
	void reset() { routes.clear(); cost = 0; }
	void setInput(const Data_Input_ESPPRC& rhs) { input = rhs; }
	void addRoute(const double weight, const Route_VRPTW &rhs) { routes.push_back(make_pair(weight, rhs)); cost += weight * rhs.getRealCost(); }
	void print(ostream &output) const;
};


class Parameter_VRPTW_CG {
public:
	// Whether the solution can be fractional.
	bool canBeFractional;
	// Whether printing is allowed.
	bool allowPrintLog;
};


class VRPTW_CG {
private:
	list<Route_VRPTW> columns;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW &rhs, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X);
	void InitiateRMP(const vector<Route_VRPTW> &initialRoutes, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X);
	Solution_VRPTW_CG getSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X);
	Solution_VRPTW_CG getAnIntegralSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X);
	Solution_VRPTW_CG columnGeneration(const Data_Input_ESPPRC &inputESPPRC, const vector<Route_VRPTW> &initialRoutes, 
		const Parameter_VRPTW_CG &prm, ostream &output);
};


unordered_map<int, int> getCount(const vector<int> &vec);
double solveModel(IloCplex &solver);


