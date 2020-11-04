#pragma once

#include"ESPPRC.h"

typedef Label_ESPPRC Route_VRPTW;


class Solution_VRPTW {
private:
	vector<pair<double, Route_VRPTW>> routes;
	double cost;
public:

};


class CG_VRPTW {
private:
	list<Route_VRPTW> columns;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW &rhs, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X);
	void InitiateRMP(const vector<Route_VRPTW> &initialRoutes, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X);
	Solution_VRPTW columnGeneration(const Data_Input_ESPPRC &inputESPPRC, const vector<Route_VRPTW> &initialRoutes, ostream &output);
};


unordered_map<int, int> getCount(const vector<int> &vec);


