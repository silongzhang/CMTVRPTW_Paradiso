#pragma once

#include"ESPPRC.h"
#include"VRPTW.h"
#include"RSFC.h"

class Parameter_TOPTW_CG {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Route_VRPTW> initialRoutes;
	bool allowPrintLog;
};

class TOPTW_CG {
private:
	bool feasible;
	bool integer;
	double value;
	bool explored;
	vector<Route_VRPTW> columns;
	vector<Route_VRPTW> integerSolution;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW& rhs, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void InitiateRMP(const vector<Route_VRPTW>& initialRoutes, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void columnGeneration(const Parameter_TOPTW_CG& parameter, ostream& output);
	void getIntegerSolution(const IloCplex& cplex, const IloNumVarArray& X);
};

void renewReducedCost(Data_Input_ESPPRC& inputESPPRC, const Parameter_TOPTW_CG& parameter, const IloNumArray& dualValue);
bool isBool(const IloCplex& cplex, const IloNumVarArray& X);

