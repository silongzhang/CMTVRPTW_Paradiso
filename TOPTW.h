#pragma once

#include"ESPPRC.h"
#include"VRPTW.h"

class Parameter_TOPTW_CG {
public:
	Data_Input_VRPTW input_VRPTW;
	Data_Input_ESPPRC inputESPPRC;
	vector<Route_VRPTW> initialRoutes;
	bool allowPrintLog;
};

class TOPTW_CG {
private:
	int depth;
	bool explored;

	bool integral;
	double upperBound;
	list<Route_VRPTW> columns;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW& rhs, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void InitiateRMP(const Parameter_TOPTW_CG& parameter, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void columnGeneration(const Parameter_TOPTW_CG& parameter, ostream& output);
};

void renewReducedCost(Data_Input_ESPPRC& inputESPPRC, const Parameter_TOPTW_CG& parameter, const IloNumArray& dualValue);


