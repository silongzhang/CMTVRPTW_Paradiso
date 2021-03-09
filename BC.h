#pragma once
#include"BP.h"

class Parameter_BC {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Label_TimePath> columnPool;

	bool allowPrintLog;
};

class Solution_BC {
public:
	OptimalityStatus status;
	double objective;
	vector<Label_TimePath> routes;

	set<double, timeSortCriterion> timeSet;
	set<tuple<int, int, int>> tripletSet;
};

void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloBoolVarArray x);
void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloBoolVarArray x);

