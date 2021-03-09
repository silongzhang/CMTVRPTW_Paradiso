#pragma once
#include"BP.h"

class Parameter_BC {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Label_TimePath> columnPool;
};

class Solution_BC {
public:
	OptimalityStatus status;
	double objective;
	vector<Label_TimePath> routes;

	set<double, timeSortCriterion> timeSet;
	set<tuple<int, int, int>> tripletSet;
	vector<pair<vector<int>, double>> SFCSet;
};

void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloBoolVarArray x);
void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloBoolVarArray x);
Data_Input_VRPTW constructDataVRPTW(const Parameter_BC& parameter, const vector<Label_TimePath>& selectedStructures);
vector<Label_TimePath> getSolutionBCAlgorithm(const Parameter_BC& parameter, const IloCplex& cplex, const IloBoolVarArray& X);
Solution_BC BCAlgorithm(const Parameter_BC& parameter);

