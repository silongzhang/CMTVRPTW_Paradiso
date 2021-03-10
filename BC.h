#pragma once
#include"BP.h"

typedef IloBoolVarArray TypeX;

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
	int numBranch;
};

void setObjectiveBC(const vector<Label_TimePath>& structures, IloModel model, TypeX x);
void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, TypeX x);
Data_Input_VRPTW constructDataVRPTW(const Parameter_BC& parameter, const vector<Label_TimePath>& selectedStructures);
vector<Label_TimePath> getSolutionBCAlgorithm(const Parameter_BC& parameter, const IloCplex& cplex, const TypeX& X);
Solution_BC BCAlgorithm(const Parameter_BC& parameter);

