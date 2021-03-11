#pragma once
#include"BP.h"

typedef IloBoolVarArray TypeX;

class Parameter_CuttingPlane {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Label_TimePath> columnPool;
};

class Solution_CuttingPlane {
public:
	OptimalityStatus status;
	double objective;
	vector<Label_TimePath> routes;

	set<double, timeSortCriterion> timeSet;
	set<tuple<int, int, int>> tripletSet;
	vector<pair<vector<int>, double>> SFCSet;
	int numBranch;
};

void setObjectiveCuttingPlane(const vector<Label_TimePath>& structures, IloModel model, TypeX x);
void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, TypeX x);
Data_Input_VRPTW constructDataVRPTW(const int maxNumVehicles, const vector<Label_TimePath>& selectedStructures);
vector<Label_TimePath> getSolutionCuttingPlaneAlgorithm(const Parameter_CuttingPlane& parameter, const IloCplex& cplex, const TypeX& X);
Solution_CuttingPlane CuttingPlaneAlgorithm(const Parameter_CuttingPlane& parameter);

