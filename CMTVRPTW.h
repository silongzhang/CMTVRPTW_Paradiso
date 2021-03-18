#pragma once
#include"CuttingPlane.h"
#include"BC.h"

class Parameter_OPRE_2019_1874 {
public:
	Data_Input_VRPTW input_VRPTW;
	double gapInit;
	double gapIncre;
};

class Solution_OPRE_2019_1874 {
public:
	OptimalityStatus status;
	double objective;
	vector<Label_TimePath> routes;
};

class Framework_OPRE_2019_1874 {
public:
	double gapGuess;
	double LB_1;
	double UB_Guess;
	double LB_2;
	vector<Label_TimePath> columnPool;

	Solution_OPRE_2019_1874 solve(const Parameter_OPRE_2019_1874& parameter);
};

class Parameter_CMTVRPTW_ArcFlow {
public:
	int NumVertices;									// 0, 1, 2, ..., N - 1
	int NumDummyDepots;									// N, N + 1, ..., N + K - 1
	int NumVehicles;
	vector<vector<QuantityType>> Quantity;
	vector<vector<DistanceType>> Distance;
	vector<vector<TimeType>> Time;
	vector<pair<TimeType, TimeType>> TimeWindow;
	vector<vector<bool>> ExistingArcs;
};

Solution_OPRE_2019_1874 run_OPRE_2019_1874(const string& strInput);
void setObjective(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X);
void setConstraintsX(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X);
void setConstraintsTimeWindow(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y);

