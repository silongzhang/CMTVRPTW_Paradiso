#pragma once
#include"CuttingPlane.h"
#include"BC.h"

class Parameter_OPRE_2019_1874 {
public:
	Data_Input_VRPTW input_VRPTW;
	double gapInit;
	double gapIncre;
	bool BCRatherThanCuttingPlane;				// true: use BC in the last step; false: use cutting plane method.
	bool ArcFlowRatherThanBP;					// true: solve TOPTW by the arc-flow model; false: by BP.
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
	int V;									// The number of available vehicles.
	double Capacity;
	int N;									// The set of customers is {1, 2, ..., N - 1}, 0 and N are the depot and dummy depot respectively.
	int K;									// {N + 1, N + 2, ..., N + K} is the set of dummy depots, and each of which can be visited at most once.
	vector<vector<QuantityType>> Quantity;
	vector<QuantityType> QuantityNode;
	vector<vector<DistanceType>> Distance;
	vector<vector<TimeType>> Time;
	vector<pair<TimeType, TimeType>> TimeWindow;
	vector<vector<bool>> ExistingArcs;

	double timeLimit;

	bool isDepot(const int i) const { return i == 0 || (N <= i && i <= N + K); }
	vector<int> depotIndex(const vector<int>& route) const;
	vector<vector<int>> getPaths(const vector<int>& route) const;
	int realIndex(const int i) const { return i < N ? i : 0; }
};

double maxNumCoexist(const bool ArcFlowRatherThanBP, const int maxNumVehicles, const vector<Label_TimePath>& selectedStructures);

Solution_OPRE_2019_1874 run_OPRE_2019_1874(const string& strInput);
void setObjective(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X);
void setConstraintsX(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X);
void setConstraintsTimeWindow(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y);
void setConstraintsCapacity(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Z);
void reduceSymmetry(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y);
tuple<double, double, double> CMTVRPTW_ArcFlow(const Parameter_CMTVRPTW_ArcFlow& parameter, ostream& output);
tuple<double, double, double> CMTVRPTW_ArcFlow(const string& strInput, const int numDummyDepots, ostream& output);
void Test_CMTVRPTW_ArcFlow(const string& outFile);

