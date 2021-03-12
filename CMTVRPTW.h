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

Solution_OPRE_2019_1874 run_OPRE_2019_1874(const string& strInput);

