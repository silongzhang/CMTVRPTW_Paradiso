#pragma once
#include"BP.h"

enum class Status { Infeasible, Feasible, Optimal };

class Parameter_OPRE_2019_1874 {
public:
	Data_Input_VRPTW input_VRPTW;
	double gapInit;
	double gapIncre;
};

class Solution_OPRE_2019_1874 {
public:
	Status status;
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

