#pragma once
#include"BP.h"

enum class Status { Infeasible, Feasible, Optimal };

class Solution_OPRE_2019_1874 {
public:
	double objective;
	vector<Label_TimePath> routes;
};

class Framework_OPRE_2019_1874 {
public:
	Status status;
	double gapGuess;
	double gapIncre;
	double LB_1;
	double UB_Guess;
	double LB_2;

	vector<Label_TimePath> structures;
	Solution_OPRE_2019_1874 solution;
};

