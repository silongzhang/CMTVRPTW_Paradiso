#pragma once

#include"ESPPRC.h"
#include"VRPTW.h"

class Parameter_TOPTW_CG {
public:
	Data_Input_VRPTW input_VRPTW;
	bool allowPrintLog;
};

class Solution_TOPTW_CG {
public:
	double profit;
};

class TOPTW_CG {
private:
	list<Route_VRPTW> columns;
public:
	void columnGeneration(const Parameter_TOPTW_CG& parameter, Solution_TOPTW_CG& solution, ostream& output);
};


