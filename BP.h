#pragma once

#include"TOPTW.h"

class Node_BP {
private:
	int id;
	int depth;

	TOPTW_CG model;
	Parameter_TOPTW_CG parameter;
	Solution_TOPTW_CG solution;
};
