#pragma once

#include"Enumeration_TimePath.h"

class timeSortCriterion {
public:
	bool operator()(const double lhs, const double rhs) const {
		return lessThanReal(lhs, rhs, PPM);
	}
};

class outputRSFC {
public:
	double objective;
};

vector<Label_TimePath> linearize(const Map_Label_TimePath& structures);
void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x);
void setConstraintsPartition(const Data_Input_ESPPRC& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x);

