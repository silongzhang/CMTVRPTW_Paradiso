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
void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x);
void addConstraintsActive(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const set<double, timeSortCriterion>& additionalTimes);
void addConstraintsSR(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const set<tuple<int, int, int>>& additionalTriplets);
vector<int> getPositiveStructures(const IloCplex& cplex, const IloNumVarArray& x);
set<double, timeSortCriterion> detectAdditionalTimes(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures,
	const set<double, timeSortCriterion>& times, const IloCplex& cplex, const IloNumVarArray& x);
set<tuple<int, int, int>> detectAdditionalTriplets(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures,
	const set<tuple<int, int, int>>& triplets, const IloCplex& cplex, const IloNumVarArray& x);

