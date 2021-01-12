#pragma once

#include"Enumeration_TimePath.h"

class timeSortCriterion {
public:
	bool operator()(const double lhs, const double rhs) const {
		return lessThanReal(lhs, rhs, PPM);
	}
};

vector<Label_TimePath> linearize(const Map_Label_TimePath& structures);
vector<double> linearize(const set<double, timeSortCriterion>& tms);
vector<tuple<int, int, int>> linearize(const set<tuple<int, int, int>>& tps);
void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x);
void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	IloRangeArray rangePartition);
void addConstraintsActive(outputRSFC& output, const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const vector<double>& additionalTimes, IloRangeArray rangeActive);
void addConstraintsSR(outputRSFC& output, const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const vector<tuple<int, int, int>>& additionalTriplets, IloRangeArray rangeSR);
vector<int> getPositiveStructures(const IloCplex& cplex, const IloNumVarArray& x);
set<double, timeSortCriterion> detectAdditionalTimes(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures,
	const set<double, timeSortCriterion>& times, const IloCplex& cplex, const IloNumVarArray& x);
set<tuple<int, int, int>> detectAdditionalTriplets(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures,
	const set<tuple<int, int, int>>& triplets, const IloCplex& cplex, const IloNumVarArray& x);
void RSFC(outputRSFC& output, const Data_Input_VRPTW& input, const Map_Label_TimePath& stt);
vector<Label_TimePath> StructureReduction(const Data_Input_VRPTW& input, const outputRSFC& rsfc, const double reducedCostGap);
void testUntilStructureReduction();

