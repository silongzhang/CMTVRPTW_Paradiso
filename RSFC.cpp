#include"RSFC.h"


vector<Label_TimePath> linearize(const Map_Label_TimePath& structures) {
	vector<Label_TimePath> result;
	try {
		for (const auto& elem : structures.getMpVisitedLabels()) {
			for (const auto& structure : elem.second) {
				result.push_back(structure);
			}
		}
		if (result.size() != structures.getSize()) throw exception();
	}
	catch (const exception& exc) {
		printErrorAndExit("linearize", exc);
	}
	return result;
}


void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x) {
	try {
		auto env = model.getEnv();
		IloExpr expr(env);
		for (int i = 0; i < structures.size(); ++i) {
			expr += structures[i].getRealCost() * x[i];
		}
		model.add(IloMinimize(env, expr));
		expr.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("setObjective", exc);
	}
}


void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x) {
	try {
		auto env = model.getEnv();
		for (int i = 1; i < input.NumVertices; ++i) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].hasVisited(i)) {
					expr += x[s];
				}
			}
			model.add(expr == 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsPartition", exc);
	}
}


void addConstraintsActive(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x, 
	const set<double, timeSortCriterion>& additionalTimes) {
	try {
		auto env = model.getEnv();
		for (const auto t : additionalTimes) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].strongActive(t)) {
					expr += x[s];
				}
			}
			model.add(expr <= input.MaxNumVehicles);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsActive", exc);
	}
}


void addConstraintsSR(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const set<tuple<int, int, int>>& additionalTriplets) {
	try {
		auto env = model.getEnv();
		for (const auto& triplet : additionalTriplets) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].atLeastTwo(triplet)) {
					expr += x[s];
				}
			}
			model.add(expr <= 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("addConstraintsSR", exc);
	}
}


vector<int> getPositiveStructures(const IloCplex& cplex, const IloNumVarArray& x) {
	vector<int> result;
	try {
		for (int i = 0; i < x.getSize(); ++i) {
			if (greaterThanReal(cplex.getValue(x[i]), 0, PPM)) {
				result.push_back(i);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("getPositiveStructures", exc);
	}
	return result;
}


set<double, timeSortCriterion> detectAdditionalTimes(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, 
	const set<double, timeSortCriterion>& times, const IloCplex& cplex, const IloNumVarArray& x) {
	set<double, timeSortCriterion> result;
	try {
		auto positive = getPositiveStructures(cplex, x);
		set<double, timeSortCriterion> potential;

		for (const auto s : positive) {
			auto tm = structures[s].getTimeAttribute().getLatestDeparture();
			if (times.find(tm) == times.end() && potential.find(tm) == potential.end()) {
				potential.insert(tm);
			}
		}

		auto env = cplex.getEnv();
		for (const auto tm : potential) {
			IloExpr expr(env);
			for (const auto s : positive) {
				if (structures[s].strongActive(tm)) {
					expr += x[s];
				}
			}
			if (greaterThanReal(cplex.getValue(expr), input.MaxNumVehicles, PPM)) {
				result.insert(tm);
			}
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("detectAdditionalTimes", exc);
	}
	return result;
}


set<tuple<int, int, int>> detectAdditionalTriplets(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures,
	const set<tuple<int, int, int>>& triplets, const IloCplex& cplex, const IloNumVarArray& x) {
	set<tuple<int, int, int>> result;
	try {
		auto positive = getPositiveStructures(cplex, x);
		set<tuple<int, int, int>> existing;

		for (const auto s : positive) {
			for (const auto& tp : structures[s].getTuples(1, input.NumVertices)) {
				if (triplets.find(tp) == triplets.end()) {
					if (existing.find(tp) == existing.end()) {
						existing.insert(tp);
					}
					else {
						result.insert(tp);
					}
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("detectAdditionalTriplets", exc);
	}
	return result;
}


void RSFC(outputRSFC& output, const Data_Input_VRPTW& input, const Map_Label_TimePath& stt) {
	IloEnv env;
	try {
		set<double, timeSortCriterion> times;
		set<tuple<int, int, int>> triplets;

		vector<Label_TimePath> structures = linearize(stt);
		const long long NumStructures = structures.size();

		// Define the variables.
		IloNumVarArray x(env, NumStructures, 0, IloInfinity);

		// Define the model.
		IloModel model(env);
		setObjective(structures, model, x);
		setConstraintsPartition(input, structures, model, x);

		// Solve the model.
		IloCplex cplex(model);
		while (true) {
			cplex.solve();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("RSFC", exc);
	}
	env.end();
}

