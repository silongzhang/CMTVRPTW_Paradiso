#include"BC.h"


bool operator<(const NODE_VRPTW_BC& lhs, const NODE_VRPTW_BC& rhs) {
	return lhs.priority < rhs.priority;
}


void setConstraintsPartition_VRPTW_BC(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray X) {
	try {
		auto env = model.getEnv();
		for (int i = 1; i < input.NumVertices; ++i) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].hasVisited(i)) {
					expr += X[s];
				}
			}
			model.add(expr == 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsPartition_VRPTW_BC", exc);
	}
}


void addConstraintsTime_VRPTW_BC(const Parameter_VRPTW_BC& parameter, IloModel model, IloNumVarArray X, 
	const set<double, timeSortCriterion>& additionalTimes) {
	try {
		auto env = model.getEnv();
		for (const auto t : additionalTimes) {
			IloExpr expr(env);
			for (int s = 0; s < parameter.columnPool.size(); ++s) {
				if (parameter.columnPool[s].strongActive(t)) {
					expr += X[s];
				}
			}
			model.add(expr <= parameter.input_VRPTW.MaxNumVehicles);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("addConstraintsTime_VRPTW_BC", exc);
	}
}


void addConstraintsSR_VRPTW_BC(const Parameter_VRPTW_BC& parameter, IloModel model, IloNumVarArray X,
	const set<tuple<int, int, int>>& additionalTriplets) {
	try {
		auto env = model.getEnv();
		for (const auto& triplet : additionalTriplets) {
			IloExpr expr(env);
			for (int s = 0; s < parameter.columnPool.size(); ++s) {
				if (parameter.columnPool[s].atLeastTwo(triplet)) {
					expr += X[s];
				}
			}
			model.add(expr <= 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("addConstraintsSR_VRPTW_BC", exc);
	}
}


void addConstraintsSFC_VRPTW_BC(IloModel model, IloNumVarArray X, const vector<pair<vector<int>, double>>& SFCSet) {
	try {
		auto env = model.getEnv();
		for (const auto& elem : SFCSet) {
			IloExpr expr(env);
			for (const auto i : elem.first) {
				expr += X[i];
			}
			model.add(expr <= elem.second);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("addConstraintsSFC_VRPTW_BC", exc);
	}
}


void addConstraintsBranchOnArcs_VRPTW_BC(IloModel model, IloNumVarArray X, const vector<pair<vector<int>, bool>>& branchOnArcs) {
	try {
		auto env = model.getEnv();
		for (const auto& elem : branchOnArcs) {
			IloExpr expr(env);
			for (const auto i : elem.first) {
				expr += X[i];
			}
			elem.second ? model.add(expr == 1) : model.add(expr == 0);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("addConstraintsBranchOnArcs_VRPTW_BC", exc);
	}
}


void NODE_VRPTW_BC::solve(const Parameter_VRPTW_BC& parameter, ostream& output) {
	IloEnv env;
	try {
		solution.explored = solution.feasible = solution.integer = false;
		solution.objective = InfinityPos;

		// Define the model.
		IloModel model(env);
		IloNumVarArray X(env, parameter.columnPool.size(), 0, 1);
		setObjective(parameter.columnPool, model, X);
		setConstraintsPartition_VRPTW_BC(parameter.input_VRPTW, parameter.columnPool, model, X);
		addConstraintsTime_VRPTW_BC(parameter, model, X, input.timeSet);
		addConstraintsSR_VRPTW_BC(parameter, model, X, input.tripletSet);
		addConstraintsSFC_VRPTW_BC(model, X, input.SFCSet);
		addConstraintsBranchOnArcs_VRPTW_BC(model, X, input.branchOnArcs);

		// Solve the model.
		IloCplex cplex(model);





	}
	catch (const exception& exc) {
		printErrorAndExit("NODE_VRPTW_BC::solve", exc);
	}
	env.end();
}

