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


void NODE_VRPTW_BC::getIntegerSolution(const Parameter_VRPTW_BC& parameter, const IloCplex& cplex, const IloNumVarArray& X) {
	try {
		solution.UB_Integer_Solution.clear();
		for (int i = 0; i < X.getSize(); ++i) {
			if (equalToReal(cplex.getValue(X[i]), 1, PPM)) {
				solution.UB_Integer_Solution.push_back(parameter.columnPool[i]);
			}
			else if (!equalToReal(cplex.getValue(X[i]), 0, PPM)) throw exception();
		}

		solution.UB_Integer_Value = 0;
		for (const auto& elem : solution.UB_Integer_Solution) {
			solution.UB_Integer_Value += elem.getRealCost();
		}
		if (!equalToReal(solution.UB_Integer_Value, cplex.getObjValue(), PPM)) throw exception();
	}
	catch (const exception& exc) {
		printErrorAndExit("NODE_VRPTW_BC::getIntegerSolution", exc);
	}
}


void NODE_VRPTW_BC::getVisitArcs(const Parameter_VRPTW_BC& parameter, const IloCplex& cplex, const IloNumVarArray& X) {
	try {
		vector<int> basicVariables;
		for (int i = 0; i < X.getSize(); ++i) {
			if (greaterThanReal(cplex.getValue(X[i]), 0, PPM)) {
				basicVariables.push_back(i);
			}
		}

		const int N = parameter.input_VRPTW.NumVertices;
		solution.visitArcs = vector<vector<double>>(N, vector<double>(N, 0));
		for (const auto& i : basicVariables) {
			const auto path = parameter.columnPool[i].getPath();
			auto pre = path.begin();
			for (auto suc = pre + 1; suc != path.end(); ++pre, ++suc) {
				solution.visitArcs[*pre][*suc] += cplex.getValue(X[i]);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("NODE_VRPTW_BC::getVisitArcs", exc);
	}
}


void NODE_VRPTW_BC::solve(const Parameter_VRPTW_BC& parameter, ostream& output) {
	IloEnv env;
	try {
		solution.feasible = solution.integer = false;
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
		for (; true;) {
			if (!cplex.solve()) {
				solution.feasible = false;
				return;
			}

			auto additionalTimes = detectAdditionalTimes(parameter.input_VRPTW, parameter.columnPool, input.timeSet, cplex, X);
			for (const auto& elem : additionalTimes) input.timeSet.insert(elem);
			auto additionalTriplets = detectAdditionalTriplets(parameter.input_VRPTW, parameter.columnPool, input.tripletSet, cplex, X);
			for (const auto& elem : additionalTriplets) input.tripletSet.insert(elem);
			if (additionalTimes.empty() && additionalTriplets.empty()) {
				break;
			}
			else {
				addConstraintsTime_VRPTW_BC(parameter, model, X, additionalTimes);
				addConstraintsSR_VRPTW_BC(parameter, model, X, additionalTriplets);
			}
		}

		// Get the solution.
		if (cplex.getCplexStatus() != IloCplex::Optimal) throw exception();
		solution.feasible = true;
		solution.objective = cplex.getObjValue();
		solution.integer = isBool(cplex, X);
		if (solution.integer) getIntegerSolution(parameter, cplex, X);
		getVisitArcs(parameter, cplex, X);
	}
	catch (const exception& exc) {
		printErrorAndExit("NODE_VRPTW_BC::solve", exc);
	}
	env.end();
}

