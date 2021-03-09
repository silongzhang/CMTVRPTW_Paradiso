#include"BC.h"


void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloBoolVarArray x) {
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


void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloBoolVarArray x) {
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


ILOUSERCUTCALLBACK3(TimeUserCut, const Parameter_BC&, parameter, IloBoolVarArray, x, Solution_BC&, solution) {
	try {
		// Get positive structures.
		vector<int> positive;
		for (int i = 0; i < x.getSize(); ++i) {
			if (greaterThanReal(getValue(x[i]), 0, PPM)) {
				positive.push_back(i);
			}
		}

		// Get the set of latest departure times.
		set<double, timeSortCriterion> tails;
		for (const auto s : positive) {
			auto tm = parameter.columnPool[s].getTimeAttribute().getLatestDeparture();
			if (tails.find(tm) == tails.end()) {
				tails.insert(tm);
			}
		}

		// Get the set of times where RSFC constraints are violated.
		set<double, timeSortCriterion> times;
		auto env = getEnv();
		for (const auto tm : tails) {
			IloExpr expr(env);
			for (const auto s : positive) {
				if (parameter.columnPool[s].strongActive(tm)) {
					expr += x[s];
				}
			}
			if (greaterThanReal(getValue(expr), parameter.input_VRPTW.MaxNumVehicles, PPM)) {
				times.insert(tm);
			}
			expr.end();
		}

		// Add RSFC constraints.
		for (const auto t : times) {
			solution.timeSet.insert(t);

			IloExpr expr(env);
			for (int s = 0; s < parameter.columnPool.size(); ++s) {
				if (parameter.columnPool[s].strongActive(t)) {
					expr += x[s];
				}
			}
			add(expr <= parameter.input_VRPTW.MaxNumVehicles);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TimeUserCut", exc);
	}
}


ILOUSERCUTCALLBACK3(TripletUserCut, const Parameter_BC&, parameter, IloBoolVarArray, x, Solution_BC&, solution) {
	try {
		// Get positive structures.
		vector<int> positive;
		for (int i = 0; i < x.getSize(); ++i) {
			if (greaterThanReal(getValue(x[i]), 0, PPM)) {
				positive.push_back(i);
			}
		}

		// Get the set of triplets where SR constraints are violated.
		set<tuple<int, int, int>> existing, triplets;
		for (const auto s : positive) {
			for (const auto& tp : parameter.columnPool[s].getTuples(1, parameter.input_VRPTW.NumVertices)) {
				if (existing.find(tp) == existing.end()) {
					existing.insert(tp);
				}
				else {
					triplets.insert(tp);
				}
			}
		}

		// Add SR constraints.
		auto env = getEnv();
		for (const auto& triplet : triplets) {
			solution.tripletSet.insert(triplet);

			IloExpr expr(env);
			for (int s = 0; s < parameter.columnPool.size(); ++s) {
				if (parameter.columnPool[s].atLeastTwo(triplet)) {
					expr += x[s];
				}
			}
			add(expr <= 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TripletUserCut", exc);
	}
}


ILOLAZYCONSTRAINTCALLBACK3(CoexistLazyConstraint, const Parameter_BC&, parameter, IloBoolVarArray, X, Solution_BC&, solution) {
	try {
		// Check whether X is an integer vector.
		for (int i = 0; i < X.getSize(); ++i) {
			if (!equalToReal(getValue(X[i]), 0, PPM) && !equalToReal(getValue(X[i]), 1, PPM)) throw exception();
		}

		// Get selected structures.
		vector<int> selected;
		for (int i = 0; i < X.getSize(); ++i) {
			if (equalToReal(getValue(X[i]), 1, PPM)) {
				selected.push_back(i);
			}
		}

		// Solve the TOPTW determined by selected structures.

		// Add lazy constraints.
















	}
	catch (const exception& exc) {
		printErrorAndExit("CoexistLazyConstraint", exc);
	}
}














Solution_BC BCAlgorithm(const Parameter_BC& parameter, ostream& output) {
	Solution_BC solution;
	solution.status = OptimalityStatus::Infeasible;
	solution.objective = InfinityPos;

	IloEnv env;
	try {
		string strLog = "Begin running the procedure titled BCAlgorithm.";
		print(parameter.allowPrintLog, output, strLog);
		clock_t start = clock();

		// Define the model.
		IloModel model(env);
		IloBoolVarArray X(env, parameter.columnPool.size());
		setObjective(parameter.columnPool, model, X);
		setConstraintsPartition(parameter.input_VRPTW, parameter.columnPool, model, X);

		// Solve the model.
		IloCplex cplex(model);
		cplex.use(TimeUserCut(env, parameter, X, solution));
		cplex.use(TripletUserCut(env, parameter, X, solution));



		// Add user cuts and lazy constraints.




		strLog = "The procedure titled BCAlgorithm is finished.";
		print(parameter.allowPrintLog, output, strLog);
	}
	catch (const exception& exc) {
		printErrorAndExit("BCAlgorithm", exc);
	}
	env.end();

	return solution;
}

