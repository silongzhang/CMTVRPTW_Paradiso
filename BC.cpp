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


ILOUSERCUTCALLBACK2(TimeUserCut, Parameter_BC, parameter, IloBoolVarArray, x) {
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


ILOUSERCUTCALLBACK2(TripletUserCut, Parameter_BC, parameter, IloBoolVarArray, x) {
	try {

	}
	catch (const exception& exc) {
		printErrorAndExit("TripletUserCut", exc);
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

