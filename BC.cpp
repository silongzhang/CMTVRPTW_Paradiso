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

