#include"CG_VRPTW.h"


unordered_map<int, int> getCount(const vector<int> &vec) {
	unordered_map<int, int> unMp;
	for (const auto &i : vec) {
		if (unMp.find(i) == unMp.end()) unMp[i] = 1;
		else ++unMp[i];
	}
	return unMp;
}


void CG_VRPTW::addColumn(const Route_VRPTW &rhs, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X) {
	try {
		// Add the column.
		columns.push_back(rhs);

		// The coefficient in objective function.
		IloNumColumn col = objectiveRMP(rhs.getRealCost());

		// The coefficients in constraints for unique visit.
		unordered_map<int, int> unMp = getCount(rhs.getPath());
		for (const auto &elem : unMp) {
			if (elem.first != 0) {
				col += constraintRMP[elem.first](elem.second);
			}
		}

		// Add the new variable.
		X.add(IloNumVar(col, 0, IloInfinity, ILOFLOAT));

		col.end();
	}
	catch (const exception &exc) {
		printErrorAndExit("CG_VRPTW::addColumn", exc);
	}
}


void CG_VRPTW::InitiateRMP(const vector<Route_VRPTW> &initialRoutes, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X) {
	try {
		clearColumns();
		for (const auto &rhs : initialRoutes) {
			addColumn(rhs, objectiveRMP, constraintRMP, X);
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("CG_VRPTW::InitiateRMP", exc);
	}
}


Solution_VRPTW CG_VRPTW::columnGeneration(const Data_Input_ESPPRC &inputESPPRC, const vector<Route_VRPTW> &initialRoutes, ostream &output) {
	IloEnv env;
	Solution_VRPTW sol;
	try {
		// Define the model of restricted master problem.
		IloModel modelRMP(env);

		// Define the objective function of restricted master problem.
		IloObjective objectiveRMP = IloAdd(modelRMP, IloMinimize(env));

		// Define bounds of constraints of restricted master problem.
		IloNumArray rightSide(env, inputESPPRC.NumVertices);
		for (int i = 1; i < inputESPPRC.NumVertices; ++i) rightSide[i] = 1;
		IloRangeArray constraintRMP = IloAdd(modelRMP, IloRangeArray(env, rightSide, rightSide));

		// Define the variables of restricted master problem.
		IloNumVarArray X(env);

		// Initiate the model of restricted master problem.
		InitiateRMP(initialRoutes, objectiveRMP, constraintRMP, X);

	}
	catch (const exception &exc) {
		printErrorAndExit("CG_VRPTW::columnGeneration", exc);
	}
	env.end();
	return sol;
}

