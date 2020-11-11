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


double solveModel(IloCplex &solver) {
	solver.solve();
	solver.out() << "solution status = " << solver.getStatus() << endl;
	solver.out() << "objective = " << solver.getObjValue() << endl;
	return solver.getObjValue();
}


Solution_VRPTW CG_VRPTW::getAnIntegralSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X) {
	Solution_VRPTW sol;
	sol.reset();
	try {
		auto env = modelRMP.getEnv();
		modelRMP.add(IloConversion(env, X, ILOBOOL));
		solveModel(solverRMP);
		if (columns.size() != X.getSize()) throw exception();
		auto pos = columns.begin();
		for (int i = 0; i < X.getSize(); ++i, ++pos) {
			if (equalToReal(solverRMP.getValue(X[i]), 1, PPM)) sol.addRoute(solverRMP.getValue(X[i]), *pos);
			else if (!equalToReal(solverRMP.getValue(X[i]), 0, PPM)) throw exception();
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("CG_VRPTW::getAnIntegralSolution", exc);
	}
	return sol;
}


Solution_VRPTW CG_VRPTW::columnGeneration(const Data_Input_ESPPRC &inputESPPRC, const vector<Route_VRPTW> &initialRoutes, 
	const Parameter_CG_VRPTW &prm, ostream &output) {
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

		// Define the solver of restricted master problem.
		IloCplex solverRMP(modelRMP);
		solverRMP.setOut(env.getNullStream());

		string strLog;
		int iter = 0;
		do {
			// Solve the RMP.
			strLog = "Solve the master problem for the " + numToStr(++iter) + "th time." + '\n';
			print(prm.allowPrintLog, output, strLog);
			solveModel(solverRMP);

			// Get dual values.
			IloNumArray dualValue(env);
			solverRMP.getDuals(dualValue, constraintRMP);
			dualValue[0] = 0;

			// Set reduced cost.
			Data_Input_ESPPRC input(inputESPPRC);
			for (int i = 0; i < inputESPPRC.NumVertices; ++i) {
				for (int j = 0; j < inputESPPRC.NumVertices; ++j) {
					input.ReducedCost[i][j] = input.RealCost[i][j] - (dualValue[i] + dualValue[j]) / 2;
				}
			}

			// Solve the subproblem.
			Data_Auxiliary_ESPPRC auxiliary;
			auto resultSP = DPAlgorithmESPPRC(input, auxiliary, output);

			// Stopping criterion for iteration.
			if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost(), 0, PPM)) break;

			// Add new columns.
			for (const auto &elem : resultSP) {
				addColumn(elem, objectiveRMP, constraintRMP, X);
			}
		} while (true);

		// Get an integral solution.
		sol = getAnIntegralSolution(modelRMP, solverRMP, X);
	}
	catch (const exception &exc) {
		printErrorAndExit("CG_VRPTW::columnGeneration", exc);
	}
	env.end();
	return sol;
}

