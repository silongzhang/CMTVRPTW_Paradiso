#include"VRPTW_CG.h"


void Solution_VRPTW_CG::print(ostream &output) const {
	try {
		output << "Total cost: " << cost << '\t' << "Number of routes: " << routes.size() << endl;
		for (const auto &elem : routes) {
			output << "Weight: " << elem.first << '\t' << "Cost: " << elem.second.getRealCost() << '\t' << "Sequence: ";
			for (const auto &vertex : elem.second.getPath()) output << vertex << '\t';
			output << endl;
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("Solution_VRPTW_CG::print", exc);
	}
}


unordered_map<int, int> getCount(const vector<int> &vec) {
	unordered_map<int, int> unMp;
	for (const auto &i : vec) {
		if (unMp.find(i) == unMp.end()) unMp[i] = 1;
		else ++unMp[i];
	}
	return unMp;
}


void VRPTW_CG::addColumn(const Route_VRPTW &rhs, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X) {
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
		printErrorAndExit("VRPTW_CG::addColumn", exc);
	}
}


void VRPTW_CG::InitiateRMP(const vector<Route_VRPTW> &initialRoutes, IloObjective &objectiveRMP, IloRangeArray &constraintRMP, IloNumVarArray &X) {
	try {
		clearColumns();
		for (const auto &rhs : initialRoutes) {
			addColumn(rhs, objectiveRMP, constraintRMP, X);
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("VRPTW_CG::InitiateRMP", exc);
	}
}


double solveModel(IloCplex &solver) {
	try {
		solver.solve();
		solver.out() << "solution status = " << solver.getStatus() << endl;
		solver.out() << "objective = " << solver.getObjValue() << endl;
	}
	catch (const exception& exc) {
		printErrorAndExit("solveModel", exc);
	}
	return solver.getObjValue();
}


// Get a solution (may be fractional).
Solution_VRPTW_CG VRPTW_CG::getSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X) {
	Solution_VRPTW_CG sol;
	sol.reset();
	try {
		auto env = modelRMP.getEnv();
		solveModel(solverRMP);
		if (columns.size() != X.getSize()) throw exception();
		auto pos = columns.begin();
		for (int i = 0; i < X.getSize(); ++i, ++pos) {
			if (greaterThanReal(solverRMP.getValue(X[i]), 0, PPM)) sol.addRoute(solverRMP.getValue(X[i]), *pos);
			else if (lessThanReal(solverRMP.getValue(X[i]), 0, PPM)) throw exception();
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("VRPTW_CG::getSolution", exc);
	}
	return sol;
}


// Get an integer solution.
Solution_VRPTW_CG VRPTW_CG::getAnIntegralSolution(IloModel &modelRMP, IloCplex &solverRMP, IloNumVarArray &X) {
	Solution_VRPTW_CG sol;
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
		printErrorAndExit("VRPTW_CG::getAnIntegralSolution", exc);
	}
	return sol;
}


Solution_VRPTW_CG VRPTW_CG::columnGeneration(const Data_Input_ESPPRC &inputESPPRC, const vector<Route_VRPTW> &initialRoutes, 
	const Parameter_VRPTW_CG &prm, ostream &output) {
	IloEnv env;
	Solution_VRPTW_CG sol;
	Data_Input_ESPPRC input;
	try {
		string strLog = "Begin running the procedure titled VRPTW_CG::columnGeneration.";
		print(prm.allowPrintLog, output, strLog);

		// Define the model of restricted master problem.
		IloModel modelRMP(env);

		// Define the objective function of restricted master problem.
		IloObjective objectiveRMP = IloAdd(modelRMP, IloMinimize(env));

		// Define bounds of constraints of restricted master problem.
		IloNumArray leftSide(env, inputESPPRC.NumVertices);
		for (int i = 1; i < inputESPPRC.NumVertices; ++i) leftSide[i] = 1;
		IloRangeArray constraintRMP = IloAdd(modelRMP, IloRangeArray(env, leftSide, IloInfinity));

		// Define the variables of restricted master problem.
		IloNumVarArray X(env);

		// Initiate the model of restricted master problem.
		InitiateRMP(initialRoutes, objectiveRMP, constraintRMP, X);

		// Define the solver of restricted master problem.
		IloCplex solverRMP(modelRMP);
//		solverRMP.setOut(env.getNullStream());

		int iter = 0;
		do {
			// Solve the RMP.
			strLog = "\nSolve the master problem for the " + numToStr(++iter) + "th time.";
			print(prm.allowPrintLog, output, strLog);
			solveModel(solverRMP);

			// Get dual values.
			IloNumArray dualValue(env);
			solverRMP.getDuals(dualValue, constraintRMP);
			dualValue[0] = 0;

			// Set reduced cost.
			input = inputESPPRC;
			for (int i = 0; i < inputESPPRC.NumVertices; ++i) {
				for (int j = 0; j < inputESPPRC.NumVertices; ++j) {
					input.ReducedCost[i][j] = input.RealCost[i][j] - (dualValue[i] + dualValue[j]) / 2;
				}
			}

			// Solve the subproblem.
			Data_Auxiliary_ESPPRC auxiliary;
			input.mustOptimal = false;
			input.minRunTime = 0;
			auto resultSP = DPAlgorithmESPPRC(input, auxiliary, output);
			/*
			vector<double> mrt = { 1,5 };
			for (auto posMRT = mrt.begin(); resultSP.empty() && posMRT != mrt.end(); ++posMRT) {
				input.minRunTime = *posMRT;
				resultSP = DPAlgorithmESPPRC(input, auxiliary, output);
			}
			if (resultSP.empty()) {
				input.mustOptimal = true;
				resultSP = DPAlgorithmESPPRC(input, auxiliary, output);
			}
			*/

			// Stopping criterion for iteration.
			if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost(), -PPM, 0)) {
				input.mustOptimal = true;
				resultSP = DPAlgorithmESPPRC(input, auxiliary, output);
				if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost(), -PPM, 0)) break;
			}

			// Add new columns.
			for (const auto &elem : resultSP) {
				addColumn(elem, objectiveRMP, constraintRMP, X);
			}
			strLog = numToStr(resultSP.size()) + " routes are added." + '\t' +
				"The minimum reduced cost of added routes: " + numToStr(resultSP.begin()->getReducedCost());
			print(prm.allowPrintLog, output, strLog);
		} while (true);

		// Get a solution.
		sol = (prm.canBeFractional ? getSolution(modelRMP, solverRMP, X) : getAnIntegralSolution(modelRMP, solverRMP, X));

		strLog = "\nThe procedure titled VRPTW_CG::columnGeneration is finished.";
		print(prm.allowPrintLog, output, strLog);
	}
	catch (const exception &exc) {
		printErrorAndExit("VRPTW_CG::columnGeneration", exc);
	}
	env.end();
	sol.setInput(input);
	return sol;
}

