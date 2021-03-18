#include"CMTVRPTW.h"


Solution_OPRE_2019_1874 Framework_OPRE_2019_1874::solve(const Parameter_OPRE_2019_1874& parameter) {
	Solution_OPRE_2019_1874 solution;
	solution.status = OptimalityStatus::Infeasible;
	solution.objective = InfinityPos;
	try {
		clock_t last;
		for (gapGuess = parameter.gapInit; solution.status != OptimalityStatus::Optimal; gapGuess += parameter.gapIncre) {
			last = clock();
			cout << "**************************************" << endl;
			Solution_VRPTW_CG result_VRPTW_CG_LB = lbAtCGRootNodeVRPTW(parameter.input_VRPTW);
			LB_1 = result_VRPTW_CG_LB.getCost();
			UB_Guess = min(solution.objective, (1 + gapGuess) * LB_1);
			cout << "Time: " << numToStr(runTime(last)) << '\t' << "Lower Bound 1: " << LB_1 << "\t" << "UB_Guess: " << UB_Guess << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			Map_Label_TimePath resultEnumeration = EnumerationStructure(result_VRPTW_CG_LB.getInput(), UB_Guess - LB_1);
			cout << "Time: " << numToStr(runTime(last)) << '\t' << "The number of structures: " << resultEnumeration.getSize() << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			outputRSFC resultRSFC;
			RSFC(resultRSFC, parameter.input_VRPTW, resultEnumeration);
			LB_2 = resultRSFC.objective;
			cout << "Time: " << numToStr(runTime(last)) << '\t' << "Lower Bound 2: " << LB_2 << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			vector<Label_TimePath> resultReduction = StructureReduction(parameter.input_VRPTW, resultRSFC, UB_Guess - LB_2);
			cout << "Time: " << numToStr(runTime(last)) << '\t' << "The number of structures: " << resultReduction.size() << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			Parameter_VRPTW_BC parameter_BC;
			parameter_BC.input_VRPTW = parameter.input_VRPTW;
			parameter_BC.columnPool = resultReduction;
			parameter_BC.weightLB = parameter_BC.weightDepth = 1;
			parameter_BC.allowPrintLog = true;
			NODE_VRPTW_BC resultBC = BCAlgorithm(parameter_BC, cout);
			cout << "######################################" << endl << endl;
			if (resultBC.solution.feasible) {
				solution.objective = resultBC.solution.objective;
				solution.routes = resultBC.solution.UB_Integer_Solution;
				solution.status = greaterThanReal(resultBC.solution.objective, UB_Guess, PPM) ? OptimalityStatus::Feasible : OptimalityStatus::Optimal;
			}

			//last = clock();
			//cout << endl << "**************************************" << endl;
			//Parameter_CuttingPlane parameter_CuttingPlane;
			//parameter_CuttingPlane.input_VRPTW = parameter.input_VRPTW;
			//parameter_CuttingPlane.columnPool = resultReduction;
			//Solution_CuttingPlane resultCuttingPlane = CuttingPlaneAlgorithm(parameter_CuttingPlane);
			//cout << "######################################" << endl << endl;

			//if (resultCuttingPlane.status == OptimalityStatus::Optimal) {
			//	solution.objective = resultCuttingPlane.objective;
			//	solution.routes = resultCuttingPlane.routes;
			//	solution.status = greaterThanReal(resultCuttingPlane.objective, UB_Guess, PPM) ? OptimalityStatus::Feasible : OptimalityStatus::Optimal;
			//}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Framework_OPRE_2019_1874::solve", exc);
	}
	return solution;
}


Solution_OPRE_2019_1874 run_OPRE_2019_1874(const string& strInput) {
	Solution_OPRE_2019_1874 solution;
	try {
		Parameter_OPRE_2019_1874 parameter;
		parameter.gapInit = 0.05;
		parameter.gapIncre = 0.01;

		readFromFileVRPTW(parameter.input_VRPTW, strInput);
		parameter.input_VRPTW.constrainResource = { true,false,true };
		parameter.input_VRPTW.preprocess();

		Framework_OPRE_2019_1874 frame;
		solution = frame.solve(parameter);
	}
	catch (const exception& exc) {
		printErrorAndExit("run_OPRE_2019_1874", exc);
	}
	return solution;
}


void setObjective(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X) {
	try {
		auto env = model.getEnv();
		IloExpr expr(env);
		for (int i = 0; i < X.getSize(); ++i) {
			for (int j = 0; j < X[i].getSize(); ++j) {
				if (parameter.ExistingArcs[i][j]) {
					expr += parameter.Distance[i][j] * X[i][j];
				}
			}
		}
		model.add(IloMinimize(env, expr));
		expr.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("setObjective_ArcFlow", exc);
	}
}


void setConstraintsX(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X) {
	try {
		const int NUM = X.getSize();
		for (int i = 0; i < NUM; ++i) {
			for (int j = 0; j < NUM; ++j) {
				if (!parameter.ExistingArcs[i][j]) {
					model.add(X[i][j] == IloFalse);
				}
			}
		}

		auto env = model.getEnv();
		IloExpr numVeh(env);
		for (int j = 0; j < NUM; ++j) {
			model.add(X[j][0] == IloFalse);
			model.add(X[NUM - 1][j] == IloFalse);
			if (parameter.ExistingArcs[0][j]) numVeh += X[0][j];
		}
		model.add(numVeh <= parameter.NumVehicles);
		numVeh.end();

		for (int i = 1; i < NUM - 1; ++i) {
			IloExpr outDegree(env), inDegree(env);
			for (int j = 0; j < NUM; ++j) {
				if (parameter.ExistingArcs[i][j]) outDegree += X[i][j];
				if (parameter.ExistingArcs[j][i]) inDegree += X[j][i];
			}
			model.add(outDegree == 1);
			model.add(outDegree == inDegree);
			outDegree.end(), inDegree.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsDomainX_CMTVRPTW_ArcFlow", exc);
	}
}


void setConstraintsTimeWindow(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y) {
	try {
		for (int i = 0; i < X.getSize(); ++i) {
			for (int j = 0; j < X[i].getSize(); ++j) {
				if (parameter.ExistingArcs[i][j]) {
					const double tm = parameter.Time[i][j];
					const double M = parameter.TimeWindow[i].second + tm - parameter.TimeWindow[j].first;
					model.add(Y[i] + tm - Y[j] <= (1 - X[i][j]) * M);
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsTimeWindow", exc);
	}
}


double CMTVRPTW_ArcFlow(const Parameter_CMTVRPTW_ArcFlow& parameter, ostream& output) {
	double result = -1;
	IloEnv env;
	try {
		const int N = parameter.NumVertices, K = parameter.NumDummyDepots;
		// Define the variables.
		IloBoolVarArray2 X(env, N + K);
		for (int i = 0; i < X.getSize(); ++i) {
			X[i] = IloBoolVarArray(env, N + K);
		}

		IloNumArray early(env, N + K), late(env, N + K);
		for (int i = 0; i < N + K; ++i) {
			early[i] = parameter.TimeWindow[i].first;
			late[i] = parameter.TimeWindow[i].second;
		}
		IloNumVarArray Y(env, early, late);

		// Define the model.
		IloModel model(env);
		setObjective(parameter, model, X);
		setConstraintsX(parameter, model, X);
		setConstraintsTimeWindow(parameter, model, X, Y);

	}
	catch (const exception& exc) {
		printErrorAndExit("CMTVRPTW_ArcFlow", exc);
	}
	env.end();
	return result;
}

