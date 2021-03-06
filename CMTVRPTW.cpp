#include"CMTVRPTW.h"


double maxNumCoexist(const bool ArcFlowRatherThanBP, const int maxNumVehicles, const vector<Label_TimePath>& selectedStructures) {
	double numCoexist = -1;
	try {
		Data_Input_VRPTW input_TOPTW = constructDataVRPTW(maxNumVehicles, selectedStructures);
		if (ArcFlowRatherThanBP) {
			Parameter_TOPTW_ArcFlow parameter_ArcFlow;
			parameter_ArcFlow.input_VRPTW = input_TOPTW;
			parameter_ArcFlow.allowPrintLog = false;
			numCoexist = TOPTW_ArcFlow(parameter_ArcFlow, cout);
		}
		else {
			Parameter_BP parameter_BP;
			parameter_BP.weightLB = parameter_BP.weightDepth = 1;
			parameter_BP.allowPrintLog = false;
			BBNODE solTOPTW = BPAlgorithm(input_TOPTW, parameter_BP, cout);
			if (!solTOPTW.solution.feasible || !solTOPTW.solution.integer) throw exception();
			numCoexist = -solTOPTW.solution.objective;
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("maxNumCoexist", exc);
	}
	return numCoexist;
}


Solution_CMTVRPTW_SP Framework_CMTVRPTW_SP::solve(const Parameter_CMTVRPTW_SP& parameter) {
	Solution_CMTVRPTW_SP solution;
	solution.status = OptimalityStatus::Infeasible;
	solution.objective = InfinityPos;
	try {
		clock_t last;
		for (gapGuess = parameter.gapInit; solution.status != OptimalityStatus::Optimal; gapGuess += parameter.gapIncre) {
			last = clock();
			cout << "**************************************" << endl;
			Solution_VRPTW_CG result_VRPTW_CG_LB = lbAtCGRootNodeVRPTW(parameter.input_VRPTW);
			solution.LB_1 = result_VRPTW_CG_LB.getCost();
			//UB_Guess = min(solution.objective, (1 + gapGuess) * solution.LB_1);
			UB_Guess = solution.status == OptimalityStatus::Feasible ? solution.objective : (1 + gapGuess) * solution.LB_1;
			solution.time_LB_1 = runTime(last);
			cout << "Time: " << numToStr(solution.time_LB_1) << '\t' << "Lower Bound 1: " << solution.LB_1 << "\t" << "UB_Guess: " << UB_Guess << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			Map_Label_TimePath resultEnumeration = EnumerationStructure(result_VRPTW_CG_LB.getInput(), UB_Guess - solution.LB_1);
			solution.size_1 = resultEnumeration.getSize();
			solution.time_enumeration = runTime(last);
			cout << "Time: " << numToStr(solution.time_enumeration) << '\t' << "The number of structures: " << solution.size_1 << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			outputRSFC resultRSFC;
			RSFC(resultRSFC, parameter.input_VRPTW, resultEnumeration);
			solution.LB_2 = resultRSFC.objective;
			solution.time_LB_2 = runTime(last);
			cout << "Time: " << numToStr(solution.time_LB_2) << '\t' << "Lower Bound 2: " << solution.LB_2 << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			vector<Label_TimePath> columnPool = StructureReduction(parameter.input_VRPTW, resultRSFC, UB_Guess - solution.LB_2);
			solution.size_2 = columnPool.size();
			cout << "Time: " << numToStr(runTime(last)) << '\t' << "The number of structures: " << solution.size_2 << endl;

			last = clock();
			cout << endl << "**************************************" << endl;
			if (parameter.BCRatherThanCuttingPlane) {
				Parameter_VRPTW_BC parameter_BC;
				parameter_BC.input_VRPTW = parameter.input_VRPTW;
				parameter_BC.columnPool = columnPool;
				parameter_BC.weightLB = parameter_BC.weightDepth = 1;
				parameter_BC.ArcFlowRatherThanBP = parameter.ArcFlowRatherThanBP;
				parameter_BC.allowPrintLog = true;
				NODE_VRPTW_BC resultBC = BCAlgorithm(parameter_BC, cout);
				cout << "######################################" << endl << endl;
				if (resultBC.solution.feasible) {
					solution.objective = resultBC.solution.objective;
					solution.routes = resultBC.solution.UB_Integer_Solution;
					solution.status = greaterThanReal(resultBC.solution.objective, UB_Guess, PPM) ?
						OptimalityStatus::Feasible : OptimalityStatus::Optimal;
				}
			}
			else {
				Parameter_CuttingPlane parameter_CuttingPlane;
				parameter_CuttingPlane.input_VRPTW = parameter.input_VRPTW;
				parameter_CuttingPlane.columnPool = columnPool;
				parameter_CuttingPlane.ArcFlowRatherThanBP = parameter.ArcFlowRatherThanBP;
				Solution_CuttingPlane resultCuttingPlane = CuttingPlaneAlgorithm(parameter_CuttingPlane);
				cout << "######################################" << endl << endl;
				if (resultCuttingPlane.status == OptimalityStatus::Optimal) {
					solution.objective = resultCuttingPlane.objective;
					solution.routes = resultCuttingPlane.routes;
					solution.status = greaterThanReal(resultCuttingPlane.objective, UB_Guess, PPM) ?
						OptimalityStatus::Feasible : OptimalityStatus::Optimal;
				}
			}
			solution.time_BC = runTime(last);
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Framework_OPRE_2019_1874::solve", exc);
	}
	return solution;
}


Solution_CMTVRPTW_SP CMTVRPTW_SP(const string& strInput) {
	Solution_CMTVRPTW_SP solution;
	try {
		Parameter_CMTVRPTW_SP parameter;
		parameter.gapInit = 0.05;
		parameter.gapIncre = 0.01;

		parameter.BCRatherThanCuttingPlane = true;
		parameter.ArcFlowRatherThanBP = true;

		readFromFileVRPTW(parameter.input_VRPTW, strInput);
		parameter.input_VRPTW.constrainResource = { true,false,true };
		parameter.input_VRPTW.preprocess();

		Framework_CMTVRPTW_SP frame;
		solution = frame.solve(parameter);
	}
	catch (const exception& exc) {
		printErrorAndExit("run_OPRE_2019_1874", exc);
	}
	return solution;
}


vector<int> Parameter_CMTVRPTW_ArcFlow::depotIndex(const vector<int>& route) const {
	vector<int> result;
	try {
		if (!isDepot(route.front()) || !isDepot(route.back())) throw exception();
		for (int i = 0; i < route.size(); ++i) {
			if (isDepot(route[i])) {
				result.push_back(i);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Parameter_CMTVRPTW_ArcFlow::depotIndex", exc);
	}
	return result;
}


vector<vector<int>> Parameter_CMTVRPTW_ArcFlow::getPaths(const vector<int>& route) const {
	vector<vector<int>> result;
	try {
		vector<int> indices = depotIndex(route);
		if (indices.size() < 2) throw exception();
		for (int i = 0; i < indices.size() - 1; ++i) {
			result.push_back(vector<int>(route.begin() + indices[i], route.begin() + indices[i + 1] + 1));
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Parameter_CMTVRPTW_ArcFlow::getPaths", exc);
	}
	return result;
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
		auto env = model.getEnv();
		const int N = parameter.N;
		const int K = parameter.K;
		const int NUM = X.getSize();
		IloExpr vehicles(env);
		for (int i = 0; i < NUM; ++i) {
			model.add(X[i][0] == IloFalse);
			model.add(X[N][i] == IloFalse);
			if (parameter.ExistingArcs[0][i]) vehicles += X[0][i];
			for (int j = 0; j < NUM; ++j) {
				if (!parameter.ExistingArcs[i][j] || (parameter.isDepot(i) && parameter.isDepot(j))) {
					model.add(X[i][j] == IloFalse);
				}
			}
		}
		model.add(vehicles <= parameter.V);
		vehicles.end();

		for (int i = 1; i < NUM; ++i) {
			if (i == N) continue;
			IloExpr out(env), in(env);
			for (int j = 0; j < NUM; ++j) {
				if (parameter.ExistingArcs[i][j]) out += X[i][j];
				if (parameter.ExistingArcs[j][i]) in += X[j][i];
			}
			if (1 <= i && i <= N - 1) {
				model.add(out == in);
				model.add(out == 1);
			}
			else if (N + 1 <= i && i <= N + K) {
				model.add(out == in);
				model.add(out <= 1);
			}
			out.end(), in.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsX", exc);
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


void setConstraintsCapacity(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Z) {
	try {
		for (int i = 0; i < X.getSize(); ++i) {
			if (parameter.isDepot(i)) {
				model.add(Z[i] == 0);
			}
			for (int j = 0; j < X[i].getSize(); ++j) {
				if (!parameter.isDepot(j) && parameter.ExistingArcs[i][j]) {
					const double q = parameter.QuantityNode[j];
					const double M = parameter.Capacity + q;
					model.add(Z[i] + q - Z[j] <= (1 - X[i][j]) * M);
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsCapacity", exc);
	}
}


void reduceSymmetry(const Parameter_CMTVRPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y) {
	try {
		auto env = model.getEnv();
		const int N = parameter.N;
		const int K = parameter.K;
		const int NUM = X.getSize();
		for (int i = N + 1; i < N + K; ++i) {
			IloExpr pre(env), suc(env);
			for (int j = 0; j < NUM; ++j) {
				if (parameter.ExistingArcs[i][j]) pre += X[i][j];
				if (parameter.ExistingArcs[i + 1][j]) suc += X[i + 1][j];
			}
			model.add(pre >= suc);
			model.add(Y[i] <= Y[i + 1]);
			pre.end(), suc.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("reduceSymmetry", exc);
	}
}


tuple<double, double, double> CMTVRPTW_ArcFlow(const Parameter_CMTVRPTW_ArcFlow& parameter, ostream& output) {
	tuple<double, double, double> result = make_tuple(-1, -1, -1);
	IloEnv env;
	try {
		const int N = parameter.N, K = parameter.K;
		// Define the variables.
		IloBoolVarArray2 X(env, N + K + 1);
		for (int i = 0; i < X.getSize(); ++i) {
			X[i] = IloBoolVarArray(env, N + K + 1);
		}

		IloNumArray early(env, N + K + 1), late(env, N + K + 1);
		for (int i = 0; i < early.getSize(); ++i) {
			early[i] = parameter.TimeWindow[i].first;
			late[i] = parameter.TimeWindow[i].second;
		}
		IloNumVarArray Y(env, early, late);

		IloNumVarArray Z(env, N + K + 1, 0, parameter.Capacity);

		// Define the model.
		IloModel model(env);
		setObjective(parameter, model, X);
		setConstraintsX(parameter, model, X);
		setConstraintsTimeWindow(parameter, model, X, Y);
		setConstraintsCapacity(parameter, model, X, Z);
		reduceSymmetry(parameter, model, X, Y);

		// Solve the model.
		IloCplex cplex(model);
		cplex.setParam(IloCplex::Param::TimeLimit, parameter.timeLimit);
		cplex.solve();

		IloAlgorithm::Status status = cplex.getStatus();
		env.out() << "solution status is " << status << endl;
		if (equalToReal(status, IloAlgorithm::Optimal, PPM) || equalToReal(status, IloAlgorithm::Feasible, PPM)) {
			double objective = cplex.getObjValue();
			double gap = cplex.getMIPRelativeGap();
			double time = cplex.getDetTime() / Thousand;
			result = make_tuple(objective, gap, time);
			env.out() << "solution value  is " << get<0>(result) << endl;
			env.out() << "solution gap    is " << get<1>(result) << endl;
			env.out() << "elapsed  time   is " << get<2>(result) << endl;
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("CMTVRPTW_ArcFlow", exc);
	}
	env.end();
	return result;
}


tuple<double, double, double> CMTVRPTW_ArcFlow(const string& strInput, const int numDummyDepots, ostream& output) {
	tuple<double, double, double> result = make_tuple(-1, -1, -1);
	try {
		Data_Input_VRPTW input_VRPTW;
		readFromFileVRPTW(input_VRPTW, strInput);
		input_VRPTW.constrainResource = { true,false,true };
		input_VRPTW.preprocess();

		Parameter_CMTVRPTW_ArcFlow parameter;
		parameter.V = input_VRPTW.MaxNumVehicles;
		parameter.Capacity = input_VRPTW.capacity;
		parameter.N = input_VRPTW.NumVertices;
		parameter.K = numDummyDepots;

		const int NUM = parameter.N + parameter.K + 1;
		parameter.Quantity = parameter.Distance = parameter.Time = vector<vector<double>>(NUM, vector<double>(NUM));
		parameter.QuantityNode = vector<double>(NUM);
		parameter.TimeWindow = vector<pair<double, double>>(NUM);
		parameter.ExistingArcs = vector<vector<bool>>(NUM, vector<bool>(NUM));

		for (int i = 0; i < NUM; ++i) {
			const int I = parameter.realIndex(i);
			parameter.QuantityNode[i] = 2 * input_VRPTW.Quantity[0][I];
			parameter.TimeWindow[i] = input_VRPTW.TimeWindow[I];
			for (int j = 0; j < NUM; ++j) {
				const int J = parameter.realIndex(j);
				parameter.Quantity[i][j] = input_VRPTW.Quantity[I][J];
				parameter.Distance[i][j] = input_VRPTW.Distance[I][J];
				parameter.Time[i][j] = input_VRPTW.Time[I][J];
				parameter.ExistingArcs[i][j] = input_VRPTW.ExistingArcs[I][J];
			}
		}

		parameter.timeLimit = 3600 * 2;
		result = CMTVRPTW_ArcFlow(parameter, output);
	}
	catch (const exception& exc) {
		printErrorAndExit("CMTVRPTW_ArcFlow", exc);
	}
	return result;
}


void Test_CMTVRPTW_ArcFlow(const string& outFile) {
	try {
		ofstream os(outFile);
		if (!os) throw exception();
		os << "Name" << '\t' << "Vertices" << '\t' << "Capacity" << '\t' << "Density" << '\t'
			<< "Objective" << '\t' << "Gap" << '\t' << "Time" << endl;

		vector<string> folders = { "data//CMTVRPTW//Solomon Type 2 - 25//",
			"data//CMTVRPTW//Solomon Type 2 - 40//",
			"data//CMTVRPTW//Solomon Type 2 - 50//" };
		vector<string> names;
		getFiles(folders[0], vector<string>(), names);

		for (const auto& folder : folders) {
			for (const auto& name : names) {
				//if (folder == "data//CMTVRPTW//Solomon Type 2 - 50//" && name[0] == 'R' && name[1] == '2') continue;

				string strInput = folder + name;
				Data_Input_VRPTW inputVRPTW;
				inputVRPTW.constrainResource = { true,false,true };
				readFromFileVRPTW(inputVRPTW, strInput);
				inputVRPTW.preprocess();

				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "Instance: " << inputVRPTW.name << '\t' << "NumVertices: " << inputVRPTW.NumVertices << endl;
				os << inputVRPTW.name << '\t' << inputVRPTW.NumVertices << '\t' << inputVRPTW.capacity << '\t' << inputVRPTW.density << '\t';

				const int numDummyDepots = 5;
				auto result = CMTVRPTW_ArcFlow(strInput, numDummyDepots, cout);
				os << get<0>(result) << '\t' << get<1>(result) << '\t' << get<2>(result) << endl;
			}
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("Test_CMTVRPTW_ArcFlow", exc);
	}
}


void Test_CMTVRPTW_SP(const string& outFile) {
	try {
		ofstream os(outFile);
		if (!os) throw exception();
		os << "Instance" << '\t' << "Vertices" << '\t' << "Capacity" << '\t' << "Density" << '\t'
			<< "LB_1" << '\t' << "Gap_1" << '\t' << "Time_LB_1" << '\t' << "Size_1" << '\t' << "Time_Enumeration" << '\t'
			<< "LB_2" << '\t' << "Gap_2" << '\t' << "Time_LB_2" << '\t' << "Size_2" << '\t' << "Time_BC" << '\t'
			<< "Objective" << '\t' << "Time_Total" << endl;

		vector<string> folders = { "data//CMTVRPTW//Solomon Type 2 - 25//",
			"data//CMTVRPTW//Solomon Type 2 - 40//",
			"data//CMTVRPTW//Solomon Type 2 - 50//" };
		vector<string> names;
		getFiles(folders[0], vector<string>(), names);
		vector<string> inputs;
		for (const auto& folder : folders) {
			for (const auto& name : names) {
				inputs.push_back(folder + name);
			}
		}
		set<int> ignore;
		int start = 15;
		for (int i = start; i < inputs.size(); ++i) {
			if (ignore.find(i) != ignore.end()) continue;
			string strInput = inputs[i];
			Data_Input_VRPTW inputVRPTW;
			inputVRPTW.constrainResource = { true,false,true };
			readFromFileVRPTW(inputVRPTW, strInput);
			inputVRPTW.preprocess();

			cout << "*****************************************" << endl;
			cout << "*****************************************" << endl;
			cout << "*****************************************" << endl;
			cout << "Instance: " << inputVRPTW.name << '\t' << "NumVertices: " << inputVRPTW.NumVertices << endl;
			os << inputVRPTW.name << '\t' << inputVRPTW.NumVertices << '\t' << inputVRPTW.capacity << '\t' << inputVRPTW.density << '\t';

			clock_t last = clock();
			auto result = CMTVRPTW_SP(strInput);
			os << result.LB_1 << '\t' << gap(result.LB_1, result.objective) << '\t' << result.time_LB_1 << '\t' << result.size_1 << '\t'
				<< result.time_enumeration << '\t' << result.LB_2 << '\t' << gap(result.LB_2, result.objective) << '\t' << result.time_LB_2
				<< '\t' << result.size_2 << '\t' << result.time_BC << '\t' << result.objective << '\t' << runTime(last) << endl;
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("Test_CMTVRPTW_ArcFlow", exc);
	}
}

