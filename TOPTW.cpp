#include"TOPTW.h"
#include"BP.h"


void Parameter_TOPTW_CG::reviseInputVRPTW() {
	try {
		// Revise the existing status of arcs.
		for (const auto& elem : branchOnArcs) {
			int i = elem.first.first, j = elem.first.second;
			if (elem.second) {
				// Arc (i, j) must be visited if i and j are visited.
				for (int k = 0; k < input_VRPTW.NumVertices; ++k) {
					if (k != j) {
						input_VRPTW.ExistingArcs[i][k] = false;
					}
					if (k != i) {
						input_VRPTW.ExistingArcs[k][j] = false;
					}
				}
			}
			else {
				// Arc (i, j) cannot be visited.
				input_VRPTW.ExistingArcs[i][j] = false;
			}
		}

		// Revise the existing status of vertices.
		for (const auto& elem : branchOnVertices) {
			if (!elem.second) {
				int i = elem.first;
				for (int j = 0; j < input_VRPTW.NumVertices; ++j) {
					input_VRPTW.ExistingArcs[i][j] = input_VRPTW.ExistingArcs[j][i] = false;
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Parameter_TOPTW_CG::reviseInputVRPTW", exc);
	}
}


void Parameter_TOPTW_CG::reviseInitialRoutes() {
	try {
		Data_Input_ESPPRC inputESPPRC = setParametersInputESPPRCFromInputVRPTW(input_VRPTW);
		vector<Route_VRPTW> routes;
		for (auto&& elem : initialRoutes) {
			if (elem.feasible(inputESPPRC)) {
				routes.push_back(std::move(elem));
			}
		}
		swap(routes, initialRoutes);
	}
	catch (const exception& exc) {
		printErrorAndExit("Parameter_TOPTW_CG::reviseInitialRoutes", exc);
	}
}


void TOPTW_CG::addColumn(const Route_VRPTW& rhs, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X) {
	try {
		// Add the column.
		columns.push_back(rhs);

		// The coefficient in objective function.
		IloNumColumn col = objectiveRMP(rhs.getRealCost());

		// The coefficient in constraint for limiting the number of vehicles.
		col += constraintRMP[0](1);
		col += constraintRMP[constraintRMP.getSize() - 1](1);

		// The coefficients in constraints for each vertex can be visited at most once.
		unordered_map<int, int> unMp = getCount(rhs.getPath());
		for (const auto& elem : unMp) {
			if (elem.first != 0) {
				col += constraintRMP[elem.first](1);
			}
		}

		// Add the new variable.
		X.add(IloNumVar(col, 0, IloInfinity, ILOFLOAT));

		col.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::addColumn", exc);
	}
}


void TOPTW_CG::InitiateRMP(const Parameter_TOPTW_CG& parameter, const Data_Input_ESPPRC& inputESPPRC, IloObjective& objectiveRMP, 
	IloRangeArray& constraintRMP, IloNumVarArray& X) {
	try {
		clearColumns();

		// Generate a dummy column.
		vector<int> dummyPath;
		for (const auto& elem : parameter.branchOnVertices) {
			if (elem.second) {
				dummyPath.push_back(elem.first);
			}
		}
		Cost_ESPPRC cst(InfinityNeg, 0);
		Route_VRPTW dummyRoute(dummyPath, cst);

		// Generate other dummy columns and add them.
		vector<Route_VRPTW> dummyRoutes({ dummyRoute });
		for (int i = 1; i < parameter.numArtificial; ++i) {
			dummyRoutes.push_back(Route_VRPTW(vector<int>(), cst));
		}
		for (int i = 0; i < parameter.numArtificial; ++i) {
			addColumn(dummyRoutes[i], objectiveRMP, constraintRMP, X);
		}

		// Add initial columns.
		for (const auto& rhs : parameter.initialRoutes) {
			if (!rhs.feasible(inputESPPRC)) throw exception();
			addColumn(rhs, objectiveRMP, constraintRMP, X);
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::InitiateRMP", exc);
	}
}


void renewReducedCost(Data_Input_ESPPRC& inputESPPRC, const Parameter_TOPTW_CG& parameter, const IloNumArray& dualValue) {
	try {
		for (int i = 0; i < parameter.input_VRPTW.NumVertices; ++i) {
			double dualI = i == 0 ? 0 : dualValue[i];
			for (int j = 0; j < parameter.input_VRPTW.NumVertices; ++j) {
				if (i == j) {
					inputESPPRC.ReducedCost[i][j] = 0;
				}
				else {
					double dualJ = j == 0 ? 0 : dualValue[j];
					inputESPPRC.ReducedCost[i][j] = (dualI + dualJ) / 2 - inputESPPRC.RealCost[i][j];
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("renewReducedCost", exc);
	}
}


bool isFeasible(const Parameter_TOPTW_CG& parameter, const IloCplex& cplex, const IloNumVarArray& X) {
	try {
		for (int i = 0; i < parameter.numArtificial; ++i) {
			if (!equalToReal(cplex.getValue(X[i]), 0, PPM)) {
				return false;
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("isFeasible", exc);
	}
	return true;
}


bool isBool(const IloCplex& cplex, const IloNumVarArray& X) {
	try {
		for (int i = 0; i < X.getSize(); ++i) {
			if (!(equalToReal(cplex.getValue(X[i]), 0, PPM) || equalToReal(cplex.getValue(X[i]), 1, PPM))) {
				return false;
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("isBool", exc);
	}
	return true;
}


void TOPTW_CG::getIntegerSolution(const IloCplex& cplex, const IloNumVarArray& X, Solution_TOPTW_CG& solution) {
	try {
		solution.UB_Integer_Solution.clear();
		for (int i = 0; i < X.getSize(); ++i) {
			if (equalToReal(cplex.getValue(X[i]), 1, PPM)) {
				solution.UB_Integer_Solution.push_back(columns[i]);
			}
			else if (!equalToReal(cplex.getValue(X[i]), 0, PPM)) throw exception();
		}
		solution.UB_Integer_Value = 0;
		for (const auto& elem : solution.UB_Integer_Solution) {
			solution.UB_Integer_Value += elem.getRealCost();
		}
		if (!equalToReal(solution.UB_Integer_Value, cplex.getObjValue(), PPM)) throw exception();
		solution.UB_Integer_Value = -solution.UB_Integer_Value;
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::getIntegerSolution", exc);
	}
}


void TOPTW_CG::getSolution(const Parameter_TOPTW_CG& parameter, const IloCplex& solverRMP,
	const IloRangeArray& constraintRMP, const IloNumVarArray& X, Solution_TOPTW_CG& solution) {
	try {
		// The solution is feasible if values of all artificial variables are zero.
		solution.feasible = isFeasible(parameter, solverRMP, X);
		solution.objective = -solverRMP.getObjValue();
		solution.explored = true;

		if (solution.feasible) {
			// Check whether the optimal solution is an integer solution.
			solution.integer = isBool(solverRMP, X);
			if (solution.integer && lessThanReal(solution.objective, solution.UB_Integer_Value, PPM)) {
				getIntegerSolution(solverRMP, X, solution);
			}

			vector<int> basicVariables;
			for (int i = 0; i < X.getSize(); ++i) {
				if (greaterThanReal(solverRMP.getValue(X[i]), 0, PPM)) {
					basicVariables.push_back(i);
				}
			}

			solution.numVehicles = 0;
			for (const auto& i : basicVariables) {
				solution.numVehicles += solverRMP.getValue(X[i]);
			}

			solution.visitVertices.clear();
			solution.visitArcs.clear();
			solution.visitVertices.resize(parameter.input_VRPTW.NumVertices);
			solution.visitArcs.resize(parameter.input_VRPTW.NumVertices);
			solution.visitArcs[0] = vector<double>(parameter.input_VRPTW.NumVertices, 0);
			for (int i = 1; i < parameter.input_VRPTW.NumVertices; ++i) {
				solution.visitVertices[i] = solverRMP.getValue(constraintRMP[i].getExpr());
				solution.visitArcs[i] = vector<double>(parameter.input_VRPTW.NumVertices, 0);
			}

			for (const auto& i : basicVariables) {
				const auto path = columns[i].getPath();
				auto pre = path.begin();
				for (auto suc = pre + 1; suc != path.end(); ++pre, ++suc) {
					solution.visitArcs[*pre][*suc] += solverRMP.getValue(X[i]);
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::getSolution", exc);
	}
}


void setRangeArray(const Parameter_TOPTW_CG& parameter, IloModel& modelRMP, IloRangeArray& constraintRMP) {
	try {
		auto env = modelRMP.getEnv();

		// Constraint: the number of vehicles can not be greater than aboveNum.
		const auto bovn = parameter.branchOnVehicleNumber;
		const int aboveNum = bovn.second ? parameter.input_VRPTW.MaxNumVehicles : min(parameter.input_VRPTW.MaxNumVehicles, bovn.first);
		constraintRMP.add(IloAdd(modelRMP, IloRange(env, -IloInfinity, aboveNum)));

		// Constraints: a vertex can be visited at most once, also, constraints due to branching on vertices are considered.
		for (int i = 1; i < parameter.input_VRPTW.NumVertices; ++i) {
			auto pos = parameter.branchOnVertices.find(i);
			if (pos == parameter.branchOnVertices.end()) {
				constraintRMP.add(IloAdd(modelRMP, IloRange(env, -IloInfinity, 1)));
			}
			else if (pos->second) {
				constraintRMP.add(IloAdd(modelRMP, IloRange(env, 1, 1)));
			}
			else {
				constraintRMP.add(IloAdd(modelRMP, IloRange(env, 0, 0)));
			}
		}

		// Constraint: the number of vehicles can not be less than underNum.
		const int underNum = bovn.second ? max(bovn.first, 0) : 0;
		constraintRMP.add(IloAdd(modelRMP, IloRange(env, underNum, IloInfinity)));
	}
	catch (const exception& exc) {
		printErrorAndExit("setRangeArray", exc);
	}
}


multiset<Label_ESPPRC, Label_ESPPRC_Sort_Criterion> subFunction_TOPTW_CG(const Parameter_TOPTW_CG& parameter, Data_Input_ESPPRC& inputESPPRC,
	IloCplex solverRMP, IloRangeArray constraintRMP, double& fixedReducedCost, int& iter, ostream& output, clock_t& start) {
	try {
		auto env = solverRMP.getEnv();

		// Solve the RMP.
		string strLog = "\nSolve the master problem for the " + numToStr(++iter) + "th time.";
		print(parameter.allowPrintLog, output, strLog);
		if (!solverRMP.solve()) throw exception();
		strLog = "Time: " + numToStr(runTime(start)) + "\t" + "Objective value: " + numToStr(solverRMP.getObjValue());
		print(parameter.allowPrintLog, output, strLog);

		// Get dual values.
		IloNumArray dualValue(env);
		solverRMP.getDuals(dualValue, constraintRMP);
		fixedReducedCost = dualValue[0] + dualValue[inputESPPRC.NumVertices];

		// Renew reduced cost.
		renewReducedCost(inputESPPRC, parameter, dualValue);
	}
	catch (const exception& exc) {
		printErrorAndExit("subFunction_TOPTW_CG", exc);
	}

	// Solve the subproblem.
	Data_Auxiliary_ESPPRC auxiliary;
	return DPAlgorithmESPPRC(inputESPPRC, auxiliary, output);
}


void TOPTW_CG::columnGeneration(const Parameter_TOPTW_CG& parameter, Solution_TOPTW_CG& solution, ostream& output) {
	IloEnv env;
	try {
		string strLog = "Begin running the procedure titled TOPTW_CG::columnGeneration.";
		print(parameter.allowPrintLog, output, strLog);

		// Define the model of restricted master problem.
		IloModel modelRMP(env);
		IloNumVarArray X(env);
		IloObjective objectiveRMP = IloAdd(modelRMP, IloMaximize(env));
		IloRangeArray constraintRMP(env);
		setRangeArray(parameter, modelRMP, constraintRMP);

		// Initialization.
		Data_Input_ESPPRC inputESPPRC = setParametersInputESPPRCFromInputVRPTW(parameter.input_VRPTW);
		inputESPPRC.maxRunTime = 3600;
		inputESPPRC.maxNumCandidates = 1e9;
		inputESPPRC.maxNumPotentialEachStep = 1e6;
		inputESPPRC.allowPrintLog = false;
		solution.explored = solution.feasible = solution.integer = false;
		InitiateRMP(parameter, inputESPPRC, objectiveRMP, constraintRMP, X);

		// Solve the problem iteratively.
		IloCplex solverRMP(modelRMP);
		solverRMP.setOut(env.getNullStream());
		clock_t start = clock();
		double fixedReducedCost;
		for (int iter = 0; true;) {
			// Solve the subproblem.
			inputESPPRC.mustOptimal = false;
			auto resultSP = subFunction_TOPTW_CG(parameter, inputESPPRC, solverRMP, constraintRMP, fixedReducedCost, iter, output, start);

			// Stopping criterion.
			if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost() + fixedReducedCost, -PPM, 0)) {
				// When the LP is feasible, the current solution is not an interger solution, 
				// and the lower bound (the negative of the objective) is less than the upper bound, we can branch directly, 
				// instead of performing a time-consuming procedure to obtain the optimal objective of the LP.
				if (isFeasible(parameter, solverRMP, X) && !isBool(solverRMP, X) && 
					lessThanReal(-solverRMP.getObjValue(), solution.UB_Integer_Value, PPM)) {
					break;
				}
				else {
					inputESPPRC.mustOptimal = true;
					resultSP = subFunction_TOPTW_CG(parameter, inputESPPRC, solverRMP, constraintRMP, fixedReducedCost, iter, output, start);
					if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost() + fixedReducedCost, -PPM, 0)) break;
				}
			}

			// Add new columns.
			for (const auto& elem : resultSP) {
				addColumn(elem, objectiveRMP, constraintRMP, X);
			}
			strLog = numToStr(resultSP.size()) + " routes are added." + '\t' +
				"The minimum reduced cost of added routes: " + numToStr(resultSP.begin()->getReducedCost() + fixedReducedCost);
			print(parameter.allowPrintLog, output, strLog);
		}
		strLog = "Time: " + numToStr(runTime(start));
		print(parameter.allowPrintLog, output, strLog);

		getSolution(parameter, solverRMP, constraintRMP, X, solution);
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::columnGeneration", exc);
	}
	env.end();
}


void testTOPTW() {
	try {
		string outFile = "data//CMTVRPTW//Test//TOPTW//Output//testTOPTW.txt";
		ofstream os(outFile);
		if (!os) throw exception();

		Data_Input_VRPTW inputVRPTW;
		inputVRPTW.constrainResource = { false,false,true };

		vector<string> folders = { "data//CMTVRPTW//Test//TOPTW//Input//Solomon Type 2 - 25//",
			"data//CMTVRPTW//Test//TOPTW//Input//Solomon Type 2 - 40//",
			"data//CMTVRPTW//Test//TOPTW//Input//Solomon Type 2 - 50//" };
		vector<string> names;
		getFiles(folders[0], vector<string>(), names);

		clock_t start = clock();
		for (const auto& folder : folders) {
			for (const auto& name : names) {
				if (folder == "data//CMTVRPTW//Solomon Type 2 - 50//" && name[0] == 'R' && name[1] == '2') {
					continue;
				}
				string strInput = folder + name;
				readFromFileVRPTW(inputVRPTW, strInput);

				for (int i = 0; i < inputVRPTW.NumVertices; ++i) {
					for (int j = 0; j < inputVRPTW.NumVertices; ++j) {
						inputVRPTW.Quantity[i][j] = 0;
						inputVRPTW.Distance[i][j] = 0;
						if (i != 0 && i != j) {
							inputVRPTW.RealCost[i][j] = 1;
						}
						else {
							inputVRPTW.RealCost[i][j] = 0;
						}
					}
				}

				writeToFileVRPTW(inputVRPTW, strInput);

				inputVRPTW.preprocess();
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "Instance: " << inputVRPTW.name << '\t' << "NumVertices: " << inputVRPTW.NumVertices << '\t' << "Time: " << runTime(start) << endl;
				os << inputVRPTW.name << '\t' << inputVRPTW.NumVertices << '\t' << inputVRPTW.density << '\t';

				clock_t last = clock();
				Parameter_BP parameter;
				parameter.weightLB = parameter.weightDepth = 1;
				auto rootNode = generateRootNode(inputVRPTW, parameter);
				rootNode.solve(cout);

				os << rootNode.solution.explored << '\t' << rootNode.solution.feasible << '\t' << rootNode.solution.integer << '\t'
					<< rootNode.solution.objective << '\t' << rootNode.solution.UB_Integer_Value << '\t';
				for (const auto& route : rootNode.solution.UB_Integer_Solution) {
					os << "[ ";
					for (const auto& elem : route.getPath()) {
						os << elem << ' ';
					}
					os << "]. ";
				}
				os << endl;
			}
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("testTOPTW", exc);
	}
}


int realIndexTOPTW(int i, int N) {
	return i % N;
}


void setConstraintsDomainX(const Parameter_TOPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X) {
	try {
		const int N = parameter.input_VRPTW.NumVertices;
		for (int i = 0; i < X.getSize(); ++i) {
			model.add(X[i][i] == 0);
			for (int j = 0; j < X[i].getSize(); ++j) {
				if (!parameter.input_VRPTW.ExistingArcs[realIndexTOPTW(i, N)][realIndexTOPTW(j, N)]) {
					model.add(X[i][j] == 0);
				}
			}
		}

		for (int j = 0; j < X.getSize(); ++j) {
			model.add(X[j][0] == 0);
		}

		for (int j = 0; j < X[N].getSize(); ++j) {
			model.add(X[N][j] == 0);
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsDomainX", exc);
	}
}


void setConstraintsDomainY(const Parameter_TOPTW_ArcFlow& parameter, IloModel model, IloNumVarArray Y) {
	try {
		const int N = parameter.input_VRPTW.NumVertices;
		for (int i = 0; i < N; ++i) {
			double early = parameter.input_VRPTW.TimeWindow[i].first;
			double late = parameter.input_VRPTW.TimeWindow[i].second;
			model.add(early <= Y[i] <= late);
		}
		double early = parameter.input_VRPTW.TimeWindow[0].first;
		double late = parameter.input_VRPTW.TimeWindow[0].second;
		model.add(early <= Y[N] <= late);
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsDomainY", exc);
	}
}


void setConstraintsFlow(const Parameter_TOPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X) {
	try {
		const int N = parameter.input_VRPTW.NumVertices;
		auto env = model.getEnv();

		// The number of available vehicles is limited.
		IloExpr numVehicles(env);
		for (int j = 0; j < X[0].getSize(); ++j) {
			if (parameter.input_VRPTW.ExistingArcs[0][realIndexTOPTW(j, N)])
				numVehicles += X[0][j];
		}
		model.add(numVehicles <= parameter.input_VRPTW.MaxNumVehicles);
		numVehicles.end();

		for (int i = 1; i < X.getSize() - 1; ++i) {
			IloExpr out(env), in(env);
			for (int j = 0; j < X.getSize(); ++j) {
				if (parameter.input_VRPTW.ExistingArcs[realIndexTOPTW(i, N)][realIndexTOPTW(j, N)])
					out += X[i][j];
				if (parameter.input_VRPTW.ExistingArcs[realIndexTOPTW(j, N)][realIndexTOPTW(i, N)])
					in += X[j][i];
			}
			model.add(out == in);			// Flow balance.
			model.add(out <= 1);			// A customer can be visited at most once.
			out.end(), in.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsFlow", exc);
	}
}


void setConstraintsTimeWindow(const Parameter_TOPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y) {
	try {
		const int N = parameter.input_VRPTW.NumVertices;
		for (int i = 0; i < X.getSize(); ++i) {
			for (int j = 0; j < X.getSize(); ++j) {
				if (parameter.input_VRPTW.ExistingArcs[realIndexTOPTW(i, N)][realIndexTOPTW(j, N)]) {
					const double tm = parameter.input_VRPTW.Time[realIndexTOPTW(i, N)][realIndexTOPTW(j, N)];
					const double M = parameter.input_VRPTW.TimeWindow[realIndexTOPTW(i, N)].second + tm 
						- parameter.input_VRPTW.TimeWindow[realIndexTOPTW(j, N)].first;

					model.add(Y[i] + tm - Y[j] <= (1 - X[i][j]) * M);
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsTimeWindow", exc);
	}
}


void setObjective(const Parameter_TOPTW_ArcFlow& parameter, IloModel model, IloBoolVarArray2 X) {
	try {
		auto env = model.getEnv();
		const int N = parameter.input_VRPTW.NumVertices;
		IloExpr expr(env);
		for (int i = 0; i < X.getSize(); ++i) {
			for (int j = 0; j < X.getSize() - 1; ++j) {
				if (parameter.input_VRPTW.ExistingArcs[realIndexTOPTW(i, N)][realIndexTOPTW(j, N)])
					expr += X[i][j];
			}
		}
		model.add(IloMaximize(env, expr));
		expr.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("setObjective", exc);
	}
}


double TOPTW_ArcFlow(const Parameter_TOPTW_ArcFlow& parameter, ostream& output) {
	double result = InfinityNeg;
	IloEnv env;
	try {
		// Define the variables.
		const int N = parameter.input_VRPTW.NumVertices;
		IloBoolVarArray2 X(env, N + 1);
		for (int i = 0; i < X.getSize(); ++i) {
			X[i] = IloBoolVarArray(env, N + 1);
		}
		IloNumVarArray Y(env, N + 1);
		for (int i = 0; i < Y.getSize(); ++i) Y[i] = IloNumVar(env);

		// Define the model.
		IloModel model(env);
		setObjective(parameter, model, X);
		setConstraintsDomainX(parameter, model, X);
		setConstraintsDomainY(parameter, model, Y);
		setConstraintsFlow(parameter, model, X);
		setConstraintsTimeWindow(parameter, model, X, Y);

		// Solve the model.
		clock_t start = clock();
		IloCplex cplex(model);
		cplex.setOut(env.getNullStream());
		if (!cplex.solve()) throw exception();
		result = cplex.getObjValue();
		string strLog = "Time: " + numToStr(runTime(start)) + "\t" + "Objective value: " + numToStr(result);
		print(parameter.allowPrintLog, output, strLog);
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_ArcFlow", exc);
	}
	env.end();
	return result;
}


void test_TOPTW_BP_ArcFlow() {
	try {
		string outFile = "data//CMTVRPTW//Test//TOPTW//Output//TOPTW_BP_ArcFlow.txt";
		ofstream os(outFile);
		if (!os) throw exception();

		const string folder = "data//CMTVRPTW//Test//TOPTW//Input//TOPTW_BP_ArcFlow//";
		vector<string> names;
		getFiles(folder, vector<string>(), names);

		Parameter_BP parameter;
		parameter.weightDepth = 1;
		parameter.weightLB = 1;
		parameter.allowPrintLog = true;

		clock_t last = clock();
		for (const auto& name : names) {
			string strInput = folder + name;

			last = clock();
			cout << "#########################################" << endl;
			cout << "#########################################" << endl;
			cout << "#########################################" << endl;
			cout << "Method: ArcFlow" << '\t' << "Instance: " << strInput << endl;
			Parameter_TOPTW_ArcFlow parameterArcFlow;
			readFromFileVRPTW(parameterArcFlow.input_VRPTW, strInput);
			parameterArcFlow.input_VRPTW.constrainResource = { false,false,true };
			parameterArcFlow.input_VRPTW.preprocess();
			parameterArcFlow.allowPrintLog = true;

			double objectiveArcFlow = TOPTW_ArcFlow(parameterArcFlow, cout);
			os << parameterArcFlow.input_VRPTW.name << '\t' << parameterArcFlow.input_VRPTW.NumVertices << '\t' 
				<< parameterArcFlow.input_VRPTW.density << '\t' << runTime(last) << '\t' << objectiveArcFlow << '\t';

			last = clock();
			cout << "*****************************************" << endl;
			cout << "*****************************************" << endl;
			cout << "*****************************************" << endl;
			cout << "Method: BP" << '\t' << "Instance: " << strInput << endl;
			auto bestNode = TOPTW_BP(strInput, parameter);

			os << runTime(last) << '\t';
			os << bestNode.solution.explored << '\t' << bestNode.solution.feasible << '\t' << bestNode.solution.integer << '\t'
				<< bestNode.solution.objective << '\t' << bestNode.solution.UB_Integer_Value << '\t';
			for (const auto& route : bestNode.solution.UB_Integer_Solution) {
				os << "[ ";
				for (const auto& elem : route.getPath()) {
					os << elem << ' ';
				}
				os << "]. ";
			}
			os << endl;
		}
		os.close();
	}	
	catch (const exception& exc) {
		printErrorAndExit("test_TOPTW_BP_ArcFlow", exc);
	}
}

