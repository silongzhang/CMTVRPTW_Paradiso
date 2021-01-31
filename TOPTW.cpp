#include"TOPTW.h"


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
		Cost_ESPPRC cst(-InfinityNeg, 0);
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
		solution.integerSolution.clear();
		for (int i = 0; i < X.getSize(); ++i) {
			if (equalToReal(cplex.getValue(X[i]), 1, PPM)) {
				solution.integerSolution.push_back(columns[i]);
			}
			else if (!equalToReal(cplex.getValue(X[i]), 0, PPM)) throw exception();
		}
		solution.UB_Integer = 0;
		for (const auto& elem : solution.integerSolution) {
			solution.UB_Integer += elem.getRealCost();
		}
		if (!equalToReal(solution.UB_Integer, cplex.getObjValue(), PPM)) throw exception();
		solution.UB_Integer = -solution.UB_Integer;
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::getIntegerSolution", exc);
	}
}


void setRangeArray(const Parameter_TOPTW_CG& parameter, IloModel& modelRMP, IloRangeArray& constraintRMP) {
	try {
		auto env = modelRMP.getEnv();

		// Constraint: the number of available vehicles is limited.
		constraintRMP.add(IloAdd(modelRMP, IloRange(env, -IloInfinity, parameter.input_VRPTW.MaxNumVehicles)));

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

		// Constraint: due to branching on the number of vehicles.
		auto bovn = parameter.branchOnVehicleNumber;
		bovn.second ? constraintRMP.add(IloAdd(modelRMP, IloRange(env, bovn.first, IloInfinity))) : 
			constraintRMP.add(IloAdd(modelRMP, IloRange(env, -IloInfinity, bovn.first)));
	}
	catch (const exception& exc) {
		printErrorAndExit("setRangeArray", exc);
	}
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
		InitiateRMP(parameter, inputESPPRC, objectiveRMP, constraintRMP, X);

		// Solve the problem iteratively.
		IloCplex solverRMP(modelRMP);
		for (int iter = 1; true; ++iter) {
			// Solve the RMP.
			strLog = "\nSolve the master problem for the " + numToStr(iter) + "th time.";
			print(parameter.allowPrintLog, output, strLog);
			if (!solverRMP.solve()) throw exception();

			// Get dual values.
			IloNumArray dualValue(env);
			solverRMP.getDuals(dualValue, constraintRMP);
			double fixedReducedCost = dualValue[0] + dualValue[inputESPPRC.NumVertices];

			// Renew reduced cost.
			renewReducedCost(inputESPPRC, parameter, dualValue);

			// Solve the subproblem.
			Data_Auxiliary_ESPPRC auxiliary;
			inputESPPRC.mustOptimal = false;
			inputESPPRC.minRunTime = 0;
			auto resultSP = DPAlgorithmESPPRC(inputESPPRC, auxiliary, output);

			// Stopping criterion.
			if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost() + fixedReducedCost, -PPM, 0)) {
				inputESPPRC.mustOptimal = true;
				resultSP = DPAlgorithmESPPRC(inputESPPRC, auxiliary, output);
				if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost() + fixedReducedCost, -PPM, 0)) break;
			}

			// Add new columns.
			for (const auto& elem : resultSP) {
				addColumn(elem, objectiveRMP, constraintRMP, X);
			}
			strLog = numToStr(resultSP.size()) + " routes are added." + '\t' +
				"The minimum reduced cost of added routes: " + numToStr(resultSP.begin()->getReducedCost() + fixedReducedCost);
			print(parameter.allowPrintLog, output, strLog);
		}

		// The solution is feasible if values of all artificial variables are zero.
		solution.feasible = isFeasible(parameter, solverRMP, X);

		if (solution.feasible) {
			// Check whether the optimal solution is an integer solution.
			solution.integer = isBool(solverRMP, X);
			if (solution.integer) {
				getIntegerSolution(solverRMP, X, solution);
			}

			// Set other information.
			solution.LB_Linear = -solverRMP.getObjValue();
			solution.explored = true;
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::columnGeneration", exc);
	}
	env.end();
}


void BBNODE::reviseParameter() {
	try {
		parameter.reviseInputVRPTW();
		parameter.reviseInitialRoutes();
		parameter.reviseNumArtificial();
	}
	catch (const exception& exc) {
		printErrorAndExit("BBNODE::reviseParameter", exc);
	}
}


void BBNODE::solve(ostream& output) {
	try {
		reviseParameter();
		model.columnGeneration(parameter, solution, output);
	}
	catch (const exception& exc) {
		printErrorAndExit("BBNODE::solve", exc);
	}
}


BBNODE generateRootNode(const Data_Input_VRPTW& inputVRPTW) {
	BBNODE rootNode;
	try {
		Parameter_TOPTW_CG rootParameter;
		rootParameter.input_VRPTW = inputVRPTW;
		rootParameter.allowPrintLog = true;
		rootParameter.branchOnVehicleNumber = make_pair(inputVRPTW.MaxNumVehicles, false);
		rootParameter.numArtificial = 0;
		rootNode.parameter = rootParameter;

		Solution_TOPTW_CG rootSolution;
		rootSolution.explored = false;
		rootSolution.LB_Linear = InfinityNeg;
		rootSolution.UB_Integer = InfinityPos;
		rootNode.solution = rootSolution;

		rootNode.model = TOPTW_CG();
	}
	catch (const exception& exc) {
		printErrorAndExit("generateRootNode", exc);
	}
	return rootNode;
}


// To be completed.
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

		clock_t last = clock();
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
				cout << "Instance: " << inputVRPTW.name << '\t' << "NumVertices: " << inputVRPTW.NumVertices << '\t' << "Time: " << runTime(last) << endl;
				os << inputVRPTW.name << '\t' << inputVRPTW.NumVertices << '\t' << inputVRPTW.density << '\t';

				last = clock();
				auto rootNode = generateRootNode(inputVRPTW);
				rootNode.solve(cout);

				os << rootNode.solution.explored << '\t' << rootNode.solution.feasible << '\t' << rootNode.solution.integer << '\t'
					<< rootNode.solution.LB_Linear << '\t' << rootNode.solution.UB_Integer << '\t';
				for (const auto& route : rootNode.solution.integerSolution) {
					os << "[ ";
					for (const auto& elem : route.getPath()) {
						os << elem << ' ';
					}
					os << "]. ";
				}
			}
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("testTOPTW", exc);
	}
}
