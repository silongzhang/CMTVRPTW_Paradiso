#include"TOPTW.h"


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


void TOPTW_CG::InitiateRMP(const Parameter_TOPTW_CG& parameter, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X) {
	try {
		clearColumns();
		for (const auto& rhs : parameter.initialRoutes) {
			addColumn(rhs, objectiveRMP, constraintRMP, X);
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::InitiateRMP", exc);
	}
}


void renewReducedCost(Data_Input_ESPPRC& inputESPPRC, const Parameter_TOPTW_CG& parameter, const IloNumArray& dualValue) {
	try {
		for (int i = 0; i < parameter.inputESPPRC.NumVertices; ++i) {
			double dualI = i == 0 ? 0 : dualValue[i];
			for (int j = 0; j < parameter.inputESPPRC.NumVertices; ++j) {
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


void TOPTW_CG::columnGeneration(const Parameter_TOPTW_CG& parameter, ostream& output) {
	IloEnv env;
	try {
		string strLog = "Begin running the procedure titled TOPTW_CG::columnGeneration.";
		print(parameter.allowPrintLog, output, strLog);

		// Define the model of restricted master problem.
		IloModel modelRMP(env);

		// Define the objective function of restricted master problem.
		IloObjective objectiveRMP = IloAdd(modelRMP, IloMaximize(env));

		// Define bounds of constraints of restricted master problem.
		IloNumArray RightSide(env, parameter.input_VRPTW.NumVertices);
		RightSide[0] = parameter.input_VRPTW.MaxNumVehicles;
		for (int i = 1; i < parameter.input_VRPTW.NumVertices; ++i) RightSide[i] = 1;
		IloRangeArray constraintRMP = IloAdd(modelRMP, IloRangeArray(env, -IloInfinity, RightSide));

		// Define the variables of restricted master problem.
		IloNumVarArray X(env);

		// Initiate the model of restricted master problem.
		InitiateRMP(parameter, objectiveRMP, constraintRMP, X);

		// Define the solver of restricted master problem.
		IloCplex solverRMP(modelRMP);

		Data_Input_ESPPRC inputESPPRC = parameter.inputESPPRC;
		for (int iter = 1; true; ++iter) {
			// Solve the RMP.
			strLog = "\nSolve the master problem for the " + numToStr(iter) + "th time.";
			print(parameter.allowPrintLog, output, strLog);
			solveModel(solverRMP);

			// Get dual values.
			IloNumArray dualValue(env);
			solverRMP.getDuals(dualValue, constraintRMP);

			// Renew reduced cost.
			renewReducedCost(inputESPPRC, parameter, dualValue);

			// Solve the subproblem.
			Data_Auxiliary_ESPPRC auxiliary;
			inputESPPRC.mustOptimal = false;
			inputESPPRC.minRunTime = 0;
			auto resultSP = DPAlgorithmESPPRC(inputESPPRC, auxiliary, output);

			// Stopping criterion.
			if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost(), -PPM, 0)) {
				inputESPPRC.mustOptimal = true;
				resultSP = DPAlgorithmESPPRC(inputESPPRC, auxiliary, output);
				if (resultSP.empty() || greaterThanReal(resultSP.begin()->getReducedCost(), -PPM, 0)) break;
			}

			// Add new columns.
			for (const auto& elem : resultSP) {
				addColumn(elem, objectiveRMP, constraintRMP, X);
			}
			strLog = numToStr(resultSP.size()) + " routes are added." + '\t' +
				"The minimum reduced cost of added routes: " + numToStr(resultSP.begin()->getReducedCost());
			print(parameter.allowPrintLog, output, strLog);
		}

		// Check whether the optimal solution of this node is integral or linear, then get other information about this optimal solution.

		// If the optimal solution is linear, then get two child nodes by branching.
	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::columnGeneration", exc);
	}
	env.end();
}

