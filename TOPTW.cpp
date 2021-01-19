#include"TOPTW.h"


void TOPTW_CG::columnGeneration(const Parameter_TOPTW_CG& parameter, Solution_TOPTW_CG& solution, ostream& output) {
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
		IloNumVarArray Y(env);



	}
	catch (const exception& exc) {
		printErrorAndExit("TOPTW_CG::columnGeneration", exc);
	}
	env.end();
}

