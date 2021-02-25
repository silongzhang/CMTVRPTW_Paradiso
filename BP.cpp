#include"BP.h"


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
		rootSolution.objective = InfinityNeg;
		rootSolution.UB_Integer_Value = InfinityPos;
		rootNode.solution = rootSolution;

		rootNode.model = TOPTW_CG();
	}
	catch (const exception& exc) {
		printErrorAndExit("generateRootNode", exc);
	}
	return rootNode;
}

