#include"BP.h"


bool operator<(const BBNODE& lhs, const BBNODE& rhs) {
	return lhs.priority < rhs.priority;
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


BBNODE generateRootNode(const Data_Input_VRPTW& inputVRPTW, const Parameter_BP& parameter) {
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

		rootNode.depth = 1;
		rootNode.setPriority(parameter.weightLB, parameter.weightDepth);
	}
	catch (const exception& exc) {
		printErrorAndExit("generateRootNode", exc);
	}
	return rootNode;
}


BBNODE BPAlgorithm(const Data_Input_VRPTW& inputVRPTW, const Parameter_BP& parameter, ostream& output) {
	BBNODE bestNode = generateRootNode(inputVRPTW, parameter);
	try {
		multiset<BBNODE> nodes{ bestNode };
		while (!nodes.empty()) {
			auto worker = *nodes.begin();
			nodes.erase(nodes.begin());
			worker.solution.UB_Integer_Value = bestNode.solution.UB_Integer_Value;
			worker.solve(output);

			// Whether the LP corresponding to this node is Linearly feasible.
			if (worker.solution.feasible) {
				// Whether the optimal solution corresponding to this node is an integer solution.
				if (worker.solution.integer) {
					if (lessThanReal(worker.solution.objective, bestNode.solution.objective, PPM)) {
						bestNode = worker;
					}
				}
				// Whether the lower bound corresponding to this node is less than the best upper bound found so far.
				else if (lessThanReal(worker.solution.objective, worker.solution.UB_Integer_Value, PPM)) {
					// Branch.
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("BPAlgorithm", exc);
	}
	return bestNode;
}

