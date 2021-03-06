#include"CMTVRPTW.h"


Solution_OPRE_2019_1874 Framework_OPRE_2019_1874::solve(const Parameter_OPRE_2019_1874& parameter) {
	Solution_OPRE_2019_1874 solution;
	solution.status = Status::Infeasible;
	solution.objective = InfinityPos;
	try {
		clock_t start = clock();
		gapGuess = parameter.gapInit;

		while (solution.status != Status::Optimal) {
			LB_1 = lbAtCGRootNodeVRPTW(parameter.input_VRPTW).getCost();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Framework_OPRE_2019_1874::solve", exc);
	}
	return solution;
}

