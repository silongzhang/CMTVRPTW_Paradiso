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
			UB_Guess = solution.status == OptimalityStatus::Feasible ? solution.objective : (1 + gapGuess) * LB_1;
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
			Parameter_BC parameter_BC;
			parameter_BC.input_VRPTW = parameter.input_VRPTW;
			parameter_BC.columnPool = resultReduction;
			Solution_BC resultBC = BCAlgorithm(parameter_BC);
			cout << "######################################" << endl << endl;

			if (resultBC.status == OptimalityStatus::Optimal) {
				solution.objective = resultBC.objective;
				solution.routes = resultBC.routes;
				solution.status = greaterThanReal(resultBC.objective, UB_Guess, PPM) ? OptimalityStatus::Feasible : OptimalityStatus::Optimal;
			}
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
		parameter.gapIncre = 0.05;

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

