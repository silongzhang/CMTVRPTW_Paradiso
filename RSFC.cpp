#include"RSFC.h"


vector<Label_TimePath> linearize(const Map_Label_TimePath& structures) {
	vector<Label_TimePath> result;
	try {
		for (const auto& elem : structures.getMpVisitedLabels()) {
			for (const auto& structure : elem.second) {
				result.push_back(structure);
			}
		}
		if (result.size() != structures.getSize()) throw exception();
	}
	catch (const exception& exc) {
		printErrorAndExit("linearize", exc);
	}
	return result;
}


vector<double> linearize(const set<double, timeSortCriterion>& tms) {
	return vector<double>(tms.begin(), tms.end());
}


vector<tuple<int, int, int>> linearize(const set<tuple<int, int, int>>& tps) {
	return vector<tuple<int, int, int>>(tps.begin(), tps.end());
}


void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x) {
	try {
		auto env = model.getEnv();
		IloExpr expr(env);
		for (int i = 0; i < structures.size(); ++i) {
			expr += structures[i].getRealCost() * x[i];
		}
		model.add(IloMinimize(env, expr));
		expr.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("setObjective", exc);
	}
}


void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x, 
	IloRangeArray rangePartition) {
	try {
		auto env = model.getEnv();
		for (int i = 1; i < input.NumVertices; ++i) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].hasVisited(i)) {
					expr += x[s];
				}
			}
			model.add(expr == 1);
			rangePartition.add(expr == 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsPartition", exc);
	}
}


void addConstraintsActive(outputRSFC& output, const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const vector<double>& additionalTimes, IloRangeArray rangeActive) {
	try {
		auto env = model.getEnv();
		for (const auto t : additionalTimes) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].strongActive(t)) {
					expr += x[s];
				}
			}
			model.add(expr <= input.MaxNumVehicles);
			rangeActive.add(expr <= input.MaxNumVehicles);
			output.times.push_back(t);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsActive", exc);
	}
}


void addConstraintsSR(outputRSFC& output, const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x,
	const vector<tuple<int, int, int>>& additionalTriplets, IloRangeArray rangeSR) {
	try {
		auto env = model.getEnv();
		for (const auto& triplet : additionalTriplets) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].atLeastTwo(triplet)) {
					expr += x[s];
				}
			}
			model.add(expr <= 1);
			rangeSR.add(expr <= 1);
			output.triplets.push_back(triplet);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("addConstraintsSR", exc);
	}
}


vector<int> getPositiveStructures(const IloCplex& cplex, const IloNumVarArray& x) {
	vector<int> result;
	try {
		for (int i = 0; i < x.getSize(); ++i) {
			if (greaterThanReal(cplex.getValue(x[i]), 0, PPM)) {
				result.push_back(i);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("getPositiveStructures", exc);
	}
	return result;
}


set<double, timeSortCriterion> detectAdditionalTimes(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, 
	const set<double, timeSortCriterion>& times, const IloCplex& cplex, const IloNumVarArray& x) {
	set<double, timeSortCriterion> result;
	try {
		auto positive = getPositiveStructures(cplex, x);
		set<double, timeSortCriterion> potential;

		for (const auto s : positive) {
			auto tm = structures[s].getTimeAttribute().getLatestDeparture();
			if (times.find(tm) == times.end() && potential.find(tm) == potential.end()) {
				potential.insert(tm);
			}
		}

		auto env = cplex.getEnv();
		for (const auto tm : potential) {
			IloExpr expr(env);
			for (const auto s : positive) {
				if (structures[s].strongActive(tm)) {
					expr += x[s];
				}
			}
			if (greaterThanReal(cplex.getValue(expr), input.MaxNumVehicles, PPM)) {
				result.insert(tm);
			}
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("detectAdditionalTimes", exc);
	}
	return result;
}


set<tuple<int, int, int>> detectAdditionalTriplets(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures,
	const set<tuple<int, int, int>>& triplets, const IloCplex& cplex, const IloNumVarArray& x) {
	set<tuple<int, int, int>> result;
	try {
		auto positive = getPositiveStructures(cplex, x);
		set<tuple<int, int, int>> existing;

		for (const auto s : positive) {
			for (const auto& tp : structures[s].getTuples(1, input.NumVertices)) {
				if (triplets.find(tp) == triplets.end()) {
					if (existing.find(tp) == existing.end()) {
						existing.insert(tp);
					}
					else {
						result.insert(tp);
					}
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("detectAdditionalTriplets", exc);
	}
	return result;
}


void RSFC(outputRSFC& output, const Data_Input_VRPTW& input, const Map_Label_TimePath& stt) {
	IloEnv env;
	try {
		output.structures.clear(); output.times.clear(); output.triplets.clear();
		output.structures = linearize(stt);
		set<double, timeSortCriterion> timeSet;
		set<tuple<int, int, int>> tripletSet;

		// Define the variables.
		IloNumVarArray x(env, output.structures.size(), 0, IloInfinity);

		// Define the model.
		IloModel model(env);
		IloRangeArray rangePartition(env), rangeActive(env), rangeSR(env);
		setObjective(output.structures, model, x);
		setConstraintsPartition(input, output.structures, model, x, rangePartition);

		// Solve the model.
		IloCplex cplex(model);
		for (int i = 1; true; ++i) {
			env.out() << "solve the RSFC model for the " << i << "th time." << endl;
			cplex.solve();
			auto additionalTimes = detectAdditionalTimes(input, output.structures, timeSet, cplex, x);
			for (const auto& elem : additionalTimes) timeSet.insert(elem);
			auto additionalTriplets = detectAdditionalTriplets(input, output.structures, tripletSet, cplex, x);
			for (const auto& elem : additionalTriplets) tripletSet.insert(elem);
			
			if (additionalTimes.empty() && additionalTriplets.empty()) {
				env.out() << "No additional time slots or triplets should be added into the RSFC model." << endl;
				break;
			}
			else {
				addConstraintsActive(output, input, output.structures, model, x, linearize(additionalTimes), rangeActive);
				env.out() << "The number of added time slots (additional / total): " << additionalTimes.size() << " / " << output.times.size() << endl;
				addConstraintsSR(output, input, output.structures, model, x, linearize(additionalTriplets), rangeSR);
				env.out() << "The number of added triplets (additional / total): " << additionalTriplets.size() << " / " << output.triplets.size() << endl;
			}
		}

		env.out() << "solution status is " << cplex.getStatus() << endl;
		env.out() << "solution value  is " << cplex.getObjValue() << endl;
		output.objective = cplex.getObjValue();

		// Get values of dual variables.
		output.dualPartition.clear(); output.dualActive.clear(); output.dualSR.clear();
		IloNumArray dual(env);
		cplex.getDuals(dual, rangePartition);
		output.dualPartition.push_back(0);
		for (int i = 0; i < dual.getSize(); ++i) output.dualPartition.push_back(dual[i]);

		cplex.getDuals(dual, rangeActive);
		for (int i = 0; i < dual.getSize(); ++i) output.dualActive.push_back(dual[i]);

		cplex.getDuals(dual, rangeSR);
		for (int i = 0; i < dual.getSize(); ++i) {
			output.dualSR.push_back(dual[i]);
			output.mapDualSR.insert(make_pair(output.triplets[i], dual[i]));
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("RSFC", exc);
	}
	env.end();
}


vector<Label_TimePath> StructureReduction(const Data_Input_VRPTW& input, const outputRSFC& rsfc, const double reducedCostGap) {
	vector<Label_TimePath> result;
	try {
		for (const auto& elem : rsfc.structures) {
			if (!greaterThanReal(elem.getReducedCostRSFC(input, rsfc), reducedCostGap, PPM)) {
				result.push_back(elem);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("StructureReduction", exc);
	}
	return result;
}


void testUntilStructureReduction() {
	try {
		Data_Input_VRPTW inputVRPTW;
		inputVRPTW.constrainResource = { true,false,true };

		string outFile = "data//CMTVRPTW//Test//testUntilStructureReduction.txt";
		ofstream os(outFile);
		if (!os) throw exception();
		os << "Name" << '\t' << "NumVertices" << '\t' << "Capacity" << '\t' << "Density" << '\t'
			<< "Lower bound (VRPTW)" << '\t' << "Running Time (s)" << '\t' << "NumStructures" << '\t' << "Running Time (s)" << '\t'
			<< "Lower bound (RSFC)" << '\t' << "Running Time (s)" << '\t' << "NumStructures" << '\t' << "Running Time (s)" << endl;

		vector<string> folders = { "data//CMTVRPTW//Solomon Type 2 - 25//",
			"data//CMTVRPTW//Solomon Type 2 - 40//",
			"data//CMTVRPTW//Solomon Type 2 - 50//" };
		vector<string> names;
		getFiles(folders[0], vector<string>(), names);

		clock_t last = clock();
		for (const auto& folder : folders) {
			for (const auto& name : names) {
				string strInput = folder + name;
				readFromFileVRPTW(inputVRPTW, strInput);
				inputVRPTW.preprocess();
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "Instance: " << inputVRPTW.name << '\t' << "NumVertices: " << inputVRPTW.NumVertices << '\t' << "Time: " << runTime(last) << endl;
				os << inputVRPTW.name << '\t' << inputVRPTW.NumVertices << '\t' << inputVRPTW.capacity << '\t' << inputVRPTW.density << '\t';

				last = clock();
				Solution_VRPTW_CG solForVRPTWCG = lbAtCGRootNodeVRPTW(inputVRPTW);
				double timeForVRPTWCG = runTime(last);
				os << solForVRPTWCG.getCost() << '\t' << timeForVRPTWCG << '\t';

				last = clock();
				double gapGuess = 0.05;
				double ubGuess = solForVRPTWCG.getCost() * (1 + gapGuess);
				Map_Label_TimePath resultForEnumeration = EnumerationStructure(solForVRPTWCG.getInput(), ubGuess - solForVRPTWCG.getCost());
				double timeForEnumeration = runTime(last);
				os << resultForEnumeration.getSize() << '\t' << timeForEnumeration << '\t';

				last = clock();
				outputRSFC rsfc;
				RSFC(rsfc, inputVRPTW, resultForEnumeration);
				double timeForRSFC = runTime(last);
				os << rsfc.objective << '\t' << timeForRSFC << '\t';

				last = clock();
				vector<Label_TimePath> structuresReduced = StructureReduction(inputVRPTW, rsfc, ubGuess - rsfc.objective);
				double timeForStructureReduction = runTime(last);
				os << structuresReduced.size() << '\t' << timeForStructureReduction << endl;
			}
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("testVRPTWCG", exc);
	}
}


