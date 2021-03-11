#include"CuttingPlane.h"


void setObjectiveCuttingPlane(const vector<Label_TimePath>& structures, IloModel model, TypeX x) {
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


void setConstraintsPartition(const Data_Input_VRPTW& input, const vector<Label_TimePath>& structures, IloModel model, TypeX x) {
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
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsPartition", exc);
	}
}


ILOUSERCUTCALLBACK3(TimeUserCut, const Parameter_CuttingPlane&, parameter, TypeX, x, Solution_CuttingPlane&, solution) {
	try {
		// Get positive structures.
		vector<int> positive;
		for (int i = 0; i < x.getSize(); ++i) {
			if (greaterThanReal(getValue(x[i]), 0, PPM)) {
				positive.push_back(i);
			}
		}

		// Get the set of latest departure times.
		set<double, timeSortCriterion> tails;
		for (const auto s : positive) {
			auto tm = parameter.columnPool[s].getTimeAttribute().getLatestDeparture();
			if (tails.find(tm) == tails.end()) {
				tails.insert(tm);
			}
		}

		// Get the set of times where RSFC constraints are violated.
		set<double, timeSortCriterion> times;
		auto env = getEnv();
		for (const auto tm : tails) {
			IloExpr expr(env);
			for (const auto s : positive) {
				if (parameter.columnPool[s].strongActive(tm)) {
					expr += x[s];
				}
			}
			if (greaterThanReal(getValue(expr), parameter.input_VRPTW.MaxNumVehicles, PPM)) {
				times.insert(tm);
			}
			expr.end();
		}

		// Add RSFC constraints.
		for (const auto t : times) {
			solution.timeSet.insert(t);

			IloExpr expr(env);
			for (int s = 0; s < parameter.columnPool.size(); ++s) {
				if (parameter.columnPool[s].strongActive(t)) {
					expr += x[s];
				}
			}
			add(expr <= parameter.input_VRPTW.MaxNumVehicles);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TimeUserCut", exc);
	}
}


ILOUSERCUTCALLBACK3(TripletUserCut, const Parameter_CuttingPlane&, parameter, TypeX, x, Solution_CuttingPlane&, solution) {
	try {
		// Get positive structures.
		vector<int> positive;
		for (int i = 0; i < x.getSize(); ++i) {
			if (greaterThanReal(getValue(x[i]), 0, PPM)) {
				positive.push_back(i);
			}
		}

		// Get the set of triplets where SR constraints are violated.
		set<tuple<int, int, int>> existing, triplets;
		for (const auto s : positive) {
			for (const auto& tp : parameter.columnPool[s].getTuples(1, parameter.input_VRPTW.NumVertices)) {
				if (existing.find(tp) == existing.end()) {
					existing.insert(tp);
				}
				else {
					triplets.insert(tp);
				}
			}
		}

		// Add SR constraints.
		auto env = getEnv();
		for (const auto& triplet : triplets) {
			solution.tripletSet.insert(triplet);

			IloExpr expr(env);
			for (int s = 0; s < parameter.columnPool.size(); ++s) {
				if (parameter.columnPool[s].atLeastTwo(triplet)) {
					expr += x[s];
				}
			}
			add(expr <= 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("TripletUserCut", exc);
	}
}


Data_Input_VRPTW constructDataVRPTW(const Parameter_CuttingPlane& parameter, const vector<Label_TimePath>& selectedStructures) {
	Data_Input_VRPTW inputVRPTW;
	try {
		inputVRPTW.NumVertices = selectedStructures.size() + 1;
		inputVRPTW.clearAndResize();

		inputVRPTW.name = "Constructed TOPTW";
		inputVRPTW.MaxNumVehicles = parameter.input_VRPTW.MaxNumVehicles;
		inputVRPTW.capacity = InfinityPos;
		inputVRPTW.constrainResource = { false,false,true };

		for (int i = 0; i < inputVRPTW.NumVertices; ++i) {
			inputVRPTW.QuantityWindow[i] = make_pair(0, InfinityPos);
			inputVRPTW.DistanceWindow[i] = make_pair(0, InfinityPos);
			for (int j = 0; j < inputVRPTW.NumVertices; ++j) {
				inputVRPTW.Quantity[i][j] = 0;
				inputVRPTW.Distance[i][j] = 0;
			}
		}

		set<double> early, late;
		for (int i = 1; i < inputVRPTW.NumVertices; ++i) {
			inputVRPTW.TimeWindow[i].first = selectedStructures[i - 1].getTimeAttribute().getEarliestDeparture();
			early.insert(inputVRPTW.TimeWindow[i].first);
			inputVRPTW.TimeWindow[i].second = selectedStructures[i - 1].getTimeAttribute().getLatestDeparture();
			late.insert(inputVRPTW.TimeWindow[i].second + selectedStructures[i - 1].getTimeAttribute().getDuration());
		}
		inputVRPTW.TimeWindow[0].first = *early.begin() - One;
		inputVRPTW.TimeWindow[0].second = *prev(late.end()) + One;

		for (int j = 1; j < inputVRPTW.NumVertices; ++j) {
			inputVRPTW.Time[0][j] = 0;
			inputVRPTW.RealCost[0][j] = 0;
			inputVRPTW.ExistingArcs[0][j] = true;
		}

		for (int i = 1; i < inputVRPTW.NumVertices; ++i) {
			for (int j = 0; j < inputVRPTW.NumVertices; ++j) {
				double e_i = selectedStructures[i - 1].getTimeAttribute().getEarliestDeparture();
				double d_i = selectedStructures[i - 1].getTimeAttribute().getDuration();
				double l_j = selectedStructures[j - 1].getTimeAttribute().getLatestDeparture();
				if (i != j && e_i + d_i <= l_j) {
					inputVRPTW.Time[i][j] = d_i;
					inputVRPTW.RealCost[i][j] = 1;
					inputVRPTW.ExistingArcs[i][j] = true;
				}
				else {
					inputVRPTW.Time[i][j] = InfinityPos;
					inputVRPTW.RealCost[i][j] = 0;
					inputVRPTW.ExistingArcs[i][j] = false;
				}
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("constructDataVRPTW", exc);
	}
	inputVRPTW.preprocess();
	return inputVRPTW;
}


ILOLAZYCONSTRAINTCALLBACK3(CoexistLazyConstraint, const Parameter_CuttingPlane&, parameter, TypeX, X, Solution_CuttingPlane&, solution) {
	try {
		// Check whether X is an integer vector.
		for (int i = 0; i < X.getSize(); ++i) {
			if (!equalToReal(getValue(X[i]), 0, PPM) && !equalToReal(getValue(X[i]), 1, PPM)) throw exception();
		}

		// Get selected structures.
		vector<int> selected;
		for (int i = 0; i < X.getSize(); ++i) {
			if (equalToReal(getValue(X[i]), 1, PPM)) {
				selected.push_back(i);
			}
		}
		vector<Label_TimePath> selectedStructures;
		for (auto i : selected) {
			selectedStructures.push_back(parameter.columnPool[i]);
		}

		// Solve the TOPTW determined by selected structures.		
		Data_Input_VRPTW input_TOPTW = constructDataVRPTW(parameter, selectedStructures);
		Parameter_BP parameter_BP;
		parameter_BP.weightLB = parameter_BP.weightDepth = 1;
		parameter_BP.allowPrintLog = false;
		BBNODE solTOPTW = BPAlgorithm(input_TOPTW, parameter_BP, cout);
		if (!solTOPTW.solution.feasible || !solTOPTW.solution.integer) throw exception();

		// Add lazy constraints.
		double numCoexist = -solTOPTW.solution.objective;
		if (greaterThanReal(selected.size(), numCoexist, PPM)) {
			solution.SFCSet.push_back(make_pair(selected, numCoexist));

			IloExpr expr(getEnv());
			for (auto i : selected) {
				expr += X[i];
			}
			add(expr <= numCoexist);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("CoexistLazyConstraint", exc);
	}
}


vector<Label_TimePath> getSolutionCuttingPlaneAlgorithm(const Parameter_CuttingPlane& parameter, const IloCplex& cplex, const TypeX& X) {
	vector<Label_TimePath> selectedStructures;
	try {
		for (int i = 0; i < X.getSize(); ++i) {
			if (equalToReal(cplex.getValue(X[i]), 1, PPM)) {
				selectedStructures.push_back(parameter.columnPool[i]);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("getSolutionBCAlgorithm", exc);
	}
	return selectedStructures;
}


ILOBRANCHCALLBACK3(BranchCallBack, const Parameter_CuttingPlane&, parameter, TypeX, X, Solution_CuttingPlane&, solution) {
	try {
		if (isIntegerFeasible()) prune();

		// Get positive structures.
		vector<int> positive;
		for (int i = 0; i < X.getSize(); ++i) {
			if (greaterThanReal(getValue(X[i]), 0, PPM)) {
				positive.push_back(i);
			}
		}

		// Get the accumulative number of structures which traverse the arc.
		vector<vector<double>> traversal(parameter.input_VRPTW.NumVertices, vector<double>(parameter.input_VRPTW.NumVertices, 0));
		for (auto i : positive) {
			auto path = parameter.columnPool[i].getPath();
			auto pre = path.begin();
			for (auto suc = pre + 1; suc != path.end(); ++pre, ++suc) {
				traversal[*pre][*suc] += getValue(X[i]);
			}
		}

		// Get the arc on which the branch will be conducted.
		int tail = -1, head = -1;
		double midDist = InfinityPos;
		for (int i = 0; i < traversal.size(); ++i) {
			for (int j = 0; j < traversal[i].size(); ++j) {
				if (greaterThanReal(traversal[i][j], 0, PPM) && greaterThanReal(midDist, abs(traversal[i][j] - 0.5), PPM)) {
					tail = i, head = j;
					midDist = abs(traversal[i][j] - 0.5);
				}
			}
		}
		if (tail == -1) throw exception();

		// Get the set of structures which traverse the arc.
		vector<int> visitTheArc;
		for (int i = 0; i < parameter.columnPool.size(); ++i) {
			if (parameter.columnPool[i].hasVisitedArc(tail, head)) {
				visitTheArc.push_back(i);
			}
		}

		// Branch.
		IloExpr expr(getEnv());
		for (const auto i : visitTheArc) {
			expr += X[i];
		}
		makeBranch(expr == 0, getObjValue());
		makeBranch(expr == 1, getObjValue());
		++solution.numBranch;
		expr.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("branchCallBack", exc);
	}
}


Solution_CuttingPlane CuttingPlaneAlgorithm(const Parameter_CuttingPlane& parameter) {
	Solution_CuttingPlane solution;
	solution.numBranch = 0;

	IloEnv env;
	try {
		cout << "Begin running the procedure titled CuttingPlaneAlgorithm." << endl;
		clock_t start = clock();

		// Define the model.
		IloModel model(env);
		TypeX X(env, parameter.columnPool.size());
		setObjectiveCuttingPlane(parameter.columnPool, model, X);
		setConstraintsPartition(parameter.input_VRPTW, parameter.columnPool, model, X);

		// Solve the model.
		IloCplex cplex(model);
		cplex.use(TimeUserCut(env, parameter, X, solution));
		cplex.use(TripletUserCut(env, parameter, X, solution));
		cplex.use(CoexistLazyConstraint(env, parameter, X, solution));
		cplex.use(BranchCallBack(env, parameter, X, solution));
		cplex.solve();

		// Get the solution.
		if (cplex.getCplexStatus() == IloCplex::CplexStatus::Infeasible) {
			solution.status = OptimalityStatus::Infeasible;
			solution.objective = InfinityPos;
		}
		else if (cplex.getCplexStatus() == IloCplex::CplexStatus::Optimal) {
			solution.status = OptimalityStatus::Optimal;
			solution.objective = cplex.getObjValue();
			solution.routes = getSolutionCuttingPlaneAlgorithm(parameter, cplex, X);
		}
		else throw exception();

		// Print information.
		cout << "Time: " << runTime(start) << '\t' << "Status: " << cplex.getCplexStatus() << endl;
		if (cplex.getCplexStatus() == IloCplex::CplexStatus::Optimal) {
			cout << "Objective: " << cplex.getObjValue() << endl;
		}
		cout << "TimeUserCut: " << solution.timeSet.size() << '\t' << "TripletUserCut: " << solution.tripletSet.size() << '\t'
			<< "CoexistLazyConstraint: " << solution.SFCSet.size() << '\t' << "BranchCallBack: " << solution.numBranch << endl;
		cout << "The procedure titled CuttingPlaneAlgorithm is finished." << endl;
	}
	catch (const exception& exc) {
		printErrorAndExit("CuttingPlaneAlgorithm", exc);
	}
	env.end();

	return solution;
}

