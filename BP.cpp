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
		rootParameter.branchOnVehicleNumber = make_pair(inputVRPTW.MaxNumVehicles, false);
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


// Whether the input number is an integer.
bool isInteger_0(double num) {
	return equalToReal(num, floor(num), PPM) || equalToReal(num, ceil(num), PPM);
}


// For a given non-integer number x.y, return the number x.5.
double middle(double num) {
	try {
		if (isInteger_0(num)) throw exception();
	}
	catch (const exception& exc) {
		printErrorAndExit("middle", exc);
	}
	auto low = floor(num), high = ceil(num);
	return double(low + high) / 2;
}


// Whether the input vector is an integer vector.
// If not, return the index corresponding to the element which is closest to its middle.
pair<bool, int> isInteger_1(const vector<double>& num) {
	int index = -1;
	double distance = 1;
	for (int i = 0; i < num.size(); ++i) {
		if (!isInteger_0(num[i]) && distance > abs(num[i] - middle(num[i]))) {
			index = i;
			distance = abs(num[i] - middle(num[i]));
		}
	}
	return make_pair(index == -1, index);
}


// Whether the input matrix is an integer matrix.
// If not, return the index pair (i, j) corresponding to the element which is closest to its middle.
tuple<bool, int, int> isInteger_2(const vector<vector<double>>& num) {
	int pre = -1, suc = -1;
	double distance = 1;
	for (int i = 0; i < num.size(); ++i) {
		for (int j = 0; j < num[i].size(); ++j) {
			if (!isInteger_0(num[i][j]) && distance > abs(num[i][j] - middle(num[i][j]))) {
				pre = i, suc = j;
				distance = abs(num[i][j] - middle(num[i][j]));
			}
		}
	}
	return make_tuple(pre == -1, pre, suc);
}


BBNODE childNode(const Parameter_BP& parameter, const BBNODE& rhs) {
	BBNODE child(rhs);
	++child.depth;
	child.setPriority(parameter.weightLB, parameter.weightDepth);
	child.solution.explored = child.solution.feasible = child.solution.integer = false;
	child.model.clearColumns();
	return child;
}


void printBranchParameter(const BBNODE& worker) {
	cout << "Branch on the number of vehicles: number(" << worker.parameter.branchOnVehicleNumber.first << "), bool("
		<< worker.parameter.branchOnVehicleNumber.second << ")" << endl;
	cout << "The number of available vehicles: " << worker.parameter.input_VRPTW.MaxNumVehicles << endl;

	if (!worker.parameter.branchOnVertices.empty()) {
		cout << "Branch on vertices: " << endl;
		for (const auto& elem : worker.parameter.branchOnVertices) {
			cout << elem.first << ": " << elem.second << '\t';
		}
		cout << endl;
	}

	if (!worker.parameter.branchOnArcs.empty()) {
		cout << "Branch on arcs: " << endl;
		for (const auto& elem : worker.parameter.branchOnArcs) {
			cout << "[(" << elem.first.first << ", " << elem.first.second << "): " << elem.second << "]" << '\t';
		}
		cout << endl;
	}
}


BBNODE BPAlgorithm(const Data_Input_VRPTW& inputVRPTW, const Parameter_BP& parameter, ostream& output) {
	string strLog = "Begin running the procedure titled BPAlgorithm.";
	print(parameter.allowPrintLog, output, strLog);
	clock_t start = clock();

	Info_BP info;
	info.prunedInfeasibility = info.prunedInteger = info.prunedBound = info.branched = 0;

	BBNODE bestNode = generateRootNode(inputVRPTW, parameter);
	bestNode.parameter.allowPrintLog = false;
	try {
		multiset<BBNODE> nodes{ bestNode };
		int iter = 0;
		while (!nodes.empty()) {
			auto worker = *nodes.begin();
			nodes.erase(nodes.begin());
			worker.solution.UB_Integer_Value = bestNode.solution.UB_Integer_Value;

			strLog = "\nTime: " + numToStr(runTime(start)) + "\t" + "Iteration: " + numToStr(++iter) + "\t"
				+ "Depth: " + numToStr(worker.depth) + "\t" + "Best upper bound: " + numToStr(bestNode.solution.UB_Integer_Value) + "\t"
				+ "Remaining nodes: " + numToStr(nodes.size() + 1) + "\t" + "prunedInfeasibility: " + numToStr(info.prunedInfeasibility) + "\t"
				+ "prunedInteger: " + numToStr(info.prunedInteger) + "\t" + "prunedBound: " + numToStr(info.prunedBound) + "\t"
				+ "branched: " + numToStr(info.branched);
			print(parameter.allowPrintLog, output, strLog);
			printBranchParameter(worker);

			worker.solve(output);

			if (!worker.solution.feasible) {
				++info.prunedInfeasibility;
			}
			else {
				// Whether the LP corresponding to this node is Linearly feasible.
				strLog = "Objective: " + numToStr(worker.solution.objective);
				print(parameter.allowPrintLog, output, strLog);

				// Whether the optimal solution corresponding to this node is an integer solution.
				if (worker.solution.integer) {
					++info.prunedInteger;
					if (lessThanReal(worker.solution.objective, bestNode.solution.UB_Integer_Value, PPM)) {
						bestNode = worker;
					}
				}
				else if (!lessThanReal(worker.solution.objective, worker.solution.UB_Integer_Value, PPM)) {
					++info.prunedBound;
				}
				else {
					// The lower bound corresponding to this node is less than the best upper bound found so far.
					// Branch.
					++info.branched;
					BBNODE left = childNode(parameter, worker), right = childNode(parameter, worker);

					const auto isIntegerVertex = isInteger_1(worker.solution.visitVertices);
					const auto isIntegerArc = isInteger_2(worker.solution.visitArcs);

					if (!isInteger_0(worker.solution.numVehicles)) {
						// Branch on the number of vehicles.
						left.parameter.branchOnVehicleNumber = make_pair(int(floor(worker.solution.numVehicles)), false);
						right.parameter.branchOnVehicleNumber = make_pair(int(ceil(worker.solution.numVehicles)), true);
					}
					else if (!isIntegerVertex.first) {
						// Branch on the vertex.
						left.parameter.branchOnVertices.insert(make_pair(isIntegerVertex.second, false));
						right.parameter.branchOnVertices.insert(make_pair(isIntegerVertex.second, true));
					}
					else if (!get<0>(isIntegerArc)) {
						// Branch on the arc.
						auto arc = make_pair(get<1>(isIntegerArc), get<2>(isIntegerArc));
						left.parameter.branchOnArcs.insert(make_pair(arc, false));
						right.parameter.branchOnArcs.insert(make_pair(arc, true));
					}
					else {
						throw exception();
					}

					// Insert child nodes to the set of nodes.
					nodes.insert(left);
					nodes.insert(right);
				}
			}
		}

		strLog = "\nTime: " + numToStr(runTime(start)) + "\t" + "Best upper bound: " + numToStr(bestNode.solution.UB_Integer_Value) + "\t"
			+ "Remaining nodes: " + numToStr(nodes.size()) + "\t" + "prunedInfeasibility: " + numToStr(info.prunedInfeasibility) + "\t"
			+ "prunedInteger: " + numToStr(info.prunedInteger) + "\t" + "prunedBound: " + numToStr(info.prunedBound) + "\t" 
			+ "branched: " + numToStr(info.branched);
		print(parameter.allowPrintLog, output, strLog);
	}
	catch (const exception& exc) {
		printErrorAndExit("BPAlgorithm", exc);
	}
	return bestNode;
}


BBNODE TOPTW_BP(const string& strInput, const Parameter_BP& parameter) {
	Data_Input_VRPTW inputVRPTW;
	inputVRPTW.constrainResource = { false,false,true };
	readFromFileVRPTW(inputVRPTW, strInput);
	inputVRPTW.preprocess();

	return BPAlgorithm(inputVRPTW, parameter, cout);
}


void testTOPTW_BP() {
	try {
		string outFile = "data//CMTVRPTW//Test//TOPTW//Output//testTOPTW_BP.txt";
		ofstream os(outFile);
		if (!os) throw exception();

		vector<string> folders = { "data//CMTVRPTW//Test//TOPTW//Input//Solomon Type 2 - 25//",
			"data//CMTVRPTW//Test//TOPTW//Input//Solomon Type 2 - 40//",
			"data//CMTVRPTW//Test//TOPTW//Input//Solomon Type 2 - 50//" };
		vector<string> names;
		getFiles(folders[0], vector<string>(), names);

		Parameter_BP parameter;
		parameter.weightDepth = 1;
		parameter.weightLB = 1;
		parameter.allowPrintLog = true;

		clock_t last = clock();
		for (const auto& folder : folders) {
			for (const auto& name : names) {
				if (folder == "data//CMTVRPTW//Solomon Type 2 - 50//" && name[0] == 'R' && name[1] == '2') {
					continue;
				}
				string strInput = folder + name;

				last = clock();
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "Instance: " << strInput << endl;
				auto bestNode = TOPTW_BP(strInput, parameter);

				os << bestNode.parameter.input_VRPTW.name << '\t' << bestNode.parameter.input_VRPTW.NumVertices << '\t'
					<< bestNode.parameter.input_VRPTW.density << '\t' << runTime(last) << '\t';
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
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("testTOPTW_BP", exc);
	}
}

