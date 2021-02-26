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
	return make_pair(index > -1, index);
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
	return make_tuple(pre > -1, pre, suc);
}


BBNODE childNode(const Parameter_BP& parameter, const BBNODE& rhs) {
	BBNODE child(rhs);
	++child.depth;
	child.setPriority(parameter.weightLB, parameter.weightDepth);
	child.solution.explored = child.solution.feasible = child.solution.integer = false;
	child.model.clearColumns();
	return child;
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
	}
	catch (const exception& exc) {
		printErrorAndExit("BPAlgorithm", exc);
	}
	return bestNode;
}


void testTOPTW_BP(const string& strInput, const Parameter_BP& parameter) {
	try {
		Data_Input_VRPTW inputVRPTW;
		inputVRPTW.constrainResource = { false,false,true };
		readFromFileVRPTW(inputVRPTW, strInput);
		inputVRPTW.preprocess();

		const auto bestNode = BPAlgorithm(inputVRPTW, parameter, cout);
		cout << bestNode.solution.explored << '\t' << bestNode.solution.feasible << '\t' << bestNode.solution.integer << '\t'
			<< bestNode.solution.objective << '\t' << bestNode.solution.UB_Integer_Value << '\t';
		for (const auto& route : bestNode.solution.UB_Integer_Solution) {
			cout << "[ ";
			for (const auto& elem : route.getPath()) {
				cout << elem << ' ';
			}
			cout << "]. ";
		}
		cout << endl;
	}
	catch (const exception& exc) {
		printErrorAndExit("testTOPTW_BP", exc);
	}
}

