#include"Enumeration_TimePath.h"


// Renew this object after extending from vertex i to vertex j.
void Time_Attribute::extend(const Data_Input_ESPPRC &data, const TimeType currentTime, const int i, const int j) {
	try {
		TimeType nextTime = currentTime + data.Time[i][j];
		nextTime = max(nextTime, data.TimeWindow[j].first);

		TimeType aheadArrival = max(TimeType(0), data.TimeWindow[j].first - (currentTime + data.Time[i][j]));
		TimeType delayDeparture = min(buffer, aheadArrival);
		earliestDeparture += delayDeparture;
		duration = nextTime - earliestDeparture;
		buffer -= delayDeparture;
		buffer = min(buffer, data.TimeWindow[j].second - nextTime);
		latestDeparture = earliestDeparture + buffer;

		if (lessThanReal(buffer, 0, PPM)) throw exception();
	}
	catch (const exception &exc) {
		printErrorAndExit("Time_Attribute::extend", exc);
	}
}


bool Time_Attribute::dominate(const Time_Attribute& rhs) const {
	return latestDeparture >= rhs.latestDeparture && earliestDeparture <= rhs.earliestDeparture && duration <= rhs.duration;
}


bool Label_TimePath::strongActive(const double t) const {
	return lessThanReal(timeAttribute.getLatestDeparture() - 2 * PPM, t, PPM) &&
		lessThanReal(t, timeAttribute.getEarliestDeparture() + timeAttribute.getDuration(), PPM);
}


bool Label_TimePath::atLeastTwo(const tuple<int, int, int>& tp) const {
	return int(visited.test(get<0>(tp))) + int(visited.test(get<1>(tp))) + int(visited.test(get<2>(tp))) >= 2;
}


// Extend this lable to vertex j.
void Label_TimePath::extend(const Data_Input_ESPPRC &data, const int j) {
	try {
		if (visited.test(j)) throw exception();
		visited.set(j, true);

		timeAttribute.extend(data, getTime(), tail, j);

		Label_ESPPRC::extend(data, j);
	}
	catch (const exception &exc) {
		printErrorAndExit("Label_TimePath::extend", exc);
	}
}


// Check whether this label is a feasible route.
bool Label_TimePath::feasible(const Data_Input_ESPPRC &data) const {
	try {
		TimeType ed = timeAttribute.getEarliestDeparture();
		TimeType ld = timeAttribute.getLatestDeparture();

		auto prBeforeED = getFeasibilityAndConsumption(data, ed - One);
		auto prED = getFeasibilityAndConsumption(data, ed);
		auto prLD = getFeasibilityAndConsumption(data, ld);
		auto prAfterLD = getFeasibilityAndConsumption(data, ld + One);

		if (prBeforeED.first && lessThanReal(prBeforeED.second.getDurationTime(), prED.second.getDurationTime(), PPM)) return false;
		if (!prED.first || !prLD.first || !equalToReal(prED.second.getDurationTime(), prLD.second.getDurationTime(), PPM)) return false;
		if (prAfterLD.first) return false;
	}
	catch (const exception &exc) {
		printErrorAndExit("Label_TimePath::feasible", exc);
	}
	return true;
}


// Constructor.
Label_TimePath::Label_TimePath(const Data_Input_ESPPRC &data, const int origin, const Consumption_ESPPRC &csp, const Cost_ESPPRC &cst) : 
	Label_ESPPRC(data, origin, csp, cst) {
	try {
		visited.reset();
		if (origin != 0) visited.set(origin, true);

		timeAttribute = Time_Attribute(0, csp.getTime(), csp.getTime(), data.TimeWindow[origin].second - csp.getTime());
	}
	catch (const exception &exc) {
		printErrorAndExit("Label_TimePath::Label_TimePath", exc);
	}
}


bool Label_TimePath::dominate(const Label_TimePath& rhs) const {
	if (tail != rhs.tail || visited != rhs.visited) return false;
	return getReducedCost() <= rhs.getReducedCost() && timeAttribute.dominate(rhs.getTimeAttribute());
}


// Try to insert a Label_TimePath object. Return the change of the size.
long long Map_Label_TimePath::tryInsert(const Label_TimePath& rhs) {
	long long originSize = size;
	try {
		if (rhs.getTail() != tail) throw exception();

		auto posPair = mpVisitedLabels.find(rhs.getVisited());
		if (posPair == mpVisitedLabels.end()) {
			mpVisitedLabels.insert(make_pair(rhs.getVisited(), list<Label_TimePath>{rhs}));
			++size;
		}
		else {
			auto posList = posPair->second.begin();
			while (posList != posPair->second.end()) {
				if (posList->dominate(rhs))
					break;
				else if (rhs.dominate(*posList)) {
					auto temp = posList;
					++posList;
					posPair->second.erase(temp);
					--size;
				}
				else
					++posList;
			}
			if (posList == posPair->second.end()) {
				posPair->second.push_front(rhs);
				++size;
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("Map_Label_TimePath::tryInsert", exc);
	}
	return size - originSize;
}


long long initiateForEnumerationStructure(const Data_Input_ESPPRC& input, vector<vector<Map_Label_TimePath>>& structures) {
	long long numStructuresNext = 0;
	try {
		// Define the initial Label_TimePath.
		Consumption_ESPPRC csp(0, 0, input.TimeWindow[0].first);
		Cost_ESPPRC cst;
		cst.reset();
		Label_TimePath initialLable(input, 0, csp, cst);
		if (initialLable.getUnreachable().test(0)) throw exception();

		int current = 0, next = 1;
		for (int i = 1; i < input.NumVertices; ++i) {
			structures[current][i] = Map_Label_TimePath(i);
			structures[next][i] = Map_Label_TimePath(i);
			if (initialLable.canExtend(input, i)) {
				Label_TimePath childLabel(initialLable);
				childLabel.extend(input, i);
				numStructuresNext += structures[next][i].tryInsert(childLabel);
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("initiateForEnumerationStructure", exc);
	}
	return numStructuresNext;
}


Map_Label_TimePath EnumerationStructure(const Data_Input_ESPPRC& input, double maxRC) {
	Map_Label_TimePath result(0);
	try {
		// Compute the completion bounds.
		Data_Auxiliary_ESPPRC auxiliary;
		lbBasedOnAllResources(input, auxiliary);

		// Initiate.
		int current = 0, next = 1;
		vector<vector<Map_Label_TimePath>> structures(2, vector<Map_Label_TimePath>(input.NumVertices, Map_Label_TimePath()));
		long long numNext = initiateForEnumerationStructure(input, structures);
		long long numTotal = numNext, numPrunedBounds = 0, numPrunedDominance = 0, numExtended = 0;

		// Label setting algorithm.
		while (numNext > 0) {
			std::swap(current, next);
			for (auto& elem : structures[next]) {
				numExtended += elem.getSize();
				elem.reset();
			}

			cout << "numTotal: " << numTotal << '\t' << "numNext: " << numNext << '\t' << "numCompleted: " << result.getSize() << endl;
			cout << "numPrunedBounds: " << numPrunedBounds << '\t' << "numPrunedDominance: " << numPrunedDominance << '\t' 
				<< "numExtended: " << numExtended << endl;

			for (int i = 1; i < input.NumVertices; ++i) {
				for (const auto& pairVisitedList : structures[current][i].getMpVisitedLabels()) {
					for (const auto& parentLabel : pairVisitedList.second) {
						if (!parentLabel.canExtend(input, 0)) throw exception();
						Label_TimePath completedLabel(parentLabel);
						completedLabel.extend(input, 0);
						++numTotal;
						numPrunedDominance += 1 - result.tryInsert(completedLabel);

						for (int j = 1; j < input.NumVertices; ++j) {
							if (parentLabel.canExtend(input, j)) {
								Label_TimePath childLabel(parentLabel);
								childLabel.extend(input, j);
								++numTotal;

								if (greaterThanReal(lbOfALabelInDPAlgorithmESPPRC(input, auxiliary, childLabel), maxRC, PPM)) {
									++numPrunedBounds;
								}
								else {
									numPrunedDominance += 1 - structures[next][j].tryInsert(childLabel);
								}
							}
						}
					}
				}
			}

			numNext = 0;
			for (const auto& elem : structures[next])
				numNext += elem.getSize();
		}

		for (const auto& elem : structures[current])
			numExtended += elem.getSize();

		cout << "numTotal: " << numTotal << '\t' << "numNext: " << numNext << '\t' << "numCompleted: " << result.getSize() << endl;
		cout << "numPrunedBounds: " << numPrunedBounds << '\t' << "numPrunedDominance: " << numPrunedDominance << '\t'
			<< "numExtended: " << numExtended << endl;
	}
	catch (const exception& exc) {
		printErrorAndExit("EnumerationStructure", exc);
	}
	return result;
}


void testUntilStructureEnumeration() {
	try {
		Data_Input_VRPTW inputVRPTW;
		inputVRPTW.constrainResource = { true,false,true };

		string outFile = "data//CMTVRPTW//Test//testUntilStructureEnumeration.txt";
		ofstream os(outFile);
		if (!os) throw exception();
		os << "Name" << '\t' << "NumVertices" << '\t' << "Capacity" << '\t' << "Density" << '\t' << "Lower bound" << '\t' << "Running Time (s)" << '\t'
			<< "NumStructures" << '\t' << "Running Time (s)" << endl;

		vector<string> folders = { "data//CMTVRPTW//Solomon Type 2 - 25//",
			"data//CMTVRPTW//Solomon Type 2 - 40//",
			"data//CMTVRPTW//Solomon Type 2 - 50//" };
		vector<string> names;
		getFiles(folders[0], vector<string>(), names);

		clock_t start = clock();
		for (const auto& folder : folders) {
			for (const auto& name : names) {
				string strInput = folder + name;
				readFromFileVRPTW(inputVRPTW, strInput);
				inputVRPTW.preprocess();
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "*****************************************" << endl;
				cout << "Instance: " << inputVRPTW.name << '\t' << "NumVertices: " << inputVRPTW.NumVertices << '\t' << "Time: " << runTime(start) << endl;

				clock_t last = clock();
				Solution_VRPTW_CG solForVRPTWCG = lbAtCGRootNodeVRPTW(inputVRPTW);
				double timeForVRPTWCG = runTime(last);

				last = clock();
				double gapGuess = 0.05;
				Map_Label_TimePath resultForEnumeration = EnumerationStructure(solForVRPTWCG.getInput(), solForVRPTWCG.getCost() * gapGuess);
				double timeForEnumeration = runTime(last);

				os << inputVRPTW.name << '\t' << inputVRPTW.NumVertices << '\t' << inputVRPTW.capacity << '\t' << inputVRPTW.density << '\t' 
					<< solForVRPTWCG.getCost() << '\t' << timeForVRPTWCG << '\t' << resultForEnumeration.getSize() << '\t' << timeForEnumeration << endl;
			}
		}
		os.close();
	}
	catch (const exception& exc) {
		printErrorAndExit("testVRPTWCG", exc);
	}
}


