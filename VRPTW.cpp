#include"VRPTW.h"


void Data_Input_VRPTW::clearAndResize() {
	try {
		Quantity.clear(); Quantity.resize(NumVertices);
		QuantityWindow.clear(); QuantityWindow.resize(NumVertices);
		Distance.clear(); Distance.resize(NumVertices);
		DistanceWindow.clear(); DistanceWindow.resize(NumVertices);
		Time.clear(); Time.resize(NumVertices);
		TimeWindow.clear(); TimeWindow.resize(NumVertices);
		RealCost.clear(); RealCost.resize(NumVertices);
		UnreachableForever.clear(); UnreachableForever.resize(NumVertices);
		ExistingArcs.clear(); ExistingArcs.resize(NumVertices);

		for (int i = 0; i < NumVertices; ++i) {
			Quantity[i].resize(NumVertices);
			Distance[i].resize(NumVertices);
			Time[i].resize(NumVertices);
			RealCost[i].resize(NumVertices);
			ExistingArcs[i].resize(NumVertices);
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("Data_Input_VRPTW::clearAndResize", exc);
	}
}


void Data_Input_VRPTW::preprocess() {
	try {
		Data_Input_ESPPRC inputESPPRC;
		inputESPPRC.NumVertices = NumVertices;
		inputESPPRC.Quantity = Quantity;
		inputESPPRC.QuantityWindow = QuantityWindow;
		inputESPPRC.Distance = Distance;
		inputESPPRC.DistanceWindow = DistanceWindow;
		inputESPPRC.Time = Time;
		inputESPPRC.TimeWindow = TimeWindow;
		inputESPPRC.ExistingArcs = ExistingArcs;
		inputESPPRC.constrainResource = constrainResource;

		inputESPPRC.preprocess();

		UnreachableForever = inputESPPRC.UnreachableForever;
		ExistingArcs = inputESPPRC.ExistingArcs;
		numArcs = inputESPPRC.numArcs;
		density = inputESPPRC.density;
		numNegArcs = inputESPPRC.numNegArcs;
		percentNegArcs = inputESPPRC.percentNegArcs;
	}
	catch (const exception &exc) {
		printErrorAndExit("Data_Input_VRPTW::preprocess", exc);
	}
}


// Read data from Solomon instance.
void readDataSolomonVRPTW(const Instance_Solomon &inst, Data_Input_VRPTW &data, const double coefDist, const int precision) {
	try {
		data.name = inst.name;
		data.NumVertices = inst.vertices.size();
		data.MaxNumVehicles = inst.numVehicle;
		data.capacity = inst.capacity;
		data.clearAndResize();

		for (int i = 0; i < data.NumVertices; ++i) {
			data.QuantityWindow[i].first = 0;
			data.QuantityWindow[i].second = inst.capacity;
			data.TimeWindow[i].first = inst.vertices[i].readyTime;
			data.TimeWindow[i].second = inst.vertices[i].dueTime;
			data.DistanceWindow[i].first = 0;
			data.DistanceWindow[i].second = (lessThanReal(coefDist, 0, PPM) ? InfinityPos : data.TimeWindow[0].second * coefDist);

			for (int j = 0; j < data.NumVertices; ++j) {
				data.Quantity[i][j] = 0.5 * (inst.vertices[i].demand + inst.vertices[j].demand);
				data.Distance[i][j] = setPrecision(EuclideanDistance(inst.vertices[i].xCoord, inst.vertices[i].yCoord,
					inst.vertices[j].xCoord, inst.vertices[j].yCoord), precision);
				data.Time[i][j] = data.Distance[i][j] + inst.vertices[i].serviceTime;
				data.RealCost[i][j] = data.Distance[i][j];
				data.ExistingArcs[i][j] = true;
			}
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("readDataSolomonVRPTW", exc);
	}
}


// Write input data to file.
void writeToFileVRPTW(const Data_Input_VRPTW &data, const string &strOutput) {
	try {
		ofstream os(strOutput);
		if (!os) throw exception("Failed file operator.");

		os << "Instance" << '\t' << "NumVertices" << '\t' << "MaxNumVehicles" << '\t' << "Capacity" << endl;
		os << data.name << '\t' << data.NumVertices << '\t' << data.MaxNumVehicles << '\t' << data.capacity << endl;

		os << "QuantMin" << '\t' << "QuantMax" << '\t' << "DistMin" << '\t' << "DistMax" << '\t' << "TimeMin" << '\t' << "TimeMax" << endl;
		for (int i = 0; i < data.NumVertices; ++i) {
			os << data.QuantityWindow[i].first << '\t' << data.QuantityWindow[i].second << '\t'
				<< data.DistanceWindow[i].first << '\t' << data.DistanceWindow[i].second << '\t'
				<< data.TimeWindow[i].first << '\t' << data.TimeWindow[i].second << endl;
		}

		os << "Quantity" << endl;
		print2(os, data.Quantity, '\t');

		os << "Distance" << endl;
		print2(os, data.Distance, '\t');

		os << "Time" << endl;
		print2(os, data.Time, '\t');

		os << "Real Cost" << endl;
		print2(os, data.RealCost, '\t');

		os << "Arcs" << endl;
		print2(os, data.ExistingArcs, '\t');

		os.close();
	}
	catch (const exception &exc) {
		printErrorAndExit("writeToFileVRPTW", exc);
	}
}


// Transfer Solomon file to VRPTW data file.
void transferDataFileVRPTW(const Parameter_TransferDataFileVRPTW &prm, const string &inputSolomon, const string &strOutput, const int precision) {
	try {
		// Read Solomon instance.
		Instance_Solomon instanceSolomon = readSolomonInstance(inputSolomon, prm.numVertex);

		instanceSolomon.numVehicle = prm.numVehicle;
		instanceSolomon.capacity = prm.capacity;

		Data_Input_VRPTW inputVRPTW;
		// Read data from Solomon instance.
		readDataSolomonVRPTW(instanceSolomon, inputVRPTW, -1, 1);

		// Write input data to file.
		writeToFileVRPTW(inputVRPTW, strOutput);
	}
	catch (const exception &exc) {
		printErrorAndExit("transferDataFile", exc);
	}
}


// Transfer Solomon files in a folder to VRPTW data files.
void transferDataFileVRPTWFolder(const Parameter_TransferDataFileVRPTW &prm, const string &strInputFolder, 
	const string &strOutputFolder, const int precision) {
	try {
		vector<string> paths;
		vector<string> names;
		getFiles(strInputFolder, paths, names);
		for (const auto &elem : names) {
			string inputSolomon = strInputFolder + elem;
			string strOutput = strOutputFolder + elem;
			transferDataFileVRPTW(prm, inputSolomon, strOutput, precision);
		}
	}
	catch (const exception &exc) {
		printErrorAndExit("transferDataFileVRPTWFolder", exc);
	}
}


// Read input data from file.
void readFromFileVRPTW(Data_Input_VRPTW &data, const string &strInput) {
	try {
		ifstream ins(strInput);
		if (!ins) throw exception("Failed file operator.");

		string strTemp;
		getline(ins, strTemp);
		ins >> data.name >> data.NumVertices >> data.MaxNumVehicles >> data.capacity;
		if (Max_Num_Vertex < data.NumVertices) throw exception("Failed file operator.");
		data.clearAndResize();

		getline(ins, strTemp);
		getline(ins, strTemp);
		for (int i = 0; i < data.NumVertices; ++i) {
			ins >> data.QuantityWindow[i].first >> data.QuantityWindow[i].second >>
				data.DistanceWindow[i].first >> data.DistanceWindow[i].second >>
				data.TimeWindow[i].first >> data.TimeWindow[i].second;
		}

		getline(ins, strTemp);
		getline(ins, strTemp);
		read2(ins, data.Quantity);

		getline(ins, strTemp);
		getline(ins, strTemp);
		read2(ins, data.Distance);

		getline(ins, strTemp);
		getline(ins, strTemp);
		read2(ins, data.Time);

		getline(ins, strTemp);
		getline(ins, strTemp);
		read2(ins, data.RealCost);

		getline(ins, strTemp);
		getline(ins, strTemp);
		vector<vector<int>> vec2Int(data.NumVertices);
		for (auto &elem : vec2Int) {
			elem.resize(data.NumVertices);
		}
		read2(ins, vec2Int);
		for (int i = 0; i < data.NumVertices; ++i) {
			for (int j = 0; j < data.NumVertices; ++j) {
				if (vec2Int[i][j] == 0) data.ExistingArcs[i][j] = false;
				else if (vec2Int[i][j] == 1) data.ExistingArcs[i][j] = true;
				else throw exception("Failed file operator.");
			}
		}

		ins.close();
	}
	catch (const exception &exc) {
		printErrorAndExit("readFromFileVRPTW", exc);
	}
}


