#include"general.h"

// Euclidean distance.
double EuclideanDistance(const double x1, const double y1, const double x2, const double y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


// Set precision.
// NB. Ceil function is used.
double setPrecision(const double data, const int numFloat) {
	return ceil(data * pow(10, numFloat)) / pow(10, numFloat);
}


// Read Solomon instance.
Instance_Solomon readSolomonInstance(const string &input, const int numVer) {
	Instance_Solomon result;
	try {
		ifstream ins(input);
		if (!ins) throw exception("Input file Error.");

		ins >> result.name;

		string temp;
		// Skip four lines.
		getline(ins, temp);
		getline(ins, temp);
		getline(ins, temp);
		getline(ins, temp);

		ins >> result.numVehicle >> result.capacity;

		// Skip five lines.
		getline(ins, temp);
		getline(ins, temp);
		getline(ins, temp);
		getline(ins, temp);
		getline(ins, temp);

		int iter = -1;
		Customer_Solomon cust;
		for (ins >> cust.id; !ins.eof() && ++iter < numVer; ins >> cust.id) {
			if (iter != cust.id) throw exception("Input file Error.");
			ins >> cust.xCoord >> cust.yCoord >> cust.demand >> cust.readyTime >> cust.dueTime >> cust.serviceTime;
			result.vertices.push_back(cust);
		}
		ins.close();
	}
	catch (const exception &exc) {
		printErrorAndExit("readSolomonInstance", exc);
	}
	return result;
}


// Get the paths and names of all files in a folder.
void getFiles(const string &folder, vector<string> &paths, vector<string> &names) {
	//文件句柄  
	intptr_t hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(folder).append("\\*").c_str(), &fileinfo)) != -1) {
		do {
			//若是目录，迭代之；否则，加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR)) {
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(folder).append("\\").append(fileinfo.name), paths, names);
			}
			else {
				paths.push_back(folder + "\\" + fileinfo.name);
				names.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


vector<int> sample(int begin, int end, int size, int numShuffle) {
	vector<int> sequence;
	try {
		if (end < begin || size > end - begin) throw exception();

		for (int i = begin; i < end; ++i) {
			sequence.push_back(i);
		}
		for (int i = 0; i < numShuffle; ++i) {
			random_shuffle(sequence.begin(), sequence.end());
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("sample", exc);
	}
	return vector<int>(sequence.begin(), sequence.begin() + size);
}


pair<bool, int> outNode(const unordered_set<int>& excluded, const IloCplex& cplex, const IloBoolVarArray2& X, int start) {
	bool exist = false;
	int index = -1;
	try {
		for (int j = 0; j < X[start].getSize(); ++j) {
			if (cplex.getValue(X[start][j]) == IloTrue && excluded.find(j) == excluded.end()) {
				exist = true;
				index = j;
				break;
			}
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("outNode", exc);
	}
	return make_pair(exist, index);
}


vector<int> getPath(const unordered_set<int>& excluded, const IloCplex& cplex, const IloBoolVarArray2& X, int start) {
	int current = start;
	vector<int> result = { current };
	try {
		for (auto out = outNode(excluded, cplex, X, current); out.first; out = outNode(excluded, cplex, X, current)) {
			current = out.second;
			result.push_back(current);
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("getPath", exc);
	}
	return result;
}

