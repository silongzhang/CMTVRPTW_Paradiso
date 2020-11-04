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
Instance_Solomon readSolomonInstance(const string &input) {
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
		for (ins >> cust.id; !ins.eof(); ins >> cust.id) {
			if (++iter != cust.id) throw exception("Input file Error.");
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


