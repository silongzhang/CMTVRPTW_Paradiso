#pragma once


#include"Header.h"
#include"define.h"
#include"general.h"
#include"ESPPRC.h"
#include"VRPTW_CG.h"


class Data_Input_VRPTW {
public:
	string name;
	// The number of vertices, including the depot.
	int NumVertices;
	int MaxNumVehicles;
	double capacity;
	vector<vector<QuantityType>> Quantity;
	vector<pair<QuantityType, QuantityType>> QuantityWindow;
	vector<vector<DistanceType>> Distance;
	vector<pair<DistanceType, DistanceType>> DistanceWindow;
	vector<vector<TimeType>> Time;
	vector<pair<TimeType, TimeType>> TimeWindow;
	vector<vector<double>> RealCost;
	// UnreachableForever[i].test(j) = true if and only if sequences {...,i,...,j,...} 
	// are always infeasible due to resource constraints.
	vector<bitset<Max_Num_Vertex>> UnreachableForever;
	// ExistingArcs[i][j] = false if and only if: (1) i = j or (2) UnreachableForever[i].test(j) = true 
	// or (3) arc (i,j) dose not exist in the network.
	vector<vector<bool>> ExistingArcs;
	// Whether constraints corresponding to resource X are considered.
	// constrainResource[0], constrainResource[1], constrainResource[2] correspond to Quantity, Distance, Time respectively.
	vector<bool> constrainResource;

	int numArcs;
	double density;
	int numNegArcs;
	double percentNegArcs;

	// The minimum run time if mustOptimal = false and optimal solutions are not necessarily already found.
	double minRunTime;
	// The maximum time for running the procedure.
	double maxRunTime;
	// Whether printing is allowed.
	bool allowPrintLog;

	void clearAndResize();
	void preprocess();
};


class Parameter_TransferDataFileVRPTW {
public:
	int numVertex;
	int numVehicle;
	double capacity;
};


// Read data from Solomon instance.
void readDataSolomonVRPTW(const Instance_Solomon &inst, Data_Input_VRPTW &data, const double coefDist, const int precision);
// Write input data to file.
void writeToFileVRPTW(const Data_Input_VRPTW &data, const string &strOutput);
Data_Input_VRPTW subData(const Data_Input_VRPTW& data, const vector<int>& index, int numVehicles);
// Transfer Solomon file to VRPTW data file.
void transferDataFileVRPTW(const Parameter_TransferDataFileVRPTW &prm, const string &inputSolomon, const string &strOutput, const int precision);
// Transfer Solomon files in a folder to VRPTW data files.
void transferDataFileVRPTWFolder(const Parameter_TransferDataFileVRPTW &prm, const string &strInputFolder,
	const string &strOutputFolder, const int precision);
// Read input data from file.
void readFromFileVRPTW(Data_Input_VRPTW &data, const string &strInput);
// Set parameters of Data_Input_ESPPRC.
Data_Input_ESPPRC setParametersInputESPPRCFromInputVRPTW(const Data_Input_VRPTW &inputVRPTW);
// Generate a route.
pair<bool, Route_VRPTW> generateRoute(const Data_Input_ESPPRC &inputESPPRC, const vector<int> &sequence);
// Generate an initial set of routes.
vector<Route_VRPTW> generateInitialRoutes(const Data_Input_ESPPRC &inputESPPRC);
// Get the value of a lower bound at the root node of BP tree.
Solution_VRPTW_CG lbAtCGRootNodeVRPTW(const Data_Input_VRPTW &inputVRPTW);
void testVRPTWCG();
void sampleDataInputVRPTW(const string& strInput, const string& outFolder, const vector<int>& sizes, const vector<int>& numVehicles);

