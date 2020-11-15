#pragma once

/*

It is a procedure for solving ESPPRC, where quantity, distance and time resources are considered.

Important Assumptions:
1. Both the origin and destination locate in the depot, and the depot is vertex 0.
2. Any feasible solution should contain at least one vertex in addition to the origin and destination.
3. Resource consumption always is nonnegative and satisfies triangle inequalities.
4. Departure time is assumed to be the earliest departure time from the depot.

*/

#include"Header.h"
#include"define.h"
#include"general.h"

constexpr auto Max_Num_Vertex = 128;

class Label_ESPPRC;

enum class ResourceType { Quantity, Distance, Time };

class Data_Input_ESPPRC {
public:
	string name;
	// The number of vertices, including the depot.
	int NumVertices;
	vector<vector<QuantityType>> Quantity;
	vector<pair<QuantityType, QuantityType>> QuantityWindow;
	vector<vector<DistanceType>> Distance;
	vector<pair<DistanceType, DistanceType>> DistanceWindow;
	vector<vector<TimeType>> Time;
	vector<pair<TimeType, TimeType>> TimeWindow;
	vector<vector<double>> RealCost;
	vector<vector<double>> ReducedCost;
	// UnreachableForever[i].test(j) = true if and only if sequences {...,i,...,j,...} 
	// are always infeasible due to resource constraints.
	vector<bitset<Max_Num_Vertex>> UnreachableForever;
	// ExistingArcs[i][j] = false if and only if: (1) i = j or (2) UnreachableForever[i].test(j) = true 
	// or (3) arc (i,j) dose not exist in the network.
	vector<vector<bool>> ExistingArcs;
	int numArcs;
	double density;
	int numNegArcs;
	double percentNegArcs;
	// Parameters.
	QuantityType incrementQuantLB;
	int sizeQuantLB;
	DistanceType incrementDistLB;
	int sizeDistLB;
	TimeType incrementTimeLB;
	int sizeTimeLB;

	// Whether at least one optimal solution must be found.
	bool mustOptimal;
	// The minimum run time if mustOptimal = false and optimal solutions are not necessarily already found.
	double minRunTime;
	// The maximum time for applying dominance rule.
	double maxDominanceTime;
	// The maximum time for running the procedure.
	double maxRunTime;
	// The procedure will terminate if candidates in an iteration exceeds this number.
	int maxNumCandidates;
	// The reduced cost of returned routes cannot exceed this parameter.
	double maxReducedCost;
	// The number of returned routes cannot exceed this parameter.
	int maxNumRoutesReturned;
	// The maximum number of saved potential labels at each step when calculating the upper bound.
	int maxNumPotentialEachStep;
	// Whether printing is allowed.
	bool allowPrintLog;
	// Whether dominance rule is adopted.
	bool dominateUninserted;
	bool dominateInserted;
	// Whether constraints corresponding to resource X are considered.
	// constrainResource[0], constrainResource[1], constrainResource[2] correspond to Quantity, Distance, Time respectively.
	vector<bool> constrainResource;
	// Whether lower bounds corresponding to resource X should be applied.
	// applyLB[0], applyLB[1], applyLB[2] correspond to Quantity, Distance, Time respectively.
	vector<bool> applyLB;

	void clearAndResize();
	void graphStatistics();
	void preprocess();
	// Output.
	void print(ostream &output) const;
};

class Data_Auxiliary_ESPPRC {
public:
	// Lower bounds.
	vector<vector<double>> LBQI;
	vector<vector<double>> LBDI;
	vector<vector<double>> LBTI;
	vector<vector<vector<vector<double>>>> LBQDTI;
	// The current upper bound.
	double ub;
	// Data structures for saving labels.
	vector<unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>>> pastIU;
	vector<unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>>> currentIU;
	vector<unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>>> nextIU;
	// Numbers of labels.
	long long numUnGeneratedLabelsInfeasibility;
	long long numGeneratedLabels;
	long long numPrunedLabelsBound;
	long long numUnInsertedLabelsDominance;
	long long numDeletedLabelsDominance;
	long long numSavedLabels;
	long long numCompletedRoutes;
	long long numDiscardPotential;
	// Time elapsed.
	double timeBoundQuantity;
	double timeBoundDistance;
	double timeBoundTime;
	double timeLB;
	double timeUB;
	double timeDP;
	double timeOverall;
	clock_t startTime;
	clock_t lastTime;
	// Whether only save potential labels.
	bool onlyPotential;
	// Whether an optimal solution can be obtained.
	bool optimal;

	void clearAndResizeLB(const Data_Input_ESPPRC &data);
	void clearAndResizeIU(const Data_Input_ESPPRC &data);
	void resetNumber() {
		numUnGeneratedLabelsInfeasibility = numGeneratedLabels = numPrunedLabelsBound = numUnInsertedLabelsDominance =
			numDeletedLabelsDominance = numSavedLabels = numCompletedRoutes = numDiscardPotential = 0;
	}
	void resetTime() {
		timeBoundQuantity = timeBoundDistance = timeBoundTime = timeLB = timeUB = timeDP = timeOverall = 0;
		startTime = clock(); lastTime = startTime;
	}
	// Output.
	void print(ostream &output) const;
};

class Consumption_ESPPRC {
	friend bool operator==(const Consumption_ESPPRC &lhs, const Consumption_ESPPRC &rhs);
	friend bool operator!=(const Consumption_ESPPRC &lhs, const Consumption_ESPPRC &rhs);
	friend bool operator<(const Consumption_ESPPRC &lhs, const Consumption_ESPPRC &rhs);

private:
	QuantityType quantity;
	DistanceType distance;
	TimeType time;
	TimeType departureTime;

public:
	// Default constructor.
	Consumption_ESPPRC() {}
	// Constructor.
	Consumption_ESPPRC(const QuantityType quant, const DistanceType dist, const TimeType depTime) : 
		quantity(quant), distance(dist), departureTime(depTime) { time = departureTime; }

	void setDepartureTime(const TimeType depTime) { departureTime = depTime; }

	QuantityType getQuantity() const { return quantity; }
	DistanceType getDistance() const { return distance; }
	TimeType getTime() const { return time; }
	TimeType getDepartureTime() const { return departureTime; }
	TimeType getDurationTime() const { return time - departureTime; }
	// Reset this object.
	void reset() { quantity = 0; distance = 0; time = departureTime; }
	// Renew this object after extending from vertex i to vertex j.
	void extend(const Data_Input_ESPPRC &data, const int i, const int j);
	// Check whether this object is feasible at vertex i (resource constraints are all satisfied).
	bool feasible(const Data_Input_ESPPRC &data, const int i) const;
	// Output.
	void print(ostream &output) const;
};

class Cost_ESPPRC {
	friend bool operator==(const Cost_ESPPRC &lhs, const Cost_ESPPRC &rhs);
	friend bool operator!=(const Cost_ESPPRC &lhs, const Cost_ESPPRC &rhs);
	friend bool operator<(const Cost_ESPPRC &lhs, const Cost_ESPPRC &rhs);

private:
	double realCost;
	double reducedCost;

public:
	double getRealCost() const { return realCost; }
	double getReducedCost() const { return reducedCost; }
	// Reset this object.
	void reset() { realCost = 0; reducedCost = 0; }
	// Renew this object after extending from vertex i to vertex j.
	void extend(const Data_Input_ESPPRC &data, const int i, const int j);
	// Output.
	void print(ostream &output) const;
};

class Label_ESPPRC {
protected:
	vector<int> path;
	int tail;									// The last vertex in the path.
	bitset<Max_Num_Vertex> unreachable;
	Consumption_ESPPRC consumption;
	Cost_ESPPRC cost;

public:
	// Default constructor.
	Label_ESPPRC() {}
	// Constructor.
	Label_ESPPRC(const Data_Input_ESPPRC &data, const int origin, const Consumption_ESPPRC &csp, const Cost_ESPPRC &cst);

	vector<int> getPath() const { return path; }
	int getTail() const { return tail; }
	bitset<Max_Num_Vertex> getUnreachable() const { return unreachable; }
	Consumption_ESPPRC getConsumption() const { return consumption; }
	QuantityType getQuantity() const { return consumption.getQuantity(); }
	DistanceType getDistance() const { return consumption.getDistance(); }
	TimeType getTime() const { return consumption.getTime(); }
	TimeType getDepartureTime() const { return consumption.getDepartureTime(); }
	Cost_ESPPRC getCost() const { return cost; }
	double getRealCost() const { return cost.getRealCost(); }
	double getReducedCost() const { return cost.getReducedCost(); }

	// Check whether this label can extend to vertex j.
	bool canExtend(const Data_Input_ESPPRC &data, const int j) const;
	// Renew unreachable indicator for vertex j.
	void renewUnreachable(const Data_Input_ESPPRC &data, const int j);
	// Renew unreachable indicators for all vertices.
	void renewUnreachable(const Data_Input_ESPPRC &data);
	// Extend this lable to vertex j.
	void extend(const Data_Input_ESPPRC &data, const int j);
	// Get the feasibility and consumption.
	pair<bool, Consumption_ESPPRC> getFeasibilityAndConsumption(const Data_Input_ESPPRC &data, const TimeType departureTime) const;
	// Check whether this label is a feasible solution.
	bool feasible(const Data_Input_ESPPRC &data) const;
	// Output.
	void print(ostream &output) const;
};

class Label_ESPPRC_Sort_Criterion {
public:
	bool operator()(const Label_ESPPRC &lhs, const Label_ESPPRC &rhs) const {
		return lessThanReal(lhs.getReducedCost(), rhs.getReducedCost(), PPM);
	}
};

// Bitwise operation for checking whether unreachable lhs is a subset of unreachable rhs.
bool operator<=(const bitset<Max_Num_Vertex> &lhs, const bitset<Max_Num_Vertex> &rhs);
// Dominance rule. Whether label lhs can dominate label rhs.
bool dominate(const Label_ESPPRC &lhs, const Label_ESPPRC &rhs);
// Initiate.
void initiateForLbBasedOnOneResourceGivenAmount(const ResourceType type, const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary,
	const int index, const int origin, double **result, Consumption_ESPPRC &csp);
// Whether lower bounds can be used.
bool canUseLBInLbBasedOnOneResourceGivenAmount(const ResourceType type, const Data_Input_ESPPRC &data, const Data_Auxiliary_ESPPRC &auxiliary,
	const int index, const Label_ESPPRC &label);
// Get the lower bound of a label.
double lbOfALabelInLbBasedOnOneResourceGivenAmount(const ResourceType type, const Data_Input_ESPPRC &data,
	const Data_Auxiliary_ESPPRC &auxiliary, const int index, const Label_ESPPRC &label);
// Whether a label is dominated.
bool labelIsDominated(const unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>> &mpBtstLst, const Label_ESPPRC &label);
// Discard labels according to the dominance rule.
int discardAccordingToDominanceRule(unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>> &mpBtstLst, const Label_ESPPRC &label);
// Insert a label.
void insertLabel(unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>> &mpBtstLst, const Label_ESPPRC &label);
// Compute the lower bound based on a single type of resource (given the amount of consumption of such resource).
void lbBasedOnOneResourceGivenAmount(const ResourceType type, const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary,
	const int index, const int origin);
// Compute lower bounds based on a single type of resource.
void lbBasedOnOneResource(const ResourceType type, const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary);
// Compute lower bounds based on all types of resources.
void lbBasedOnAllResources(const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary);
// Initiate.
bool initiateForDPAlgorithmESPPRC(const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary);
// Calculate the lower bound for a label.
double lbOfALabelInDPAlgorithmESPPRC(const Data_Input_ESPPRC &data, const Data_Auxiliary_ESPPRC &auxiliary, const Label_ESPPRC &label);
// Number of labels.
long long numOfLabels(const vector<unordered_map<bitset<Max_Num_Vertex>, list<Label_ESPPRC>>> &vecMpBtstLst);
// Core function of dynamic programming algorithm for ESPPRC.
multiset<Label_ESPPRC, Label_ESPPRC_Sort_Criterion> coreDPAlgorithmESPPRC(const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary,
	ostream &output);
// Dynamic programming algorithm for ESPPRC.
multiset<Label_ESPPRC, Label_ESPPRC_Sort_Criterion> DPAlgorithmESPPRC(const Data_Input_ESPPRC &data, Data_Auxiliary_ESPPRC &auxiliary,
	ostream &output);
void readDataSolomonESPPRC(const Instance_Solomon &inst, Data_Input_ESPPRC &data, const double coefDist, const vector<double> &prize,
	const int precision);
// Write input data to file.
void writeToFile(const Data_Input_ESPPRC &data, const string &strOutput);
// Read input data from file.
void readFromFile(Data_Input_ESPPRC &data, const string &strInput);
// Output.
void printResultsDPAlgorithmESPPRC(const Data_Input_ESPPRC &data, const Data_Auxiliary_ESPPRC &auxiliary, ostream &output,
	const multiset<Label_ESPPRC, Label_ESPPRC_Sort_Criterion> &result);

class ParameterTestDPAlgorithmESPPRC {
public:
	string strInputSolomon;
	string strInstance;
	string strOutput;
	double prize;
	bool dominateUninserted;
	bool dominateInserted;
};

// Test.
double testDPAlgorithmESPPRC(const ParameterTestDPAlgorithmESPPRC &parameter, ostream &osAll);
// Test all instances under a folder.
void testDPAlgorithmESPPRCFolder(const string &folderSolomon, const string &folderInstance, const string &folderOutput);


