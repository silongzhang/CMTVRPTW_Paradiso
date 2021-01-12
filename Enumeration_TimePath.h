#pragma once

#include"VRPTW.h"

class Label_TimePath;

class Time_Attribute {
private:
	TimeType duration;
	TimeType earliestDeparture;
	TimeType latestDeparture;
	TimeType buffer;

public:
	// Default constructor.
	Time_Attribute() {}
	// Constructor.
	Time_Attribute(const TimeType drt, const TimeType ed, const TimeType ld, const TimeType bff) : 
		duration(drt), earliestDeparture(ed), latestDeparture(ld), buffer(bff) {}

	TimeType getDuration() const { return duration; }
	TimeType getEarliestDeparture() const{ return earliestDeparture; }
	TimeType getLatestDeparture() const { return latestDeparture; }
	TimeType getBuffer() const { return buffer; }

	void reset(const Data_Input_ESPPRC &inputESPPRC) { earliestDeparture = latestDeparture = inputESPPRC.TimeWindow[0].first; 
	duration = 0; buffer = inputESPPRC.TimeWindow[0].second - inputESPPRC.TimeWindow[0].first; }
	// Renew this object after extending from vertex i to vertex j.
	void extend(const Data_Input_ESPPRC &data, const TimeType currentTime, const int i, const int j);
	bool dominate(const Time_Attribute& rhs) const;
};

class outputRSFC {
public:
	vector<Label_TimePath> structures;
	vector<double> times;
	vector<tuple<int, int, int>> triplets;

	double objective;
	vector<double> dualPartition;
	vector<double> dualActive;
	vector<double> dualSR;
	map<tuple<int, int, int>, double> mapDualSR;
};

class Label_TimePath : public Label_ESPPRC {
private:
	bitset<Max_Num_Vertex> visited;
	Time_Attribute timeAttribute;

public:
	// Default constructor.
	Label_TimePath() {}
	// Constructor.
	Label_TimePath(const Data_Input_ESPPRC &data, const int origin, const Consumption_ESPPRC &csp, const Cost_ESPPRC &cst);

	bitset<Max_Num_Vertex> getVisited() const { return visited; }
	bool hasVisited(const int i) const { return visited.test(i); }
	bool strongActive(const double t) const;
	bool atLeastTwo(const tuple<int, int, int>& tp) const;
	Time_Attribute getTimeAttribute() const { return timeAttribute; }
	TimeType getDepartureTime() const = delete;
	vector<tuple<int, int, int>> getTuples(int begin, int end) const;
	double getReducedCostRSFC(const Data_Input_VRPTW& input, const outputRSFC& rsfc) const;

	// Extend this lable to vertex j.
	void extend(const Data_Input_ESPPRC &data, const int j);
	// Check whether this label is a feasible route.
	bool feasible(const Data_Input_ESPPRC &data) const;
	bool dominate(const Label_TimePath& rhs) const;
};

class Map_Label_TimePath {
private:
	int tail;
	long long size;
	unordered_map<bitset<Max_Num_Vertex>, list<Label_TimePath>> mpVisitedLabels;

public:
	// Default constructor.
	Map_Label_TimePath() :size(0) {}
	// Constructor.
	Map_Label_TimePath(int vertex) : tail(vertex), size(0) { mpVisitedLabels.clear(); }
	void reset() { size = 0; mpVisitedLabels.clear(); }
	// Try to insert a Label_TimePath object. Return the change of the size.
	long long tryInsert(const Label_TimePath& rhs);
	long long getSize() const { return size; }
	unordered_map<bitset<Max_Num_Vertex>, list<Label_TimePath>> getMpVisitedLabels() const { return mpVisitedLabels; }
};

long long initiateForEnumerationStructure(const Data_Input_ESPPRC& input, vector<vector<Map_Label_TimePath>>& structures);
Map_Label_TimePath EnumerationStructure(const Data_Input_ESPPRC& input, double maxRC);

void testUntilStructureEnumeration();

