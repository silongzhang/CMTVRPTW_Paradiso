#pragma once

#include"VRPTW.h"

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
	Time_Attribute getTimeAttribute() const { return timeAttribute; }
	TimeType getDepartureTime() const = delete;

	// Extend this lable to vertex j.
	void extend(const Data_Input_ESPPRC &data, const int j);
	// Check whether this label is a feasible route.
	bool feasible(const Data_Input_ESPPRC &data) const;
	bool dominate(const Label_TimePath& rhs) const;
};


