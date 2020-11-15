#pragma once

#include"VRPTW.h"

class Time_Attribute {
private:
	TimeType duration;
	TimeType earliestDeparture;
	TimeType latestDeparture;
	TimeType buffer;

public:
	TimeType getDuration() const { return duration; }
	TimeType getEarliestDeparture() const{ return earliestDeparture; }
	TimeType getLatestDeparture() const { return latestDeparture; }
	TimeType getBuffer() const { return buffer; }

	void reset(const Data_Input_ESPPRC &inputESPPRC) { earliestDeparture = latestDeparture = inputESPPRC.TimeWindow[0].first; duration = buffer = 0; }
	// Renew this object after extending from vertex i to vertex j.
	void extend(const Data_Input_ESPPRC &data, const TimeType currentTime, const int i, const int j);
};

class Label_TimePath : public Label_ESPPRC {
private:
	bitset<Max_Num_Vertex> visited;
	Time_Attribute timeAttribute;

public:
	// Default constructor.
	Label_TimePath() {}
	// Constructor.

	bitset<Max_Num_Vertex> getVisited() const { return visited; }
	Time_Attribute getTimeAttribute() const { return timeAttribute; }
	TimeType getDepartureTime() const = delete;

	// Extend this lable to vertex j.
	void extend(const Data_Input_ESPPRC &data, const int j);
	// Check whether this label is a feasible route.
//	bool feasible(const Data_Input_ESPPRC &data) const;
	// Output.
//	void print(ostream &output) const;
};


