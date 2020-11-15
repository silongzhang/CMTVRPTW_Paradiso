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

