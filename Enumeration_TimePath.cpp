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

