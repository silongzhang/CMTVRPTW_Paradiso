#include"Header.h"
#include"VRPTW.h"
#include"Enumeration_TimePath.h"
#include"RSFC.h"
#include"TOPTW.h"
#include"BP.h"


int main(int argc, char** argv) {
	string strInput = "data//CMTVRPTW//Test//TOPTW//Input//TOPTW_BP_ArcFlow//RC201_25_1.txt";
	Parameter_TOPTW_ArcFlow parameterArcFlow;
	readFromFileVRPTW(parameterArcFlow.input_VRPTW, strInput);
	parameterArcFlow.input_VRPTW.constrainResource = { false,false,true };
	parameterArcFlow.input_VRPTW.preprocess();
	parameterArcFlow.allowPrintLog = true;

	double objectiveArcFlow = TOPTW_ArcFlow(parameterArcFlow, cout);

	return 0;
}

