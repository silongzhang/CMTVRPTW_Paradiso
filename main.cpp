#include"Header.h"
#include"VRPTW.h"
#include"Enumeration_TimePath.h"
#include"RSFC.h"
#include"TOPTW.h"
#include"BP.h"
#include"CMTVRPTW.h"


int main(int argc, char** argv) {
	//const string outFile = "data//CMTVRPTW//Test//CMTVRPTW_ArcFlow//ArcFlow_2021.03.20.txt";
	//Test_CMTVRPTW_ArcFlow(outFile);

	const string outFile = "data//CMTVRPTW//Test//CMTVRPTW_SP//SP_BC_ArcFlow_2021.03.20.txt";
	Test_CMTVRPTW_SP(outFile);

	return 0;
}

