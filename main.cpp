#include"Header.h"
#include"VRPTW.h"


int main(int argc, char** argv) {
	Data_Input_VRPTW inputVRPTW;
	string strInput = "data//CMTVRPTW//Solomon Type 2 - 25//C201.txt";
	readFromFileVRPTW(inputVRPTW, strInput);

	return 0;
}

