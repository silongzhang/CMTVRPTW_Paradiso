#pragma once

#include"ESPPRC.h"
#include"VRPTW.h"
#include"RSFC.h"

class Parameter_TOPTW_CG {
public:
	Data_Input_VRPTW input_VRPTW;
	vector<Route_VRPTW> initialRoutes;
	bool allowPrintLog;

	map<int, bool> branchOnVertices;					// [i, true] means vertex i must be visited; false means vertex i cannot be visited.
	map<pair<int, int>, bool> branchOnArcs;				// [(i, j), true] means arc (i, j) must be visited if i and j are visited; 
														// false means (i, j) cannot be visited.
	pair<int, bool> branchOnVehicleNumber;				// [m, true] means the number of vehicles must be greater than or equal to m;
														// false means the number of vehicles must be less than or equal to m.
	int numArtificial;									// The number of artificial variables.

	void reviseInputVRPTW();
	void reviseInitialRoutes();
	void reviseNumArtificial() { numArtificial = branchOnVehicleNumber.second ? std::max(1, branchOnVehicleNumber.first) : 1; }
};

class Solution_TOPTW_CG {
public:
	bool explored;								// Whether this node has been explored.
	bool feasible;								// Whether the LP formulation corresponding to this node is feasible.
	bool integer;								// Whether the optimal solution corresponding to this node is an integer solution.
	double objective;							// The optimal objective value corresponding to this node.
	double UB_Integer_Value;					// The best (smallest) upperbound found so far.
	vector<Route_VRPTW> UB_Integer_Solution;	// The best integer solution found so far.

	double numVehicles;							// The number of vehicles used.
	vector<double> visitVertices;				// 0 <= visitVertices[i] <=1, the times at which the vertex i has been visited.
	vector<vector<double>> visitArcs;			// 0 <= visitArcs[i][j] <=1, the times at which the arc (i, j) has been visited.
};

class TOPTW_CG {
private:
	vector<Route_VRPTW> columns;
public:
	void clearColumns() { columns.clear(); }
	void addColumn(const Route_VRPTW& rhs, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void InitiateRMP(const Parameter_TOPTW_CG& parameter, const Data_Input_ESPPRC& inputESPPRC, IloObjective& objectiveRMP, IloRangeArray& constraintRMP, IloNumVarArray& X);
	void columnGeneration(const Parameter_TOPTW_CG& parameter, Solution_TOPTW_CG& solution, ostream& output);
	void getIntegerSolution(const IloCplex& cplex, const IloNumVarArray& X, Solution_TOPTW_CG& solution);
	void getSolution(const Parameter_TOPTW_CG& parameter, const IloCplex& solverRMP, 
		const IloRangeArray& constraintRMP, const IloNumVarArray& X, Solution_TOPTW_CG& solution);
};


void setRangeArray(const Parameter_TOPTW_CG& parameter, IloModel& modelRMP, IloRangeArray& constraintRMP);
void renewReducedCost(Data_Input_ESPPRC& inputESPPRC, const Parameter_TOPTW_CG& parameter, const IloNumArray& dualValue);
bool isBool(const IloCplex& cplex, const IloNumVarArray& X);
bool isFeasible(const Parameter_TOPTW_CG& parameter, const IloCplex& cplex, const IloNumVarArray& X);
void testTOPTW();

int realIndexTOPTW(int i, int N);
void setConstraintsDomainX(const Parameter_TOPTW_CG& parameter, IloModel model, IloBoolVarArray2 X);
void setConstraintsDomainY(const Parameter_TOPTW_CG& parameter, IloModel model, IloNumVarArray Y);
void setConstraintsFlow(const Parameter_TOPTW_CG& parameter, IloModel model, IloBoolVarArray2 X);
void setConstraintsTimeWindow(const Parameter_TOPTW_CG& parameter, IloModel model, IloBoolVarArray2 X, IloNumVarArray Y);
void setObjective(const Parameter_TOPTW_CG& parameter, IloModel model, IloBoolVarArray2 X);

