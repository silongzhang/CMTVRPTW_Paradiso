#include"RSFC.h"


vector<Label_TimePath> linearize(const Map_Label_TimePath& structures) {
	vector<Label_TimePath> result;
	try {
		for (const auto& elem : structures.getMpVisitedLabels()) {
			for (const auto& structure : elem.second) {
				result.push_back(structure);
			}
		}
		if (result.size() != structures.getSize()) throw exception();
	}
	catch (const exception& exc) {
		printErrorAndExit("linearize", exc);
	}
	return result;
}


void setObjective(const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x) {
	try {
		auto env = model.getEnv();
		IloExpr expr(env);
		for (int i = 0; i < structures.size(); ++i) {
			expr += structures[i].getRealCost() * x[i];
		}
		model.add(IloMinimize(env, expr));
		expr.end();
	}
	catch (const exception& exc) {
		printErrorAndExit("setObjective", exc);
	}
}


void setConstraintsPartition(const Data_Input_ESPPRC& input, const vector<Label_TimePath>& structures, IloModel model, IloNumVarArray x) {
	try {
		auto env = model.getEnv();
		for (int i = 1; i < input.NumVertices; ++i) {
			IloExpr expr(env);
			for (int s = 0; s < structures.size(); ++s) {
				if (structures[s].hasVisited(i)) {
					expr += x[s];
				}
			}
			model.add(expr == 1);
			expr.end();
		}
	}
	catch (const exception& exc) {
		printErrorAndExit("setConstraintsPartition", exc);
	}
}




void RSFC(outputRSFC& output, const Data_Input_ESPPRC& input, const Map_Label_TimePath& stt) {
	IloEnv env;
	try {
		set<double, timeSortCriterion> times;
		set<tuple<int, int, int>> triplets;

		vector<Label_TimePath> structures = linearize(stt);
		const long long NumStructures = structures.size();

		// Define the variables.
		IloNumVarArray x(env, NumStructures, 0, IloInfinity);

		// Define the model.
		IloModel model(env);
		setObjective(structures, model, x);
		setConstraintsPartition(input, structures, model, x);


	}
	catch (const exception& exc) {
		printErrorAndExit("RSFC", exc);
	}
	env.end();
}

