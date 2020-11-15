#pragma once

#include"Header.h"

constexpr auto TwentyFour = 24;
constexpr auto Ten = 10;
constexpr auto Seven = 7;
constexpr auto One = 1;
constexpr auto DECI = 0.1;
constexpr auto CENTI = 1e-2;
constexpr auto MILLI = 1e-3;
constexpr auto PPM = 1e-6;
constexpr auto InfinityPos = 1e9;
constexpr auto InfinityNeg = -InfinityPos;


typedef int IDType;
typedef double TimeType;
typedef double DistanceType;
typedef double QuantityType;
typedef double CostType;
typedef double ReducedCostType;
typedef double CoordinateType;
typedef double DualType;


typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloNumVarArray2> IloNumVarArray3;
typedef IloArray<IloNumVarArray3> IloNumVarArray4;
typedef IloArray<IloIntVarArray> IloIntVarArray2;
typedef IloArray<IloBoolVarArray> IloBoolVarArray2;
typedef IloArray<IloBoolVarArray2> IloBoolVarArray3;


void printErrorAndExit(const string &str, const exception &exc);
double runTime(const clock_t &start);
string getNowTime();
bool lessThanReal(const double &lhs, const double &rhs, const double &threshold);
bool greaterThanReal(const double &lhs, const double &rhs, const double &threshold);
bool equalToReal(const double &lhs, const double &rhs, const double &threshold);
void printSeparator(const int &numOfEnter, const int &vertical, const int &horizontal, 
	const char &character);
void printSeparator(const int &numOfEnter, const int &vertical, const int &horizontal,
	const char &character, ofstream &os);
void print(const bool allowPrint, ostream &os, const string &str);


template<typename T>
void print1(ostream &os, const T &cont, const char character) {
	for (const auto &elem : cont) {
		os << elem << character;
	}
}


template<typename T>
void print2(ostream &os, const T &cont, const char character) {
	for (const auto &first : cont) {
		for (const auto &second : first) {
			os << second << character;
		}
		os << endl;
	}
}


template<typename T>
void read2(istream &ins, T &cont) {
	for (int i = 0; i < cont.size(); ++i) {
		for (int j = 0; j < cont[i].size(); ++j) {
			ins >> cont[i][j];
		}
	}
}


// transfer string to number
template<typename T>
void strToNum(const string &str, T &num) {
	stringstream ss;
	ss << str;
	ss >> num;
}


// transfer number to string
template<typename T>
string numToStr(const T &num) {
	string str;
	stringstream ss;
	ss << num;
	ss >> str;
	return str;
}


class thread_guard
{
private:
	thread &t;
public:
	//构造函数
	explicit thread_guard(thread &myT) :t(myT) {}
	//析构函数
	~thread_guard()
	{
		if (t.joinable())
		{
			t.join();
		}
	}
	thread_guard(const thread_guard &) = delete;
	thread_guard& operator=(const thread_guard &) = delete;
};

