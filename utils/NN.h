#ifndef NN_H
#define NN_H

#include "FunctionApproximator.h"	
#include <vector>

class NN : public FunctionApproximator {
public:
	NN(int inputs, int hiddens, int outputs, double learnrate);
	NN(NN *);
	NN(FunctionApproximator *);
	NN(char *);
	~NN();
	void load(const char *);
	void load(FILE *);
	void load(const NN *);
	void load(const FunctionApproximator *fa) { load((NN*)fa); }
	static bool validSaveFile(char *fname);
	void save(const char *);
	void save(FILE *);

	double train(std::vector<double> &input, std::vector<double> &output2);
	double *test(const std::vector<double> &input);
	double train(std::vector<unsigned int> &input, std::vector<double> &output2);
	double *test(const std::vector<unsigned int> &input);

	
//	void setLearnRate(double);
//	double getLearnRate();

	void setMomentum(double);
	double getMomentum();
	
	int getNumInputs() { return inputs; }
	double getInputWeight(int inp, int outp=0) { return weights[outp][0][inp]; }

	void Print();
private:
		void allocateMemory(const NN *nn = 0);
		void freeMemory();

		std::vector< std::vector< std::vector<double> > > weights;
		std::vector< std::vector< std::vector<double> > > errors;
//	double*** weights;
//	double*** errors;
	std::vector<double> hidden;
	std::vector<double> output;
//	double* hidden;
//	double* output;
	double momentum; // rate, 
	int inputs, hiddens, outputs;
	
	double g(double a);
	double dg(double a);
	double outputerr(std::vector<double> &output, std::vector<double> &expected, int which);
	double internalerr(std::vector<double> &output, std::vector<double> &expected, int which);
	double error(std::vector<double> &outputs);
};


#endif
