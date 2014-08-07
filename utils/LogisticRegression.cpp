//
//  LogisticRegression.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/19/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#include "LogisticRegression.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>
#include "LinearRegression.h"
#include "SwapEndian.h"

static const float VERSION = 1.1;
static const float MINVERSION = 1.0;

LogisticRegression::LogisticRegression(int _inputs, int _outputs, double _rate)
{
	useBinary = false;
	outputActivation = kExponential;
	inputs = _inputs;
	outputs = _outputs;
	rate = _rate;
	weight.resize(0);
	updates.resize(0);
	//weight = 0;
	allocateMemory();
}

LogisticRegression::LogisticRegression(LogisticRegression *perp)
{
	useBinary = perp->useBinary;
	//	inputs = perp->inputs;
	outputs = perp->outputs;
	rate = perp->rate;
	outputActivation = perp->outputActivation;
	weight.resize(0);
	updates.resize(0);
	//weight = 0;
	//allocateMemory(perp->inputs);
	load(perp);
}

LogisticRegression::LogisticRegression(char *f)
{
	useBinary = false;
	weight.resize(0);
	updates.resize(0);
	//weight = 0;
	inputs = outputs = -1;
	load(f);
}

LogisticRegression::LogisticRegression(FunctionApproximator *fa)
{
	LogisticRegression *perp = (LogisticRegression *)fa;
	useBinary = perp->useBinary;
	inputs = perp->inputs;
	outputs = perp->outputs;
	outputActivation = perp->outputActivation;
	rate = perp->rate;
	weight.resize(0);
	updates.resize(0);
	//weight = 0;
	//allocateMemory();
	load(perp);
}

LogisticRegression::~LogisticRegression()
{
	freeMemory();
}

void LogisticRegression::resizeInputs(int newSize, double newVal)
{
	if (newSize < inputs)
	{
		for (int x = 0; x < outputs; x++)
		{
			weight[x][newSize] = weight[x][inputs];
			weight[x].resize(newSize+1);
			updates[x].resize(newSize);
		}
	}
	else if (newSize == inputs)
	{
		return;
	}
	else {
		for (int x = 0; x < outputs; x++)
		{
			weight[x].resize(newSize+1);// = new double[inputs+1];
			weight[x][newSize] = weight[x][inputs];
			for (int y = inputs; y < newSize; y++)
				weight[x][y] = newVal;
			
			updates[x].resize(newSize);// = new double[inputs+1];
		}
	}
	inputs = newSize;
}

void LogisticRegression::resizeInputs(int newSize)
{
	if (newSize < inputs)
	{
		for (int x = 0; x < outputs; x++)
		{
			weight[x][newSize] = weight[x][inputs];
			weight[x].resize(newSize+1);
			updates[x].resize(newSize);
		}
	}
	else if (newSize == inputs)
	{
		return;
	}
	else {
		for (int x = 0; x < outputs; x++)
		{
			weight[x].resize(newSize+1);// = new double[inputs+1];
			weight[x][newSize] = weight[x][inputs];
			for (int y = inputs; y < newSize; y++)
				weight[x][y] = ((double)2*random()/RAND_MAX-1)/(double)newSize;
			
			updates[x].resize(newSize);// = new double[inputs+1];
		}
	}
	inputs = newSize;
}

void LogisticRegression::allocateMemory()
{
	//	if (weight.size() != 0)
	//		freeMemory();
	weight.resize(outputs);// = new double*[outputs];
	updates.resize(outputs);
	for (int x = 0; x < outputs; x++)
	{
		weight[x].resize(inputs+1);// = new double[inputs+1];
		updates[x].resize(inputs);
		for (int y = 0; y <= inputs; y++)
			weight[x][y] = 30;//((double)2*random()/RAND_MAX-1)/(double)inputs;
		weight[x][inputs] = 0;
	}
	output.resize(outputs);// = new double[outputs];
}

void LogisticRegression::freeMemory()
{
	//	if (weight != 0)
	//	{
	////		for (int x = 0; x < outputs; x++)
	////			delete [] weight[x];
	////		delete [] weight;
	////		delete [] output;
	weight.resize(0);
	updates.resize(0);
	//		//weight = 0;
	//	}
}

void LogisticRegression::load(const char *fname)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		fprintf(stderr, "LOG_REGR Error: could not open file '%s' for loading; trying once more.\n", fname);
		sleep(1);
		f = fopen(fname, "r");
		if (f == 0)
		{
			fprintf(stderr, "LOG_REGR Error: could not open file '%s' for loading.\n", fname);
			exit(0);
			return;
		}
	}
	load(f);
	fclose(f);
}

void LogisticRegression::load(FILE *f)
{
	int inputs1, outputs1;
	float version;
	int res = fscanf(f, "LOG_REGR %f %d %d\n", &version, &inputs1, &outputs1);
	if (res != 3)
	{
		printf("Error: unrecognized LogisticRegression file. Expected header 'PERP <version> <inputs> <outputs>'.");
		exit(0);
	}
	if (version > VERSION)
	{
		printf("Error: loaded network is %1.2f, but code can only handle %1.2f to %1.2f.",
			   version, MINVERSION, VERSION);
		exit(0);
	}
	if (VERSION >= 1.1)
	{
		char text[25];
		res = fscanf(f, "%s\n", text);
		assert(res == 1);
		if (strcmp(text, "binary") == 0)
			useBinary = true;
		else //if (strcmp(text, "text") == 0)
			useBinary = false;
	}
	
	if ((inputs1 != inputs) || (outputs1 != outputs))
	{
		freeMemory();
		inputs = inputs1;
		outputs = outputs1;
		allocateMemory();
	}
	
	for (int y = 0; y < outputs; y++)
	{
		for (int x = 0; x <= inputs; x++)
		{
			if (useBinary)
			{
				float shrunk;
				fread(&shrunk, sizeof(float), 1, f);
				//fread(&weight[y][x], sizeof(double), 1, f);
				little2machine(shrunk);
				weight[y][x] = shrunk;
			}
			else
				fscanf(f, "%le ", &weight[y][x]);
		}
	}
}

void LogisticRegression::load(const LogisticRegression *p)
{
	if (p && ((p->inputs != inputs) || (p->outputs != outputs)))
	{
		freeMemory();
		inputs = p->inputs;
		outputs = p->outputs;
		outputActivation = p->outputActivation;
		
		allocateMemory();
	}
	
	if (p)
	{
		for (int y = 0; y < outputs; y++)
		{
			for (int x = 0; x <= inputs; x++)
			{
				weight[y][x] = p->weight[y][x];
			}
		}
	}
}

bool LogisticRegression::validSaveFile(char *fname)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		return false;
	}
	int finput, foutput;
	float version;
	int res = fscanf(f, "LOG_REGR %f %d %d\n", &version, &finput, &foutput);
	fclose(f);
	if (res != 3)
	{
		return false;
	}
	if (version > VERSION)
	{
		return false;
	}
	return true;
}

void LogisticRegression::save(const char *fname)
{
	FILE *f;
	f = fopen(fname, "w+");
	if (f == 0)
	{
		fprintf(stderr, "Error: could not open file for saving.\n");
		return;
	}
	save(f);
	fclose(f);
}

void LogisticRegression::save(FILE *f)
{
	fprintf(f, "LOG_REGR %1.2f %d %d\n", VERSION, inputs, outputs);
	if (useBinary)
		fprintf(f, "binary\n");
	else
		fprintf(f, "text\n");
	
	for (int y = 0; y < outputs; y++)
	{
		for (int x = 0; x <= inputs; x++)
		{
			if (useBinary)
			{
				float val = weight[y][x];
				little2machine(val);
				fwrite(&val, sizeof(float), 1, f);
			}
			else {
				fprintf(f, "%le ", weight[y][x]);
			}
		}
		if (!useBinary)
			fprintf(f, "\n");
	}
}
double LogisticRegression::g(double a)
{
	return (1/(1+exp(-a)));
}

double LogisticRegression::dg(double a)
{
	double g_a = g(a);
	return g_a*(1-g_a);
}

double LogisticRegression::outputerr(std::vector<double> &out, std::vector<double> &expected, int which)
{
	double err = (expected[which]-out[which]);
	err *= dg(out[which]);
	
	return err;
//	double err = (out[which]-expected[which]);
//	//printf("Output error: %1.2f\n", err);
//	return err;
}

double LogisticRegression::train(std::vector<unsigned int> &input, std::vector<double> &target)
{
	double totalErr = 0;
	test(input);
	for (int x = 0; x < outputs; x++)
	{
		double err = outputerr(output,target,x);
		totalErr+=err*err;
		double rateTimesError = rate*err;
		for (unsigned int y = 0; y < input.size(); y++)
		{
			weight[x][input[y]] -= rateTimesError;
			updateData &val = updates[x][input[y]];
			val.n++;
			//			double delta = err-val.mean;
			double delta = rateTimesError-val.mean;
			val.mean = val.mean+delta/val.n;
			val.S += delta*(err-val.mean);
			val.totErr += rateTimesError*rateTimesError;
		}
		weight[x][inputs] -= rateTimesError; // bias
	}
	return totalErr;
}

double LogisticRegression::train(std::vector<double> &input, std::vector<double> &target)
{
	double totalErr = 0;
	test(input);
	for (int x = 0; x < outputs; x++)
	{
		double err = outputerr(output,target,x);
		totalErr+=err*err;
		double rateTimesError = rate*err;
		for (int y = 0; y < inputs; y++)
		{
			weight[x][y] += rateTimesError*input[y];
			
			updateData &val = updates[x][y];
			val.n++;
			//			double delta = err-val.mean;
			double delta = rateTimesError-val.mean;
			val.mean = val.mean+delta/val.n;
			val.S += delta*(err-val.mean);
			val.totErr += rateTimesError*rateTimesError;
		}
		weight[x][inputs] += rateTimesError*(1); // bias
	}
	return totalErr;
}

double *LogisticRegression::test(const std::vector<unsigned int> &input)
{
	for (int y = 0; y < outputs; y++)
	{
		output[y] = weight[y][inputs]; // bias
		for (unsigned int x = 0; x < input.size(); x++)
		{
			output[y] += weight[y][input[x]];
		}
		output[y] = g(output[y]);
	}
	return &output[0];
}

double *LogisticRegression::test(const std::vector<double> &input)
{
	for (int y = 0; y < outputs; y++)
	{
		output[y] = weight[y][inputs];
		for (int x = 0; x < inputs; x++)
		{
			output[y] += weight[y][x]*input[x];
		}
		output[y] = g(output[y]);
	}
	return &output[0];
}

void LogisticRegression::getWeightUpdateVariance(std::vector<double> &var, unsigned int which)
{
	assert(which < weight.size());
	var.resize(weight[which].size()-1);
	for (unsigned int x = 0; x < weight[which].size()-1; x++) // ignore bias!
	{
		updateData &val = updates[which][x];
		if (val.n > 1)
			var[x] = val.S/(val.n-1);
		else
			var[x] = 0;
	}
}

double LogisticRegression::getWeightUpdateVariance(unsigned int weightNum, unsigned int whichOutput)
{
	assert(whichOutput < weight.size());
	assert(weightNum < weight[whichOutput].size());
	
	updateData &val = updates[whichOutput][weightNum];
	if (val.n > 1)
		return val.S/(val.n-1);
	return 0;
}

void LogisticRegression::getWeightUpdateAverage(std::vector<double> &var, unsigned int which)
{
	assert(which < weight.size());
	var.resize(weight[which].size()-1);
	for (unsigned int x = 0; x < weight[which].size()-1; x++) // ignore bias!
	{
		updateData &val = updates[which][x];
		if (val.n > 1)
			var[x] = val.totErr/val.n;
		else
			var[x] = 0;
	}
}

double LogisticRegression::getWeightUpdateAverage(unsigned int weightNum, unsigned int whichOutput)
{
	assert(whichOutput < weight.size());
	assert(weightNum < weight[whichOutput].size());
	
	updateData &val = updates[whichOutput][weightNum];
	if (val.n > 1)
		return val.totErr/val.n;
	return 0;
}

void LogisticRegression::getWeightUpdateSum(std::vector<double> &var, unsigned int which)
{
	assert(which < weight.size());
	var.resize(weight[which].size()-1);
	for (unsigned int x = 0; x < weight[which].size()-1; x++) // ignore bias!
	{
		var[x] = updates[which][x].totErr;
	}
}

double LogisticRegression::getWeightUpdateSum(unsigned int weightNum, unsigned int whichOutput)
{
	assert(whichOutput < weight.size());
	assert(weightNum < weight[whichOutput].size());
	
	return updates[whichOutput][weightNum].totErr;
}

void LogisticRegression::resetWeightVariance(unsigned int weightNum, unsigned int whichOutput)
{
	updates[whichOutput][weightNum].reset();
}

void LogisticRegression::resetWeightVariance()
{
	for (unsigned int y = 0; y < updates.size(); y++)
	{
		for (unsigned int x = 0; x < updates[y].size(); x++)
		{
			updates[y][x].reset();
		}
	}
}

int LogisticRegression::getWeightFrequency(unsigned int weightNum, unsigned int whichOutput)
{
	return updates[whichOutput][weightNum].n;
}

void LogisticRegression::setInputWeight(double value, unsigned int weightNum, unsigned int whichOutput)
{
	updates[whichOutput][weightNum].reset();
	weight[whichOutput][weightNum] = value;
}

void LogisticRegression::Print()
{
	for (int y = 0; y < outputs; y++)
		for (int x = 0; x <= inputs; x++)
		{
			printf("%1.3f  ", weight[y][x]);
		}
	printf("\n");
}
