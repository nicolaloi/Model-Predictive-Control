/*
 This is an interface for MPC_FORCES that is used by optimizerFORCES to call the solver
*/ 

#include "mex.h"
#include "math.h"
#include "../include/MPC_FORCES.h"
#include "../include/MPC_FORCES.h"
#include <stdio.h>

/* copy functions */
void copyCArrayToM(double *src, double *dest, int dim) {
	while (dim--) {
		*dest++ = (double)*src++;
	}
}
void copyMArrayToC(double *src, double *dest, int dim) {
	while (dim--) {
		*dest++ = (double) (*src++) ;
	}
}

/* Some memory for mex-function */
static MPC_FORCES_params params;
static MPC_FORCES_output output;
static MPC_FORCES_info info;

/* THE mex-function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
	/* file pointer for printing */
	FILE *fp = NULL;

	/* define variables */
	mxArray *par;

	const mxArray *param_values = prhs[0];
	mxArray *outvar;
	int i;
	int exitflag;
	const char *fname;

	const char *infofields[16] = { "it", "it2opt", "res_eq", "res_ineq",  "pobj",  "dobj",  "dgap", "rdgap",  "mu",  "mu_aff",  "sigma",  "lsit_aff",  "lsit_cc",  "step_aff",   "step_cc",  "solvetime"};

	/* Check for proper number of arguments */
	if (nrhs != 1) {
		mexErrMsgTxt("This function requires exactly 1 input: parameter value cell array.");
	}
	if (nlhs > 3) {
		mexErrMsgTxt("This function returns at most 3 outputs.");
	}
	/* Check whether params is actually a cell array */
	if( !mxIsCell(param_values) ) {
		mexErrMsgTxt("MPC_FORCES requires a cell array as input.");
	}

	/* Check whether params has the right number of elements */
	if( mxGetNumberOfElements(param_values) != 1 ) {
		mexErrMsgTxt("Input must have 1 elements.");
	}

	/* Load param values into C arrays and check their size */
	par = mxGetCell(param_values,0);
	if( par == NULL ) {
		mexErrMsgTxt("Parameter #1 not found");
	}
	if( !mxIsDouble(par) ) {
		mexErrMsgTxt("Parameter #1 must be a double.");
	}
	if( mxGetM(par) != 3 || mxGetN(par) != 1 ) {
		mexErrMsgTxt("Parameter #1 must be of size [3 x 1]");
	}
	copyMArrayToC(mxGetPr(par),params.t_init,3);

	#if SET_PRINTLEVEL_MPC_FORCES > 0
		/* Prepare file for printfs */
		/*fp = freopen("stdout_temp","w+",stdout);*/
		fp = fopen("stdout_temp","w+");
		if( fp == NULL ) {
			mexErrMsgTxt("freopen of stdout did not work.");
		}
		rewind(fp);
	#endif

	/* call solver */
	exitflag = MPC_FORCES_solve(&params, &output, &info, fp );

	#if SET_PRINTLEVEL_MPC_FORCES > 0
		/* Read contents of printfs printed to file */
		rewind(fp);
		while( (i = fgetc(fp)) != EOF ) {
			mexPrintf("%c",i);
		}
		fclose(fp);
	#endif

	plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
	copyCArrayToM(output.p0, mxGetPr(plhs[0]), 3);

	/* copy exitflag */
	if( nlhs > 1 ) {
		plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
		*(mxGetPr(plhs[1])) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 ) {
		plhs[2] = mxCreateStructMatrix(1, 1, 16, infofields);

		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);

		/* iterations to optimality (branch and bound) */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it2opt;
		mxSetField(plhs[2], 0, "it2opt", outvar);

		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dobj;
		mxSetField(plhs[2], 0, "dobj", outvar);

		/* dgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dgap;
		mxSetField(plhs[2], 0, "dgap", outvar);

		/* rdgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rdgap;
		mxSetField(plhs[2], 0, "rdgap", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* mu_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu_aff;
		mxSetField(plhs[2], 0, "mu_aff", outvar);

		/* sigma */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.sigma;
		mxSetField(plhs[2], 0, "sigma", outvar);

		/* lsit_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_aff;
		mxSetField(plhs[2], 0, "lsit_aff", outvar);

		/* lsit_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_cc;
		mxSetField(plhs[2], 0, "lsit_cc", outvar);

		/* step_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_aff;
		mxSetField(plhs[2], 0, "step_aff", outvar);

		/* step_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_cc;
		mxSetField(plhs[2], 0, "step_cc", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);
	}
}