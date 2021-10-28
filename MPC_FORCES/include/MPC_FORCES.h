/*
Header file containing definitions for C interface of MPC_FORCES,
 a fast costumized optimization solver.
*/

#ifndef MPC_FORCES_H
#define MPC_FORCES_H

#include <stdio.h>

/* For Visual Studio 2015 Compatibility */
#if (_MSC_VER >= 1900)
FILE * __cdecl __iob_func(void);
#endif
/* DATA TYPE ------------------------------------------------------------*/
typedef double MPC_FORCES_float;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef SET_PRINTLEVEL_MPC_FORCES
#define SET_PRINTLEVEL_MPC_FORCES    (2)
#endif

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
	/* column vector of length 3 */
	MPC_FORCES_float t_init[3];

} MPC_FORCES_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
	/* column vector of length 3 */
	MPC_FORCES_float p0[3];

} MPC_FORCES_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
	/* iteration number */
	solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;

	/* inf-norm of equality constraint residuals */
	MPC_FORCES_float res_eq;

	/* inf-norm of inequality constraint residuals */
	MPC_FORCES_float res_ineq;

	/* primal objective */
	MPC_FORCES_float pobj;

	/* dual objective */
	MPC_FORCES_float dobj;

	/* duality gap := pobj - dobj */
	MPC_FORCES_float dgap;

	/* relative duality gap := |dgap / pobj | */
	MPC_FORCES_float rdgap;

	/* duality measure */
	MPC_FORCES_float mu;

	/* duality measure (after affine step) */
	MPC_FORCES_float mu_aff;

	/* centering parameter */
	MPC_FORCES_float sigma;

	/* number of backtracking line search steps (affine direction) */
	solver_int32_default lsit_aff;

	/* number of backtracking line search steps (combined direction) */
	solver_int32_default lsit_cc;

	/* step size (affine direction) */
	MPC_FORCES_float step_aff;

	/* step size (combined direction) */
	MPC_FORCES_float step_cc;

	/* solvertime */
	MPC_FORCES_float solvetime;

} MPC_FORCES_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* examine exitflag before using the result! */
extern solver_int32_default MPC_FORCES_solve(MPC_FORCES_params *params, MPC_FORCES_output *output, MPC_FORCES_info *info, FILE *fs);

#ifdef __cplusplus
}
#endif

#endif /* MPC_FORCES_H */
