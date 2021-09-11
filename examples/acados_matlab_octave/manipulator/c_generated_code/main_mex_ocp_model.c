/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_ocp_model.h"
// mex
#include "mex.h"

/* auxilary mex */
// prints a matrix in column-major format (exponential notation)
void MEX_print_exp_mat(int m, int n, double *A, int lda)
{
	for (int i=0; i<m; i++)
    {
		for (int j=0; j<n; j++)
        {
			mexPrintf("%e\t", A[i+lda*j]);
        }
		mexPrintf("\n");
    }
	mexPrintf("\n");
}

// prints the transposed of a matrix in column-major format (exponential notation)
void MEX_print_exp_tran_mat(int row, int col, double *A, int lda)
{
	for (int j=0; j<col; j++)
    {
		for (int i=0; i<row; i++)
        {
			mexPrintf("%e\t", A[i+lda*j]);
        }
		mexPrintf("\n");
    }
	mexPrintf("\n");
}


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

    int status = 0;
    status = ocp_model_acados_create();

    if (status)
    {
        mexPrintf("ocp_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }


    // get pointers to nlp solver related objects
    ocp_nlp_config *nlp_config = ocp_model_acados_get_nlp_config();
    ocp_nlp_dims *nlp_dims = ocp_model_acados_get_nlp_dims();
    ocp_nlp_in *nlp_in = ocp_model_acados_get_nlp_in();
    ocp_nlp_out *nlp_out = ocp_model_acados_get_nlp_out();
    ocp_nlp_solver *nlp_solver = ocp_model_acados_get_nlp_solver();
    void *nlp_opts = ocp_model_acados_get_nlp_opts();

    // initial condition
    int idxbx0[21];
    
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;
    idxbx0[17] = 17;
    idxbx0[18] = 18;
    idxbx0[19] = 19;
    idxbx0[20] = 20;

    double lbx0[21];
    double ubx0[21];
    
    lbx0[0] = 0.00000000000000000000000000000039443045259999997;
    ubx0[0] = 0.00000000000000000000000000000039443045259999997;
    lbx0[1] = -23.20142861;
    ubx0[1] = -23.20142861;
    lbx0[2] = -18.62596545;
    ubx0[2] = -18.62596545;
    lbx0[3] = 0.00000000000000043859850849999996;
    ubx0[3] = 0.00000000000000043859850849999996;
    lbx0[4] = -0.00000000000000028275730229999997;
    ubx0[4] = -0.00000000000000028275730229999997;
    lbx0[5] = -4.905;
    ubx0[5] = -4.905;
    lbx0[6] = -0.00000000000000019271073919999998;
    ubx0[6] = -0.00000000000000019271073919999998;
    lbx0[7] = 0;
    ubx0[7] = 0;
    lbx0[8] = 1.570796327;
    ubx0[8] = 1.570796327;
    lbx0[9] = -1.570796327;
    ubx0[9] = -1.570796327;
    lbx0[10] = -1.204277184;
    ubx0[10] = -1.204277184;
    lbx0[11] = 1.570796327;
    ubx0[11] = 1.570796327;
    lbx0[12] = -1.570796327;
    ubx0[12] = -1.570796327;
    lbx0[13] = 0;
    ubx0[13] = 0;
    lbx0[14] = 0;
    ubx0[14] = 0;
    lbx0[15] = 0;
    ubx0[15] = 0;
    lbx0[16] = 0;
    ubx0[16] = 0;
    lbx0[17] = 0;
    ubx0[17] = 0;
    lbx0[18] = 0;
    ubx0[18] = 0;
    lbx0[19] = 0;
    ubx0[19] = 0;
    lbx0[20] = 0;
    ubx0[20] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[21];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 0.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 0.0;
    x_init[10] = 0.0;
    x_init[11] = 0.0;
    x_init[12] = 0.0;
    x_init[13] = 0.0;
    x_init[14] = 0.0;
    x_init[15] = 0.0;
    x_init[16] = 0.0;
    x_init[17] = 0.0;
    x_init[18] = 0.0;
    x_init[19] = 0.0;
    x_init[20] = 0.0;

    // initial value for control input
    double u0[7];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    u0[6] = 0.0;

    // prepare evaluation
    int NTIMINGS = 10;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[21 * (20+1)];
    double utraj[7 * (20)];

    // solve ocp in loop
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize primal solution
        for (int i = 0; i <= nlp_dims->N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
        }
        status = ocp_model_acados_solve();
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*21]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*7]);

    mexPrintf("\n--- xtraj ---\n");
    MEX_print_exp_tran_mat( 21, 20+1, xtraj, 21 );
    mexPrintf("\n--- utraj ---\n");
    MEX_print_exp_tran_mat( 7, 20, utraj, 7 );

    mexPrintf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
        mexPrintf("ocp_model_acados_solve(): SUCCESS!\n");
    else
        mexPrintf("ocp_model_acados_solve() failed with status %d.\n", status);

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    mexPrintf("\nSolver info:\n");
    mexPrintf(" SQP iterations %2d\n minimum time for 1 solve %f [ms]\n KKT %e\n",
           sqp_iter, min_time*1000, kkt_norm_inf);

    // free solver
    status = ocp_model_acados_free();
    if (status)
    {
        mexPrintf("ocp_model_acados_free() returned status %d.\n", status);
    }

    return;
}
