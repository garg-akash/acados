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
#include <time.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_ocp_model.h"
// sim
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_ocp_model.h"


int main()
{   
    FILE *outfile;
    outfile = fopen("./output.txt", "w");
    if ( outfile == NULL ) {
        printf("Unable to open file."); 
        exit(1);
    }

    double Tsim = 0.01;
    double y_ref[45];
    FILE *fp;
    const int n_rows = 251;
    const int n_cols = 45;
    int row_ref; //to track which row of reference traj to take
    double ref_traj[n_rows][n_cols];

    fp = fopen("ref2.txt", "r");
    if (fp == NULL) {
        fprintf(stderr, "Error: Cannot open file ref.txt for reading\n");
        exit(1);
    }
    // printf("ref_traj:\n");
    for (int i = 0; i < n_rows; i++) {
        for (int j = 0; j < n_cols; j++) {
            fscanf(fp, "%0.5f", &ref_traj[i][j]);
            if (fscanf(fp, "%lf", &ref_traj[i][j]) != 1) {
                fprintf(stderr, "invalid input for ref_traj[%lf][%lf]\n", i, j);
                fclose(fp);
                exit(1);
            }
            // printf("%0.5f  ", ref_traj[i][j]);
        }
        // printf("\n");
    }
    fclose(fp);

    nlp_solver_capsule *acados_ocp_capsule = ocp_model_acados_create_capsule();
    int status = ocp_model_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("ocp_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = ocp_model_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = ocp_model_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = ocp_model_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = ocp_model_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = ocp_model_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = ocp_model_acados_get_nlp_opts(acados_ocp_capsule);

    // sim
    int status_sim = 0;
    sim_solver_capsule *capsule = ocp_model_acados_sim_solver_create_capsule();
    status_sim = ocp_model_acados_sim_create(capsule);

    if (status_sim)
    {
        printf("acados_create() returned status %d. Exiting.\n", status_sim);
        exit(1);
    }

    sim_config *acados_sim_config = ocp_model_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = ocp_model_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = ocp_model_acados_get_sim_out(capsule);
    void *acados_sim_dims = ocp_model_acados_get_sim_dims(capsule);

    // initial condition
    int idxbx0[29];
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
    idxbx0[21] = 21;
    idxbx0[22] = 22;
    idxbx0[23] = 23;
    idxbx0[24] = 24;
    idxbx0[25] = 25;
    idxbx0[26] = 26;
    idxbx0[27] = 27;
    idxbx0[28] = 28;

    double lbx0[29];
    double ubx0[29];
    // lbx0[0] = 0.417;
    // ubx0[0] = 0.417;
    // lbx0[1] = -0.417;
    // ubx0[1] = -0.417;
    // lbx0[2] = 0.454;
    // ubx0[2] = 0.454;
    // lbx0[3] = 0;
    // ubx0[3] = 0;
    // lbx0[4] = 0;
    // ubx0[4] = 0;
    // lbx0[5] = 0;
    // ubx0[5] = 0;
    // lbx0[6] = 0;
    // ubx0[6] = 0;
    // lbx0[7] = 0;
    // ubx0[7] = 0;
    // lbx0[8] = 0;
    // ubx0[8] = 0;
    // lbx0[9] = 0;
    // ubx0[9] = 0;
    // lbx0[10] = 0;
    // ubx0[10] = 0;
    // lbx0[11] = 0;
    // ubx0[11] = 0;
    // lbx0[12] = 9.8;
    // ubx0[12] = 9.8;
    // lbx0[13] = 0.3;
    // ubx0[13] = 0.3;
    // lbx0[14] = 0.3;
    // ubx0[14] = 0.3;
    // lbx0[15] = 0.3;
    // ubx0[15] = 0.3;
    // lbx0[16] = 0.3;
    // ubx0[16] = 0.3;
    // lbx0[17] = 0.3;
    // ubx0[17] = 0.3;
    // lbx0[18] = 0.3;
    // ubx0[18] = 0.3;
    // lbx0[19] = 0.3;
    // ubx0[19] = 0.3;
    // lbx0[20] = 0.3;
    // ubx0[20] = 0.3;
    // lbx0[21] = 0.3;
    // ubx0[21] = 0.3;
    // lbx0[22] = 0.3;
    // ubx0[22] = 0.3;
    // lbx0[23] = 0.3;
    // ubx0[23] = 0.3;
    // lbx0[24] = 0.3;
    // ubx0[24] = 0.3;
    // lbx0[25] = 0.3;
    // ubx0[25] = 0.3;
    // lbx0[26] = 0.3;
    // ubx0[26] = 0.3;
    // lbx0[27] = 0.3;
    // ubx0[27] = 0.3;
    // lbx0[28] = 0.3;
    // ubx0[28] = 0.3;

    // initialization for state values
    double x_init[29];
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
    x_init[21] = 0.0;
    x_init[22] = 0.0;
    x_init[23] = 0.0;
    x_init[24] = 0.0;
    x_init[25] = 0.0;
    x_init[26] = 0.0;
    x_init[27] = 0.0;
    x_init[28] = 0.0;

    // initial value for control input
    double u_init[16];
    u_init[0] = 0.0;
    u_init[1] = 0.0;
    u_init[2] = 0.0;
    u_init[3] = 0.0;
    u_init[4] = 0.0;
    u_init[5] = 0.0;
    u_init[6] = 0.0;
    u_init[7] = 0.0;
    u_init[8] = 0.0;
    u_init[9] = 0.0;
    u_init[10] = 0.0;
    u_init[11] = 0.0;
    u_init[12] = 0.0;
    u_init[13] = 0.0;
    u_init[14] = 0.0;
    u_init[15] = 0.0;

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double total_time = 0;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[29 * (10+1)]; // used to keep the per iteration solution of the ocp
    double utraj[16 * (10)];

    // extra code
    double x0[29], u0[16];
    // double X0[29* * (10+1)];
    for (int j = 0; j < 29; ++j)
        x0[j] = ref_traj[0][j];

    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

    // solve ocp in loop
    int rti_phase = 0;

    for (int i = 0; i <= nlp_dims->N; i++)
    {
        for (int j = 0; j < 29; ++j)
            xtraj[i*29+j] = ref_traj[i][j];
        if (i < nlp_dims->N) {
            for (int j = 0; j < 16; ++j)
                utraj[i*16+j] = ref_traj[i][j+29];
        }
    }
    /*printf("printing x_traj\n");
    for (int j = 0; j < 29*11; ++j)
    {
        printf("%e\n", xtraj[j]);
    }*/

    // n_iterations start from here
    for (int n_iter = 0; n_iter < 251; ++n_iter)
    {
    printf("Iter no:%d\n", n_iter);
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // constraint x0
        for (int j = 0; j < 29; ++j) {
            lbx0[j] = x0[j];
            ubx0[j] = x0[j];
        }
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);

        // initialization for state and control for next iteration
        for (int i = 0; i < nlp_dims->N; i++)
        {
            if (i != nlp_dims->N-1) {
                for (int j = 0; j < 29; ++j)
                    x_init[j] = xtraj[(i+1)*29+j];

                for (int j = 0; j < 16; ++j)
                    u_init[j] = utraj[(i+1)*16+j];
            }
            else { 
                for (int j = 0; j < 29; ++j)
                    x_init[j] = xtraj[i*29+j];
                
                for (int j = 0; j < 16; ++j)
                    u_init[j] = utraj[i*16+j];
            }
            /*printf("printing x_init\n");
            for (int j = 0; j < 29; ++j)
            {
                printf("%0.5f\n", x_init[j]);
            }*/

            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u_init);

            if (i+n_iter >= n_rows) 
                row_ref = n_rows-1;
            else
                row_ref = i+n_iter;

            for (int j = 0; j < n_cols; ++j)
                y_ref[j] = ref_traj[row_ref][j];
            printf("reff%0.5f%0.5f%0.5f\n", y_ref[0], y_ref[1], y_ref[2]);

            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "y_ref", y_ref);
        }

        for (int j = 0; j < 29; ++j)
            x_init[j] = xtraj[nlp_dims->N*29+j];

        if (nlp_dims->N+n_iter >= n_rows) 
            row_ref = n_rows-1;
        else
            row_ref = nlp_dims->N+n_iter;
        
        for (int j = 0; j < n_cols; ++j)
            y_ref[j] = ref_traj[row_ref][j];
        printf("Ref at end:%0.5f %0.5f %0.5f\n", y_ref[0], y_ref[1], y_ref[2]);
        
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_dims->N, "x", x_init);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, nlp_dims->N, "y_ref", y_ref);
        // ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, nlp_dims->N, "yref_e", y_ref);
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
        status = ocp_model_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*29]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*16]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( 29, 10+1, xtraj, 29 );
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( 16, 10, utraj, 16 );
    // ocp_nlp_out_print(nlp_solver->dims, nlp_out);

    // extract first control solution
    for (int j = 0; j < 16; ++j)
        u0[j] = utraj[j];
    /*printf("printing u0\n");
    for (int j = 0; j < 16; ++j)
    {
        printf("%0.5f\n", u0[j]);
    }*/


    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("ocp_model_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("ocp_model_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);

    ocp_model_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);

    // extra code [initialization for next iteration]
    /*for (int i = 0; i <= nlp_dims->N; ++i)
    {   
        if (i < nlp_dims->N)
            x_init[i*29] = xtraj[(i+1)*29];
        else
            x_init[i*29] = xtraj[i*29];
    }*/

    //sim
    int n_sim_steps = 1;
    // double x_current[29];
    // solve sim in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x0);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "u", u0);
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "T", &Tsim);

        status_sim = ocp_model_acados_sim_solve(capsule);

        if (status_sim != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status_sim);
        }

        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x0);
        
        printf("\nx current, %d\n", ii);
        for (int jj = 0; jj < 29; jj++)
        {
            printf("%0.5f\n", x0[jj]);
            fprintf(outfile, "%e\t", x0[jj]); 
        }

        fprintf(outfile, "\n"); 

        printf("\nu current, %d\n", ii);
        for (int jj = 0; jj < 16; jj++)
        {
            printf("%e\n", u0[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    } //n_iter loop ends

    fclose(outfile);
    // free sim solver
    /*status_sim = ocp_model_acados_sim_free(capsule);
    if (status_sim) {
        printf("ocp_model_acados_sim_free() returned status %d. \n", status_sim);
    }*/

    // free solver
    status = ocp_model_acados_free(acados_ocp_capsule);
    if (status) {
        printf("ocp_model_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = ocp_model_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("ocp_model_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
