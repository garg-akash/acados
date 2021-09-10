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
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "ocp_model_model/ocp_model_model.h"





#include "acados_solver_ocp_model.h"

#define NX     OCP_MODEL_NX
#define NZ     OCP_MODEL_NZ
#define NU     OCP_MODEL_NU
#define NP     OCP_MODEL_NP
#define NBX    OCP_MODEL_NBX
#define NBX0   OCP_MODEL_NBX0
#define NBU    OCP_MODEL_NBU
#define NSBX   OCP_MODEL_NSBX
#define NSBU   OCP_MODEL_NSBU
#define NSH    OCP_MODEL_NSH
#define NSG    OCP_MODEL_NSG
#define NSPHI  OCP_MODEL_NSPHI
#define NSHN   OCP_MODEL_NSHN
#define NSGN   OCP_MODEL_NSGN
#define NSPHIN OCP_MODEL_NSPHIN
#define NSBXN  OCP_MODEL_NSBXN
#define NS     OCP_MODEL_NS
#define NSN    OCP_MODEL_NSN
#define NG     OCP_MODEL_NG
#define NBXN   OCP_MODEL_NBXN
#define NGN    OCP_MODEL_NGN
#define NY0    OCP_MODEL_NY0
#define NY     OCP_MODEL_NY
#define NYN    OCP_MODEL_NYN
#define N      OCP_MODEL_N
#define NH     OCP_MODEL_NH
#define NPHI   OCP_MODEL_NPHI
#define NHN    OCP_MODEL_NHN
#define NPHIN  OCP_MODEL_NPHIN
#define NR     OCP_MODEL_NR


// ** solver data **

nlp_solver_capsule * ocp_model_acados_create_capsule()
{
    void* capsule_mem = malloc(sizeof(nlp_solver_capsule));
    nlp_solver_capsule *capsule = (nlp_solver_capsule *) capsule_mem;

    return capsule;
}


int ocp_model_acados_free_capsule(nlp_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int ocp_model_acados_create(nlp_solver_capsule * capsule)
{
    int status = 0;

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    /************************************************
    *  plan & config
    ************************************************/
    ocp_nlp_plan * nlp_solver_plan = ocp_nlp_plan_create(N);
    capsule->nlp_solver_plan = nlp_solver_plan;
    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = LINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = LINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
    ocp_nlp_config * nlp_config = ocp_nlp_config_create(*nlp_solver_plan);
    capsule->nlp_config = nlp_config;


    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 29;
    ny[0] = NY0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);
    capsule->nlp_dims = nlp_dims;

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);

    for (int i = 0; i < N; i++)
    {
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);



    /************************************************
    *  external functions
    ************************************************/


    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->forw_vde_casadi[i].casadi_fun = &ocp_model_expl_vde_forw;
        capsule->forw_vde_casadi[i].casadi_n_in = &ocp_model_expl_vde_forw_n_in;
        capsule->forw_vde_casadi[i].casadi_n_out = &ocp_model_expl_vde_forw_n_out;
        capsule->forw_vde_casadi[i].casadi_sparsity_in = &ocp_model_expl_vde_forw_sparsity_in;
        capsule->forw_vde_casadi[i].casadi_sparsity_out = &ocp_model_expl_vde_forw_sparsity_out;
        capsule->forw_vde_casadi[i].casadi_work = &ocp_model_expl_vde_forw_work;
        external_function_param_casadi_create(&capsule->forw_vde_casadi[i], 0);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->expl_ode_fun[i].casadi_fun = &ocp_model_expl_ode_fun;
        capsule->expl_ode_fun[i].casadi_n_in = &ocp_model_expl_ode_fun_n_in;
        capsule->expl_ode_fun[i].casadi_n_out = &ocp_model_expl_ode_fun_n_out;
        capsule->expl_ode_fun[i].casadi_sparsity_in = &ocp_model_expl_ode_fun_sparsity_in;
        capsule->expl_ode_fun[i].casadi_sparsity_out = &ocp_model_expl_ode_fun_sparsity_out;
        capsule->expl_ode_fun[i].casadi_work = &ocp_model_expl_ode_fun_work;
        external_function_param_casadi_create(&capsule->expl_ode_fun[i], 0);
    }



    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
    capsule->nlp_in = nlp_in;

    // set up time_steps
    // all time_steps are identical
    double time_step = 0.01;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    
    }


    /**** Cost ****/

    double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 10;
    W_0[1+(NY0) * 1] = 10;
    W_0[2+(NY0) * 2] = 10;
    W_0[3+(NY0) * 3] = 10;
    W_0[4+(NY0) * 4] = 10;
    W_0[5+(NY0) * 5] = 10;
    W_0[6+(NY0) * 6] = 0.01;
    W_0[7+(NY0) * 7] = 0.01;
    W_0[8+(NY0) * 8] = 0.01;
    W_0[9+(NY0) * 9] = 0.01;
    W_0[10+(NY0) * 10] = 0.01;
    W_0[11+(NY0) * 11] = 0.01;
    // W_0[13+(NY0) * 13] = 0.0001;
    // W_0[14+(NY0) * 14] = 0.0001;
    // W_0[15+(NY0) * 15] = 0.0001;
    // W_0[16+(NY0) * 16] = 0.0001;
    // W_0[17+(NY0) * 17] = 0.0001;
    // W_0[18+(NY0) * 18] = 0.0001;
    // W_0[19+(NY0) * 19] = 0.0001;
    // W_0[20+(NY0) * 20] = 0.0001;
    // W_0[21+(NY0) * 21] = 0.0001;
    // W_0[22+(NY0) * 22] = 0.0001;
    // W_0[23+(NY0) * 23] = 0.0001;
    // W_0[24+(NY0) * 24] = 0.0001;
    // W_0[25+(NY0) * 25] = 0.0001;
    // W_0[26+(NY0) * 26] = 0.0001;
    // W_0[27+(NY0) * 27] = 0.0001;
    // W_0[28+(NY0) * 28] = 0.0001;
    // W_0[29+(NY0) * 29] = 0.0001;
    // W_0[30+(NY0) * 30] = 0.0001;
    // W_0[31+(NY0) * 31] = 0.0001;
    // W_0[32+(NY0) * 32] = 0.0001;
    // W_0[33+(NY0) * 33] = 0.0001;
    // W_0[34+(NY0) * 34] = 0.0001;
    // W_0[35+(NY0) * 35] = 0.0001;
    // W_0[36+(NY0) * 36] = 0.0001;
    // W_0[37+(NY0) * 37] = 0.0001;
    // W_0[38+(NY0) * 38] = 0.0001;
    // W_0[39+(NY0) * 39] = 0.0001;
    // W_0[40+(NY0) * 40] = 0.0001;
    // W_0[41+(NY0) * 41] = 0.0001;
    // W_0[42+(NY0) * 42] = 0.0001;
    // W_0[43+(NY0) * 43] = 0.0001;
    // W_0[44+(NY0) * 44] = 0.0001;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);

    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);



    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    
    W[0+(NY) * 0] = 10;
    W[1+(NY) * 1] = 10;
    W[2+(NY) * 2] = 10;
    W[3+(NY) * 3] = 10;
    W[4+(NY) * 4] = 10;
    W[5+(NY) * 5] = 10;
    W[6+(NY) * 6] = 0.01;
    W[7+(NY) * 7] = 0.01;
    W[8+(NY) * 8] = 0.01;
    W[9+(NY) * 9] = 0.01;
    W[10+(NY) * 10] = 0.01;
    W[11+(NY) * 11] = 0.01;
    // W[13+(NY) * 13] = 0.0001;
    // W[14+(NY) * 14] = 0.0001;
    // W[15+(NY) * 15] = 0.0001;
    // W[16+(NY) * 16] = 0.0001;
    // W[17+(NY) * 17] = 0.0001;
    // W[18+(NY) * 18] = 0.0001;
    // W[19+(NY) * 19] = 0.0001;
    // W[20+(NY) * 20] = 0.0001;
    // W[21+(NY) * 21] = 0.0001;
    // W[22+(NY) * 22] = 0.0001;
    // W[23+(NY) * 23] = 0.0001;
    // W[24+(NY) * 24] = 0.0001;
    // W[25+(NY) * 25] = 0.0001;
    // W[26+(NY) * 26] = 0.0001;
    // W[27+(NY) * 27] = 0.0001;
    // W[28+(NY) * 28] = 0.0001;
    // W[29+(NY) * 29] = 0.0001;
    // W[30+(NY) * 30] = 0.0001;
    // W[31+(NY) * 31] = 0.0001;
    // W[32+(NY) * 32] = 0.0001;
    // W[33+(NY) * 33] = 0.0001;
    // W[34+(NY) * 34] = 0.0001;
    // W[35+(NY) * 35] = 0.0001;
    // W[36+(NY) * 36] = 0.0001;
    // W[37+(NY) * 37] = 0.0001;
    // W[38+(NY) * 38] = 0.0001;
    // W[39+(NY) * 39] = 0.0001;
    // W[40+(NY) * 40] = 0.0001;
    // W[41+(NY) * 41] = 0.0001;
    // W[42+(NY) * 42] = 0.0001;
    // W[43+(NY) * 43] = 0.0001;
    // W[44+(NY) * 44] = 0.0001;

    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(W);
    free(yref);


    double* Vx_0 = calloc(NY0*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx_0[0+(NY0) * 0] = 1;
    Vx_0[1+(NY0) * 1] = 1;
    Vx_0[2+(NY0) * 2] = 1;
    Vx_0[3+(NY0) * 3] = 1;
    Vx_0[4+(NY0) * 4] = 1;
    Vx_0[5+(NY0) * 5] = 1;
    Vx_0[6+(NY0) * 6] = 1;
    Vx_0[7+(NY0) * 7] = 1;
    Vx_0[8+(NY0) * 8] = 1;
    Vx_0[9+(NY0) * 9] = 1;
    Vx_0[10+(NY0) * 10] = 1;
    Vx_0[11+(NY0) * 11] = 1;
    Vx_0[12+(NY0) * 12] = 1;
    Vx_0[13+(NY0) * 13] = 1;
    Vx_0[14+(NY0) * 14] = 1;
    Vx_0[15+(NY0) * 15] = 1;
    Vx_0[16+(NY0) * 16] = 1;
    Vx_0[17+(NY0) * 17] = 1;
    Vx_0[18+(NY0) * 18] = 1;
    Vx_0[19+(NY0) * 19] = 1;
    Vx_0[20+(NY0) * 20] = 1;
    Vx_0[21+(NY0) * 21] = 1;
    Vx_0[22+(NY0) * 22] = 1;
    Vx_0[23+(NY0) * 23] = 1;
    Vx_0[24+(NY0) * 24] = 1;
    Vx_0[25+(NY0) * 25] = 1;
    Vx_0[26+(NY0) * 26] = 1;
    Vx_0[27+(NY0) * 27] = 1;
    Vx_0[28+(NY0) * 28] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vx", Vx_0);
    free(Vx_0);


    double* Vu_0 = calloc(NY0*NU, sizeof(double));
    // change only the non-zero elements:
    
    Vu_0[29+(NY0) * 0] = 1;
    Vu_0[30+(NY0) * 1] = 1;
    Vu_0[31+(NY0) * 2] = 1;
    Vu_0[32+(NY0) * 3] = 1;
    Vu_0[33+(NY0) * 4] = 1;
    Vu_0[34+(NY0) * 5] = 1;
    Vu_0[35+(NY0) * 6] = 1;
    Vu_0[36+(NY0) * 7] = 1;
    Vu_0[37+(NY0) * 8] = 1;
    Vu_0[38+(NY0) * 9] = 1;
    Vu_0[39+(NY0) * 10] = 1;
    Vu_0[40+(NY0) * 11] = 1;
    Vu_0[41+(NY0) * 12] = 1;
    Vu_0[42+(NY0) * 13] = 1;
    Vu_0[43+(NY0) * 14] = 1;
    Vu_0[44+(NY0) * 15] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Vu", Vu_0);
    free(Vu_0);


    double* Vx = calloc(NY*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx[0+(NY) * 0] = 1;
    Vx[1+(NY) * 1] = 1;
    Vx[2+(NY) * 2] = 1;
    Vx[3+(NY) * 3] = 1;
    Vx[4+(NY) * 4] = 1;
    Vx[5+(NY) * 5] = 1;
    Vx[6+(NY) * 6] = 1;
    Vx[7+(NY) * 7] = 1;
    Vx[8+(NY) * 8] = 1;
    Vx[9+(NY) * 9] = 1;
    Vx[10+(NY) * 10] = 1;
    Vx[11+(NY) * 11] = 1;
    Vx[12+(NY) * 12] = 1;
    Vx[13+(NY) * 13] = 1;
    Vx[14+(NY) * 14] = 1;
    Vx[15+(NY) * 15] = 1;
    Vx[16+(NY) * 16] = 1;
    Vx[17+(NY) * 17] = 1;
    Vx[18+(NY) * 18] = 1;
    Vx[19+(NY) * 19] = 1;
    Vx[20+(NY) * 20] = 1;
    Vx[21+(NY) * 21] = 1;
    Vx[22+(NY) * 22] = 1;
    Vx[23+(NY) * 23] = 1;
    Vx[24+(NY) * 24] = 1;
    Vx[25+(NY) * 25] = 1;
    Vx[26+(NY) * 26] = 1;
    Vx[27+(NY) * 27] = 1;
    Vx[28+(NY) * 28] = 1;
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vx", Vx);
    }
    free(Vx);


    double* Vu = calloc(NY*NU, sizeof(double));
    // change only the non-zero elements:
    
    Vu[29+(NY) * 0] = 1;
    Vu[30+(NY) * 1] = 1;
    Vu[31+(NY) * 2] = 1;
    Vu[32+(NY) * 3] = 1;
    Vu[33+(NY) * 4] = 1;
    Vu[34+(NY) * 5] = 1;
    Vu[35+(NY) * 6] = 1;
    Vu[36+(NY) * 7] = 1;
    Vu[37+(NY) * 8] = 1;
    Vu[38+(NY) * 9] = 1;
    Vu[39+(NY) * 10] = 1;
    Vu[40+(NY) * 11] = 1;
    Vu[41+(NY) * 12] = 1;
    Vu[42+(NY) * 13] = 1;
    Vu[43+(NY) * 14] = 1;
    Vu[44+(NY) * 15] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Vu", Vu);
    }
    free(Vu);







    // terminal cost


    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    
    W_e[0+(NYN) * 0] = 10;
    W_e[1+(NYN) * 1] = 10;
    W_e[2+(NYN) * 2] = 10;
    W_e[3+(NYN) * 3] = 10;
    W_e[4+(NYN) * 4] = 10;
    W_e[5+(NYN) * 5] = 10;
    W_e[6+(NYN) * 6] = 0.01;
    W_e[7+(NYN) * 7] = 0.01;
    W_e[8+(NYN) * 8] = 0.01;
    W_e[9+(NYN) * 9] = 0.01;
    W_e[10+(NYN) * 10] = 0.01;
    W_e[11+(NYN) * 11] = 0.01;
    // W_e[13+(NYN) * 13] = 0.0001;
    // W_e[14+(NYN) * 14] = 0.0001;
    // W_e[15+(NYN) * 15] = 0.0001;
    // W_e[16+(NYN) * 16] = 0.0001;
    // W_e[17+(NYN) * 17] = 0.0001;
    // W_e[18+(NYN) * 18] = 0.0001;
    // W_e[19+(NYN) * 19] = 0.0001;
    // W_e[20+(NYN) * 20] = 0.0001;
    // W_e[21+(NYN) * 21] = 0.0001;
    // W_e[22+(NYN) * 22] = 0.0001;
    // W_e[23+(NYN) * 23] = 0.0001;
    // W_e[24+(NYN) * 24] = 0.0001;
    // W_e[25+(NYN) * 25] = 0.0001;
    // W_e[26+(NYN) * 26] = 0.0001;
    // W_e[27+(NYN) * 27] = 0.0001;
    // W_e[28+(NYN) * 28] = 0.0001;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    double* Vx_e = calloc(NYN*NX, sizeof(double));
    // change only the non-zero elements:
    
    Vx_e[0+(NYN) * 0] = 1;
    Vx_e[1+(NYN) * 1] = 1;
    Vx_e[2+(NYN) * 2] = 1;
    Vx_e[3+(NYN) * 3] = 1;
    Vx_e[4+(NYN) * 4] = 1;
    Vx_e[5+(NYN) * 5] = 1;
    Vx_e[6+(NYN) * 6] = 1;
    Vx_e[7+(NYN) * 7] = 1;
    Vx_e[8+(NYN) * 8] = 1;
    Vx_e[9+(NYN) * 9] = 1;
    Vx_e[10+(NYN) * 10] = 1;
    Vx_e[11+(NYN) * 11] = 1;
    Vx_e[12+(NYN) * 12] = 1;
    Vx_e[13+(NYN) * 13] = 1;
    Vx_e[14+(NYN) * 14] = 1;
    Vx_e[15+(NYN) * 15] = 1;
    Vx_e[16+(NYN) * 16] = 1;
    Vx_e[17+(NYN) * 17] = 1;
    Vx_e[18+(NYN) * 18] = 1;
    Vx_e[19+(NYN) * 19] = 1;
    Vx_e[20+(NYN) * 20] = 1;
    Vx_e[21+(NYN) * 21] = 1;
    Vx_e[22+(NYN) * 22] = 1;
    Vx_e[23+(NYN) * 23] = 1;
    Vx_e[24+(NYN) * 24] = 1;
    Vx_e[25+(NYN) * 25] = 1;
    Vx_e[26+(NYN) * 26] = 1;
    Vx_e[27+(NYN) * 27] = 1;
    Vx_e[28+(NYN) * 28] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "Vx", Vx_e);
    free(Vx_e);



    /**** Constraints ****/

    // bounds for initial stage

    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
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

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[0] = 0.417;
    ubx0[0] = 0.417;
    lbx0[1] = -0.417;
    ubx0[1] = -0.417;
    lbx0[2] = 0.454;
    ubx0[2] = 0.454;
    lbx0[12] = 9.8;
    ubx0[12] = 9.8;
    lbx0[13] = 0.3;
    ubx0[13] = 0.3;
    lbx0[14] = 0.3;
    ubx0[14] = 0.3;
    lbx0[15] = 0.3;
    ubx0[15] = 0.3;
    lbx0[16] = 0.3;
    ubx0[16] = 0.3;
    lbx0[17] = 0.3;
    ubx0[17] = 0.3;
    lbx0[18] = 0.3;
    ubx0[18] = 0.3;
    lbx0[19] = 0.3;
    ubx0[19] = 0.3;
    lbx0[20] = 0.3;
    ubx0[20] = 0.3;
    lbx0[21] = 0.3;
    ubx0[21] = 0.3;
    lbx0[22] = 0.3;
    ubx0[22] = 0.3;
    lbx0[23] = 0.3;
    ubx0[23] = 0.3;
    lbx0[24] = 0.3;
    ubx0[24] = 0.3;
    lbx0[25] = 0.3;
    ubx0[25] = 0.3;
    lbx0[26] = 0.3;
    ubx0[26] = 0.3;
    lbx0[27] = 0.3;
    ubx0[27] = 0.3;
    lbx0[28] = 0.3;
    ubx0[28] = 0.3;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);


    // idxbxe_0
    int* idxbxe_0 = malloc(29 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    idxbxe_0[17] = 17;
    idxbxe_0[18] = 18;
    idxbxe_0[19] = 19;
    idxbxe_0[20] = 20;
    idxbxe_0[21] = 21;
    idxbxe_0[22] = 22;
    idxbxe_0[23] = 23;
    idxbxe_0[24] = 24;
    idxbxe_0[25] = 25;
    idxbxe_0[26] = 26;
    idxbxe_0[27] = 27;
    idxbxe_0[28] = 28;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);


    /* constraints that are the same for initial and intermediate */



    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    idxbu[4] = 4;
    idxbu[5] = 5;
    idxbu[6] = 6;
    idxbu[7] = 7;
    idxbu[8] = 8;
    idxbu[9] = 9;
    idxbu[10] = 10;
    idxbu[11] = 11;
    idxbu[12] = 12;
    idxbu[13] = 13;
    idxbu[14] = 14;
    idxbu[15] = 15;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -10000;
    ubu[0] = 10000;
    lbu[1] = -10000;
    ubu[1] = 10000;
    lbu[2] = -10000;
    ubu[2] = 10000;
    lbu[3] = -10000;
    ubu[3] = 10000;
    lbu[4] = -10000;
    ubu[4] = 10000;
    lbu[5] = -10000;
    ubu[5] = 10000;
    lbu[6] = -10000;
    ubu[6] = 10000;
    lbu[7] = -10000;
    ubu[7] = 10000;
    lbu[8] = -10000;
    ubu[8] = 10000;
    lbu[9] = -10000;
    ubu[9] = 10000;
    lbu[10] = -10000;
    ubu[10] = 10000;
    lbu[11] = -10000;
    ubu[11] = 10000;
    lbu[12] = -10000;
    ubu[12] = 10000;
    lbu[13] = -10000;
    ubu[13] = 10000;
    lbu[14] = -10000;
    ubu[14] = 10000;
    lbu[15] = -10000;
    ubu[15] = 10000;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);











    // x
    int* idxbx = malloc(NBX * sizeof(int));
    
    idxbx[0] = 0;
    idxbx[1] = 1;
    idxbx[2] = 2;
    idxbx[3] = 3;
    idxbx[4] = 4;
    idxbx[5] = 5;
    idxbx[6] = 6;
    idxbx[7] = 7;
    idxbx[8] = 8;
    idxbx[9] = 9;
    idxbx[10] = 10;
    idxbx[11] = 11;
    idxbx[12] = 12;
    idxbx[13] = 13;
    idxbx[14] = 14;
    idxbx[15] = 15;
    idxbx[16] = 16;
    idxbx[17] = 17;
    idxbx[18] = 18;
    idxbx[19] = 19;
    idxbx[20] = 20;
    idxbx[21] = 21;
    idxbx[22] = 22;
    idxbx[23] = 23;
    idxbx[24] = 24;
    idxbx[25] = 25;
    idxbx[26] = 26;
    idxbx[27] = 27;
    idxbx[28] = 28;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    
    lbx[0] = -2;
    ubx[0] = 2;
    lbx[1] = -2;
    ubx[1] = 2;
    lbx[2] = -2;
    ubx[2] = 2;
    lbx[3] = -3.141592654;
    ubx[3] = 3.141592654;
    lbx[4] = -1.570796327;
    ubx[4] = 1.570796327;
    lbx[5] = -3.141592654;
    ubx[5] = 3.141592654;
    lbx[6] = -2;
    ubx[6] = 2;
    lbx[7] = -2;
    ubx[7] = 2;
    lbx[8] = -2;
    ubx[8] = 2;
    lbx[9] = -3.141592654;
    ubx[9] = 3.141592654;
    lbx[10] = -3.141592654;
    ubx[10] = 3.141592654;
    lbx[11] = -3.141592654;
    ubx[11] = 3.141592654;
    lbx[12] = 9.8;
    ubx[12] = 9.8;
    ubx[13] = 20;
    ubx[14] = 20;
    ubx[15] = 20;
    ubx[16] = 20;
    ubx[17] = 20;
    ubx[18] = 20;
    ubx[19] = 20;
    ubx[20] = 20;
    ubx[21] = 20;
    ubx[22] = 20;
    ubx[23] = 20;
    ubx[24] = 20;
    ubx[25] = 20;
    ubx[26] = 20;
    ubx[27] = 20;
    ubx[28] = 20;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);








    /* terminal constraints */

















    /************************************************
    *  opts
    ************************************************/

    capsule->nlp_opts = ocp_nlp_solver_opts_create(nlp_config, nlp_dims);


    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization", "fixed_step");

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);


    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;
    qp_solver_cond_N = 5;
    
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);


    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "qp_iter_max", &qp_solver_iter_max);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "print_level", &print_level);


    int ext_cost_num_hess = 0;


    /* out */
    ocp_nlp_out * nlp_out = ocp_nlp_out_create(nlp_config, nlp_dims);
    capsule->nlp_out = nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    
    x0[0] = 0.417;
    x0[1] = -0.417;
    x0[2] = 0.454;
    x0[12] = 9.8;
    x0[13] = 0.3;
    x0[14] = 0.3;
    x0[15] = 0.3;
    x0[16] = 0.3;
    x0[17] = 0.3;
    x0[18] = 0.3;
    x0[19] = 0.3;
    x0[20] = 0.3;
    x0[21] = 0.3;
    x0[22] = 0.3;
    x0[23] = 0.3;
    x0[24] = 0.3;
    x0[25] = 0.3;
    x0[26] = 0.3;
    x0[27] = 0.3;
    x0[28] = 0.3;


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
    
    capsule->nlp_solver = ocp_nlp_solver_create(nlp_config, nlp_dims, capsule->nlp_opts);




    status = ocp_nlp_precompute(capsule->nlp_solver, nlp_in, nlp_out);

    if (status != ACADOS_SUCCESS)
    {
        printf("\nocp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int ocp_model_acados_update_params(nlp_solver_capsule * capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 0;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    return solver_status;
}



int ocp_model_acados_solve(nlp_solver_capsule * capsule)
{
    // solve NLP 
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int ocp_model_acados_free(nlp_solver_capsule * capsule)
{
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < 10; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost

    // constraints

    return 0;
}

ocp_nlp_in *ocp_model_acados_get_nlp_in(nlp_solver_capsule * capsule) { return capsule->nlp_in; }
ocp_nlp_out *ocp_model_acados_get_nlp_out(nlp_solver_capsule * capsule) { return capsule->nlp_out; }
ocp_nlp_solver *ocp_model_acados_get_nlp_solver(nlp_solver_capsule * capsule) { return capsule->nlp_solver; }
ocp_nlp_config *ocp_model_acados_get_nlp_config(nlp_solver_capsule * capsule) { return capsule->nlp_config; }
void *ocp_model_acados_get_nlp_opts(nlp_solver_capsule * capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *ocp_model_acados_get_nlp_dims(nlp_solver_capsule * capsule) { return capsule->nlp_dims; }
ocp_nlp_plan *ocp_model_acados_get_nlp_plan(nlp_solver_capsule * capsule) { return capsule->nlp_solver_plan; }


void ocp_model_acados_print_stats(nlp_solver_capsule * capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[1000];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j > 4)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

