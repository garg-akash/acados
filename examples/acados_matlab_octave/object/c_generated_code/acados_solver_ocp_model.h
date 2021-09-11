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

#ifndef ACADOS_SOLVER_ocp_model_H_
#define ACADOS_SOLVER_ocp_model_H_

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define OCP_MODEL_NX     29
#define OCP_MODEL_NZ     0
#define OCP_MODEL_NU     16
#define OCP_MODEL_NP     78
#define OCP_MODEL_NBX    29
#define OCP_MODEL_NBX0   29
#define OCP_MODEL_NBU    16
#define OCP_MODEL_NSBX   0
#define OCP_MODEL_NSBU   0
#define OCP_MODEL_NSH    0
#define OCP_MODEL_NSG    0
#define OCP_MODEL_NSPHI  0
#define OCP_MODEL_NSHN   0
#define OCP_MODEL_NSGN   0
#define OCP_MODEL_NSPHIN 0
#define OCP_MODEL_NSBXN  0
#define OCP_MODEL_NS     0
#define OCP_MODEL_NSN    0
#define OCP_MODEL_NG     0
#define OCP_MODEL_NBXN   0
#define OCP_MODEL_NGN    0
#define OCP_MODEL_NY0    45
#define OCP_MODEL_NY     45
#define OCP_MODEL_NYN    29
#define OCP_MODEL_N      20
#define OCP_MODEL_NH     0
#define OCP_MODEL_NPHI   0
#define OCP_MODEL_NHN    0
#define OCP_MODEL_NPHIN  0
#define OCP_MODEL_NR     0

#ifdef __cplusplus
extern "C" {
#endif

// ** capsule for solver data **
typedef struct ocp_model_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *forw_vde_casadi;
    external_function_param_casadi *expl_ode_fun;




    // cost






    // constraints




} ocp_model_solver_capsule;

ocp_model_solver_capsule * ocp_model_acados_create_capsule(void);
int ocp_model_acados_free_capsule(ocp_model_solver_capsule *capsule);

int ocp_model_acados_create(ocp_model_solver_capsule * capsule);
int ocp_model_acados_update_params(ocp_model_solver_capsule * capsule, int stage, double *value, int np);
int ocp_model_acados_solve(ocp_model_solver_capsule * capsule);
int ocp_model_acados_free(ocp_model_solver_capsule * capsule);
void ocp_model_acados_print_stats(ocp_model_solver_capsule * capsule);

ocp_nlp_in *ocp_model_acados_get_nlp_in(ocp_model_solver_capsule * capsule);
ocp_nlp_out *ocp_model_acados_get_nlp_out(ocp_model_solver_capsule * capsule);
ocp_nlp_solver *ocp_model_acados_get_nlp_solver(ocp_model_solver_capsule * capsule);
ocp_nlp_config *ocp_model_acados_get_nlp_config(ocp_model_solver_capsule * capsule);
void *ocp_model_acados_get_nlp_opts(ocp_model_solver_capsule * capsule);
ocp_nlp_dims *ocp_model_acados_get_nlp_dims(ocp_model_solver_capsule * capsule);
ocp_nlp_plan *ocp_model_acados_get_nlp_plan(ocp_model_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_ocp_model_H_
