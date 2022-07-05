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
#include "drone_insect_ode_model/drone_insect_ode_model.h"




#include "drone_insect_ode_constraints/drone_insect_ode_h_e_constraint.h"

#include "drone_insect_ode_cost/drone_insect_ode_external_cost.h"
#include "drone_insect_ode_cost/drone_insect_ode_external_cost_0.h"
#include "drone_insect_ode_cost/drone_insect_ode_external_cost_e.h"

#include "acados_solver_drone_insect_ode.h"

#define NX     DRONE_INSECT_ODE_NX
#define NZ     DRONE_INSECT_ODE_NZ
#define NU     DRONE_INSECT_ODE_NU
#define NP     DRONE_INSECT_ODE_NP
#define NBX    DRONE_INSECT_ODE_NBX
#define NBX0   DRONE_INSECT_ODE_NBX0
#define NBU    DRONE_INSECT_ODE_NBU
#define NSBX   DRONE_INSECT_ODE_NSBX
#define NSBU   DRONE_INSECT_ODE_NSBU
#define NSH    DRONE_INSECT_ODE_NSH
#define NSG    DRONE_INSECT_ODE_NSG
#define NSPHI  DRONE_INSECT_ODE_NSPHI
#define NSHN   DRONE_INSECT_ODE_NSHN
#define NSGN   DRONE_INSECT_ODE_NSGN
#define NSPHIN DRONE_INSECT_ODE_NSPHIN
#define NSBXN  DRONE_INSECT_ODE_NSBXN
#define NS     DRONE_INSECT_ODE_NS
#define NSN    DRONE_INSECT_ODE_NSN
#define NG     DRONE_INSECT_ODE_NG
#define NBXN   DRONE_INSECT_ODE_NBXN
#define NGN    DRONE_INSECT_ODE_NGN
#define NY0    DRONE_INSECT_ODE_NY0
#define NY     DRONE_INSECT_ODE_NY
#define NYN    DRONE_INSECT_ODE_NYN
// #define N      DRONE_INSECT_ODE_N
#define NH     DRONE_INSECT_ODE_NH
#define NPHI   DRONE_INSECT_ODE_NPHI
#define NHN    DRONE_INSECT_ODE_NHN
#define NPHIN  DRONE_INSECT_ODE_NPHIN
#define NR     DRONE_INSECT_ODE_NR


// ** solver data **

drone_insect_ode_solver_capsule * drone_insect_ode_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(drone_insect_ode_solver_capsule));
    drone_insect_ode_solver_capsule *capsule = (drone_insect_ode_solver_capsule *) capsule_mem;

    return capsule;
}


int drone_insect_ode_acados_free_capsule(drone_insect_ode_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int drone_insect_ode_acados_create(drone_insect_ode_solver_capsule * capsule)
{
    int N_shooting_intervals = DRONE_INSECT_ODE_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return drone_insect_ode_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}

int drone_insect_ode_acados_update_time_steps(drone_insect_ode_solver_capsule * capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "drone_insect_ode_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

int drone_insect_ode_acados_create_with_discretization(drone_insect_ode_solver_capsule * capsule, int N, double* new_time_steps)
{
    int status = 0;
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != DRONE_INSECT_ODE_N && !new_time_steps) {
        fprintf(stderr, "drone_insect_ode_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, DRONE_INSECT_ODE_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    /************************************************
    *  plan & config
    ************************************************/
    ocp_nlp_plan * nlp_solver_plan = ocp_nlp_plan_create(N);
    capsule->nlp_solver_plan = nlp_solver_plan;
    nlp_solver_plan->nlp_solver = SQP;
    

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = FULL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

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
    nbxe[0] = 12;
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

    for (int i = 0; i < N; i++)
    {
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);

    free(intNp1mem);



    /************************************************
    *  external functions
    ************************************************/
    capsule->nl_constr_h_e_fun_jac.casadi_fun = &drone_insect_ode_constr_h_e_fun_jac_uxt_zt;
    capsule->nl_constr_h_e_fun_jac.casadi_n_in = &drone_insect_ode_constr_h_e_fun_jac_uxt_zt_n_in;
    capsule->nl_constr_h_e_fun_jac.casadi_n_out = &drone_insect_ode_constr_h_e_fun_jac_uxt_zt_n_out;
    capsule->nl_constr_h_e_fun_jac.casadi_sparsity_in = &drone_insect_ode_constr_h_e_fun_jac_uxt_zt_sparsity_in;
    capsule->nl_constr_h_e_fun_jac.casadi_sparsity_out = &drone_insect_ode_constr_h_e_fun_jac_uxt_zt_sparsity_out;
    capsule->nl_constr_h_e_fun_jac.casadi_work = &drone_insect_ode_constr_h_e_fun_jac_uxt_zt_work;
    external_function_param_casadi_create(&capsule->nl_constr_h_e_fun_jac, 0);

    capsule->nl_constr_h_e_fun.casadi_fun = &drone_insect_ode_constr_h_e_fun;
    capsule->nl_constr_h_e_fun.casadi_n_in = &drone_insect_ode_constr_h_e_fun_n_in;
    capsule->nl_constr_h_e_fun.casadi_n_out = &drone_insect_ode_constr_h_e_fun_n_out;
    capsule->nl_constr_h_e_fun.casadi_sparsity_in = &drone_insect_ode_constr_h_e_fun_sparsity_in;
    capsule->nl_constr_h_e_fun.casadi_sparsity_out = &drone_insect_ode_constr_h_e_fun_sparsity_out;
    capsule->nl_constr_h_e_fun.casadi_work = &drone_insect_ode_constr_h_e_fun_work;
    external_function_param_casadi_create(&capsule->nl_constr_h_e_fun, 0);

    


    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->forw_vde_casadi[i].casadi_fun = &drone_insect_ode_expl_vde_forw;
        capsule->forw_vde_casadi[i].casadi_n_in = &drone_insect_ode_expl_vde_forw_n_in;
        capsule->forw_vde_casadi[i].casadi_n_out = &drone_insect_ode_expl_vde_forw_n_out;
        capsule->forw_vde_casadi[i].casadi_sparsity_in = &drone_insect_ode_expl_vde_forw_sparsity_in;
        capsule->forw_vde_casadi[i].casadi_sparsity_out = &drone_insect_ode_expl_vde_forw_sparsity_out;
        capsule->forw_vde_casadi[i].casadi_work = &drone_insect_ode_expl_vde_forw_work;
        external_function_param_casadi_create(&capsule->forw_vde_casadi[i], 0);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        capsule->expl_ode_fun[i].casadi_fun = &drone_insect_ode_expl_ode_fun;
        capsule->expl_ode_fun[i].casadi_n_in = &drone_insect_ode_expl_ode_fun_n_in;
        capsule->expl_ode_fun[i].casadi_n_out = &drone_insect_ode_expl_ode_fun_n_out;
        capsule->expl_ode_fun[i].casadi_sparsity_in = &drone_insect_ode_expl_ode_fun_sparsity_in;
        capsule->expl_ode_fun[i].casadi_sparsity_out = &drone_insect_ode_expl_ode_fun_sparsity_out;
        capsule->expl_ode_fun[i].casadi_work = &drone_insect_ode_expl_ode_fun_work;
        external_function_param_casadi_create(&capsule->expl_ode_fun[i], 0);
    }


    // external cost
    
    capsule->ext_cost_0_fun.casadi_fun = &drone_insect_ode_cost_ext_cost_0_fun;
    capsule->ext_cost_0_fun.casadi_n_in = &drone_insect_ode_cost_ext_cost_0_fun_n_in;
    capsule->ext_cost_0_fun.casadi_n_out = &drone_insect_ode_cost_ext_cost_0_fun_n_out;
    capsule->ext_cost_0_fun.casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_0_fun_sparsity_in;
    capsule->ext_cost_0_fun.casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_0_fun_sparsity_out;
    capsule->ext_cost_0_fun.casadi_work = &drone_insect_ode_cost_ext_cost_0_fun_work;
    
    external_function_param_casadi_create(&capsule->ext_cost_0_fun, 0);

    // external cost
    
    capsule->ext_cost_0_fun_jac.casadi_fun = &drone_insect_ode_cost_ext_cost_0_fun_jac;
    capsule->ext_cost_0_fun_jac.casadi_n_in = &drone_insect_ode_cost_ext_cost_0_fun_jac_n_in;
    capsule->ext_cost_0_fun_jac.casadi_n_out = &drone_insect_ode_cost_ext_cost_0_fun_jac_n_out;
    capsule->ext_cost_0_fun_jac.casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_0_fun_jac_sparsity_in;
    capsule->ext_cost_0_fun_jac.casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_0_fun_jac_sparsity_out;
    capsule->ext_cost_0_fun_jac.casadi_work = &drone_insect_ode_cost_ext_cost_0_fun_jac_work;
    
    external_function_param_casadi_create(&capsule->ext_cost_0_fun_jac, 0);

    // external cost
    
    capsule->ext_cost_0_fun_jac_hess.casadi_fun = &drone_insect_ode_cost_ext_cost_0_fun_jac_hess;
    capsule->ext_cost_0_fun_jac_hess.casadi_n_in = &drone_insect_ode_cost_ext_cost_0_fun_jac_hess_n_in;
    capsule->ext_cost_0_fun_jac_hess.casadi_n_out = &drone_insect_ode_cost_ext_cost_0_fun_jac_hess_n_out;
    capsule->ext_cost_0_fun_jac_hess.casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_0_fun_jac_hess_sparsity_in;
    capsule->ext_cost_0_fun_jac_hess.casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_0_fun_jac_hess_sparsity_out;
    capsule->ext_cost_0_fun_jac_hess.casadi_work = &drone_insect_ode_cost_ext_cost_0_fun_jac_hess_work;
    
    external_function_param_casadi_create(&capsule->ext_cost_0_fun_jac_hess, 0);
    // external cost
    capsule->ext_cost_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        
        capsule->ext_cost_fun[i].casadi_fun = &drone_insect_ode_cost_ext_cost_fun;
        capsule->ext_cost_fun[i].casadi_n_in = &drone_insect_ode_cost_ext_cost_fun_n_in;
        capsule->ext_cost_fun[i].casadi_n_out = &drone_insect_ode_cost_ext_cost_fun_n_out;
        capsule->ext_cost_fun[i].casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_fun_sparsity_in;
        capsule->ext_cost_fun[i].casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_fun_sparsity_out;
        capsule->ext_cost_fun[i].casadi_work = &drone_insect_ode_cost_ext_cost_fun_work;
        
        external_function_param_casadi_create(&capsule->ext_cost_fun[i], 0);
    }

    capsule->ext_cost_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        
        capsule->ext_cost_fun_jac[i].casadi_fun = &drone_insect_ode_cost_ext_cost_fun_jac;
        capsule->ext_cost_fun_jac[i].casadi_n_in = &drone_insect_ode_cost_ext_cost_fun_jac_n_in;
        capsule->ext_cost_fun_jac[i].casadi_n_out = &drone_insect_ode_cost_ext_cost_fun_jac_n_out;
        capsule->ext_cost_fun_jac[i].casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_fun_jac_sparsity_in;
        capsule->ext_cost_fun_jac[i].casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_fun_jac_sparsity_out;
        capsule->ext_cost_fun_jac[i].casadi_work = &drone_insect_ode_cost_ext_cost_fun_jac_work;
        
        external_function_param_casadi_create(&capsule->ext_cost_fun_jac[i], 0);
    }

    capsule->ext_cost_fun_jac_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        
        capsule->ext_cost_fun_jac_hess[i].casadi_fun = &drone_insect_ode_cost_ext_cost_fun_jac_hess;
        capsule->ext_cost_fun_jac_hess[i].casadi_n_in = &drone_insect_ode_cost_ext_cost_fun_jac_hess_n_in;
        capsule->ext_cost_fun_jac_hess[i].casadi_n_out = &drone_insect_ode_cost_ext_cost_fun_jac_hess_n_out;
        capsule->ext_cost_fun_jac_hess[i].casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_fun_jac_hess_sparsity_in;
        capsule->ext_cost_fun_jac_hess[i].casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_fun_jac_hess_sparsity_out;
        capsule->ext_cost_fun_jac_hess[i].casadi_work = &drone_insect_ode_cost_ext_cost_fun_jac_hess_work;
        
        external_function_param_casadi_create(&capsule->ext_cost_fun_jac_hess[i], 0);
    }
    // external cost
    
    capsule->ext_cost_e_fun.casadi_fun = &drone_insect_ode_cost_ext_cost_e_fun;
    capsule->ext_cost_e_fun.casadi_n_in = &drone_insect_ode_cost_ext_cost_e_fun_n_in;
    capsule->ext_cost_e_fun.casadi_n_out = &drone_insect_ode_cost_ext_cost_e_fun_n_out;
    capsule->ext_cost_e_fun.casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_e_fun_sparsity_in;
    capsule->ext_cost_e_fun.casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_e_fun_sparsity_out;
    capsule->ext_cost_e_fun.casadi_work = &drone_insect_ode_cost_ext_cost_e_fun_work;
    
    external_function_param_casadi_create(&capsule->ext_cost_e_fun, 0);

    // external cost
    
    capsule->ext_cost_e_fun_jac.casadi_fun = &drone_insect_ode_cost_ext_cost_e_fun_jac;
    capsule->ext_cost_e_fun_jac.casadi_n_in = &drone_insect_ode_cost_ext_cost_e_fun_jac_n_in;
    capsule->ext_cost_e_fun_jac.casadi_n_out = &drone_insect_ode_cost_ext_cost_e_fun_jac_n_out;
    capsule->ext_cost_e_fun_jac.casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_e_fun_jac_sparsity_in;
    capsule->ext_cost_e_fun_jac.casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_e_fun_jac_sparsity_out;
    capsule->ext_cost_e_fun_jac.casadi_work = &drone_insect_ode_cost_ext_cost_e_fun_jac_work;
    
    external_function_param_casadi_create(&capsule->ext_cost_e_fun_jac, 0);

    // external cost
    
    capsule->ext_cost_e_fun_jac_hess.casadi_fun = &drone_insect_ode_cost_ext_cost_e_fun_jac_hess;
    capsule->ext_cost_e_fun_jac_hess.casadi_n_in = &drone_insect_ode_cost_ext_cost_e_fun_jac_hess_n_in;
    capsule->ext_cost_e_fun_jac_hess.casadi_n_out = &drone_insect_ode_cost_ext_cost_e_fun_jac_hess_n_out;
    capsule->ext_cost_e_fun_jac_hess.casadi_sparsity_in = &drone_insect_ode_cost_ext_cost_e_fun_jac_hess_sparsity_in;
    capsule->ext_cost_e_fun_jac_hess.casadi_sparsity_out = &drone_insect_ode_cost_ext_cost_e_fun_jac_hess_sparsity_out;
    capsule->ext_cost_e_fun_jac_hess.casadi_work = &drone_insect_ode_cost_ext_cost_e_fun_jac_hess_work;
    
    external_function_param_casadi_create(&capsule->ext_cost_e_fun_jac_hess, 0);

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
    capsule->nlp_in = nlp_in;

    // set up time_steps
    

    if (new_time_steps) {
        drone_insect_ode_acados_update_time_steps(capsule, N, new_time_steps);
    } else {// all time_steps are identical
        double time_step = 1;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
    
    }


    /**** Cost ****/
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
    }




    // terminal cost

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);



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

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[6] = -0.746902;
    ubx0[6] = -0.746902;
    lbx0[7] = -0.659168;
    ubx0[7] = -0.659168;
    lbx0[8] = -1.38966;
    ubx0[8] = -1.38966;
    lbx0[9] = 1.18392;
    ubx0[9] = 1.18392;
    lbx0[10] = -1.32707;
    ubx0[10] = -1.32707;
    lbx0[11] = -1.43181;
    ubx0[11] = -1.43181;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);


    // idxbxe_0
    int* idxbxe_0 = malloc(12 * sizeof(int));
    
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
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);


    /* constraints that are the same for initial and intermediate */



    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    ubu[0] = 999;
    lbu[1] = -30;
    ubu[1] = 30;
    lbu[2] = -30;
    ubu[2] = 30;
    lbu[3] = -30;
    ubu[3] = 30;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);


















    /* terminal constraints */













    // set up nonlinear constraints for last stage
    double* luh_e = calloc(2*NHN, sizeof(double));
    double* lh_e = luh_e;
    double* uh_e = luh_e + NHN;
    
    lh_e[0] = -0.01;
    lh_e[1] = -0.01;
    lh_e[2] = -0.01;

    
    uh_e[0] = 0.01;
    uh_e[1] = 0.01;
    uh_e[2] = 0.01;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac", &capsule->nl_constr_h_e_fun_jac);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun", &capsule->nl_constr_h_e_fun);
    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lh", lh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "uh", uh_e);
    free(luh_e);




    /************************************************
    *  opts
    ************************************************/

    capsule->nlp_opts = ocp_nlp_solver_opts_create(nlp_config, nlp_dims);


    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization", "fixed_step");

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 15;
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

    double levenberg_marquardt = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */

    int qp_solver_iter_max = 100;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "qp_iter_max", &qp_solver_iter_max);
    // set SQP specific options
    double nlp_solver_tol_stat = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 5000;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "max_iter", &nlp_solver_max_iter);

    int initialize_t_slacks = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "initialize_t_slacks", &initialize_t_slacks);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "print_level", &print_level);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, capsule->nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);


    /* out */
    ocp_nlp_out * nlp_out = ocp_nlp_out_create(nlp_config, nlp_dims);
    capsule->nlp_out = nlp_out;

    /* sens_out */
    ocp_nlp_out *sens_out = ocp_nlp_out_create(nlp_config, nlp_dims);
    capsule->sens_out = sens_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    
    x0[6] = -0.746902;
    x0[7] = -0.659168;
    x0[8] = -1.38966;
    x0[9] = 1.18392;
    x0[10] = -1.32707;
    x0[11] = -1.43181;


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


int drone_insect_ode_acados_update_params(drone_insect_ode_solver_capsule * capsule, int stage, double *p, int np)
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



int drone_insect_ode_acados_solve(drone_insect_ode_solver_capsule * capsule)
{
    // solve NLP 
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int drone_insect_ode_acados_free(drone_insect_ode_solver_capsule * capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);

    // cost
    external_function_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);

    // constraints
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun_jac);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun);

    return 0;
}

ocp_nlp_in *drone_insect_ode_acados_get_nlp_in(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_in; }
ocp_nlp_out *drone_insect_ode_acados_get_nlp_out(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_out; }
ocp_nlp_out *drone_insect_ode_acados_get_sens_out(drone_insect_ode_solver_capsule * capsule) { return capsule->sens_out; }
ocp_nlp_solver *drone_insect_ode_acados_get_nlp_solver(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_solver; }
ocp_nlp_config *drone_insect_ode_acados_get_nlp_config(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_config; }
void *drone_insect_ode_acados_get_nlp_opts(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *drone_insect_ode_acados_get_nlp_dims(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_dims; }
ocp_nlp_plan *drone_insect_ode_acados_get_nlp_plan(drone_insect_ode_solver_capsule * capsule) { return capsule->nlp_solver_plan; }


void drone_insect_ode_acados_print_stats(drone_insect_ode_solver_capsule * capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[50000];
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

