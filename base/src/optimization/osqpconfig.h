#pragma once
#include <osqp.h>
#include <iostream>

class QPSettings {
public:
    QPSettings(OSQPSettings *osqp_settings);
    QPSettings() {};
    void rho(float new_rho);
    void sigma(float new_sigma);
    void max_iter(uint new_max_iter);
    void eps_abs(float new_eps_abs);
    void eps_rel(float new_eps_rel);
    void eps_prim_inf(float new_eps_prim_inf);
    void eps_dual_inf(float new_eps_dual_inf);
    void alpha(float new_alpha);
    void delta(float new_delta);
    void polish(bool new_polish);
    void polish_refine_iter(uint new_polish_refine_iter);
    void verbose(bool new_verbose);
    void scaled_termination(bool new_scaled_termination);
    void check_termination(int new_check_termination);
    void warm_start(bool new_warm_start);
    void scaling(uint new_scaling);
    void adaptive_rho(bool new_adaptive_rho);
    void adaptive_rho_interval(float new_adaptive_rho_interval);
    void adaptive_rho_tolerance(float new_adaptive_rho_tolerance);
    void adaptive_rho_fraction(float new_adaptive_rho_fraction);
    void time_limit(float new_time_limit);

    float rho() {return _rho;};
    float sigma() {return _sigma;};
    uint max_iter() {return _max_iter;};
    float eps_abs() {return _eps_abs;};
    float eps_rel() {return _eps_rel;};
    float eps_prim_inf() {return _eps_prim_inf;};
    float eps_dual_inf() {return _eps_dual_inf;};
    float alpha() {return _alpha;};
    float delta() {return _delta;};
    bool polish() {return _polish;};
    uint polish_refine_iter() {return _polish_refine_iter;};
    bool verbose() {return _verbose;};
    bool scaled_termination() {return _scaled_termination;};
    int check_termination() {return _check_termination;};
    bool warm_start() {return _warm_start;};
    uint scaling() {return _scaling;};
    bool adaptive_rho() {return _adaptive_rho;};
    float adaptive_rho_interval() {return _adaptive_rho_interval;};
    float adaptive_rho_tolerance() {return _adaptive_rho_tolerance;};
    float adaptive_rho_fraction() {return _adaptive_rho_fraction;};
    float time_limit() {return _time_limit;};

    void apply(OSQPSettings *osqp_settings);

private:
    float _rho = 0.1; // default: 0.1
    float _sigma = 1e-6; //default: 1e-6
    uint _max_iter = 4000; // default: 4000
    float _eps_abs = 1e-3; // default: 1e-3
    float _eps_rel = 1e-3; // default: 1e-3
    float _eps_prim_inf = 1e-4; // default: 1-4
    float _eps_dual_inf = 1e-4; // default: 1-4
    float _alpha = 1.6; // default: 1.6
    // _float linsys_solver = QDLDL_SOLVER; // default: QDLDL_SOLVER
    float _delta = 1e-6; // default: 1e-6
    bool _polish = false; // default: false
    uint _polish_refine_iter = 3; // default: 3
    bool _verbose = 0; // default: true
    bool _scaled_termination = false; // default: false
    int _check_termination = 25; // default: 25
    bool _warm_start = true; // default: true
    uint _scaling = 10; //default 10
    bool _adaptive_rho = true; // default: true
    float _adaptive_rho_interval = 0; // default: 0
    float _adaptive_rho_tolerance = 5; // default: 5
    float _adaptive_rho_fraction = 0.4; // default: 0.4
    float _time_limit = 0; // default: 0.1

};

std::ostream &operator<<(std::ostream &os, QPSettings &qp_config);


