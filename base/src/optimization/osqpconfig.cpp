#include "osqpconfig.h"

QPSettings::QPSettings(OSQPSettings *osqp_settings) {
    _rho = osqp_settings->rho;
    _sigma = osqp_settings->sigma;
    _max_iter = osqp_settings->max_iter;
    _eps_abs = osqp_settings->eps_abs;
    _eps_rel = osqp_settings->eps_rel;
    _eps_prim_inf = osqp_settings->eps_prim_inf;
    _eps_dual_inf = osqp_settings->eps_dual_inf;
    _alpha = osqp_settings->alpha;
    _delta = osqp_settings->delta;
    _polish = osqp_settings->polish;
    _polish_refine_iter = osqp_settings->polish_refine_iter;
    _verbose = osqp_settings->verbose;
    _scaled_termination = osqp_settings->scaled_termination;
    _check_termination = osqp_settings->check_termination;
    _warm_start = osqp_settings->warm_start;
    _scaling = osqp_settings->scaling;
    _adaptive_rho = osqp_settings->adaptive_rho;
    _adaptive_rho_interval = osqp_settings->adaptive_rho_interval;
    _adaptive_rho_tolerance = osqp_settings->adaptive_rho_tolerance;
    _adaptive_rho_fraction = osqp_settings->adaptive_rho_fraction;
    _time_limit = osqp_settings->time_limit;
}

void QPSettings::apply(OSQPSettings *osqp_settings) {
    osqp_settings->rho = _rho;
    osqp_settings->sigma = _sigma;
    osqp_settings->max_iter = _max_iter;
    osqp_settings->eps_abs = _eps_abs;
    osqp_settings->eps_rel = _eps_rel;
    osqp_settings->eps_prim_inf = _eps_prim_inf;
    osqp_settings->eps_dual_inf = _eps_dual_inf;
    osqp_settings->alpha = _alpha;
    osqp_settings->delta = _delta;
    osqp_settings->polish = _polish;
    osqp_settings->polish_refine_iter = _polish_refine_iter;
    osqp_settings->verbose = _verbose;
    osqp_settings->scaled_termination = _scaled_termination;
    osqp_settings->check_termination = _check_termination;
    osqp_settings->warm_start = _warm_start;
    osqp_settings->scaling = _scaling;
    osqp_settings->adaptive_rho = _adaptive_rho;
    osqp_settings->adaptive_rho_interval = _adaptive_rho_interval;
    osqp_settings->adaptive_rho_tolerance = _adaptive_rho_tolerance;
    osqp_settings->adaptive_rho_fraction = _adaptive_rho_fraction;
    osqp_settings->time_limit = _time_limit;
}
void QPSettings::rho(float new_rho) {
    if (new_rho > 0)
        _rho = new_rho;
};
void QPSettings::sigma(float new_sigma) {
    if (new_sigma > 0)
        _sigma = new_sigma;
};
void QPSettings::max_iter(uint new_max_iter) {
    if (new_max_iter > 0)
        _max_iter = new_max_iter;
};
void QPSettings::eps_abs(float new_eps_abs) {
    if (new_eps_abs >= 0)
        _eps_abs = new_eps_abs;
};
void QPSettings::eps_rel(float new_eps_rel) {
    if (new_eps_rel >= 0)
        _eps_rel = new_eps_rel;
};
void QPSettings::eps_prim_inf(float new_eps_prim_inf) {
    if (new_eps_prim_inf >= 0)
        _eps_prim_inf = new_eps_prim_inf;
};
void QPSettings::eps_dual_inf(float new_eps_dual_inf) {
    if (new_eps_dual_inf >= 0)
        _eps_dual_inf = new_eps_dual_inf;
};
void QPSettings::alpha(float new_alpha) {
    if (2 < new_alpha && new_alpha > 0)
        _alpha = new_alpha;
};
void QPSettings::delta(float new_delta) {
    if (new_delta > 0)
        _delta = new_delta;
};
void QPSettings::polish(bool new_polish) {
    _polish = new_polish;
};
void QPSettings::polish_refine_iter(uint new_polish_refine_iter) {
    if (new_polish_refine_iter > 0)
        _polish_refine_iter = new_polish_refine_iter;
};
void QPSettings::verbose(bool new_verbose) {
    _verbose = new_verbose;
};
void QPSettings::scaled_termination(bool new_scaled_termination) {
    _scaled_termination = new_scaled_termination;
};
void QPSettings::check_termination(int new_check_termination) {
    if (new_check_termination >= 0)
        _check_termination = new_check_termination;
};
void QPSettings::warm_start(bool new_warm_start) {
    _warm_start = new_warm_start;
};
void QPSettings::scaling(uint new_scaling) {
    if (new_scaling > 0)
        _scaling = new_scaling;
};
void QPSettings::adaptive_rho(bool new_adaptive_rho) {
    _adaptive_rho = new_adaptive_rho;
};
void QPSettings::adaptive_rho_interval(float new_adaptive_rho_interval) {
    if (new_adaptive_rho_interval > 0)
        _adaptive_rho_interval = new_adaptive_rho_interval;
};
void QPSettings::adaptive_rho_tolerance(float new_adaptive_rho_tolerance) {
    if (new_adaptive_rho_tolerance >= 1)
        _adaptive_rho_tolerance = new_adaptive_rho_tolerance;
};
void QPSettings::adaptive_rho_fraction(float new_adaptive_rho_fraction) {
    if (new_adaptive_rho_fraction > 0)
        _adaptive_rho_fraction = new_adaptive_rho_fraction;
};
void QPSettings::time_limit(float new_time_limit) {
    if (new_time_limit >= 0)
        _time_limit = new_time_limit;
};

std::ostream &operator<<(std::ostream &os, QPSettings &qp_config) {
    os << "rho: " << qp_config.rho() <<
       ", sigma: " << qp_config.sigma() <<
       ", max_iter: " << qp_config.max_iter() <<
       ", eps_abs: " << qp_config.eps_abs() <<
       ", eps_rel: " << qp_config.eps_rel() <<
       ", eps_prim_inf: " << qp_config.eps_prim_inf() <<
       ", eps_dual_inf: " << qp_config.eps_dual_inf() <<
       ", alpha: " << qp_config.alpha() <<
       ", delta: " << qp_config.delta() <<
       ", polish: " << qp_config.polish() <<
       ", polish_refine_iter: " << qp_config.polish_refine_iter() <<
       ", verbose: " << qp_config.verbose() <<
       ", scaled_termination: " << qp_config.scaled_termination() <<
       ", check_termination: " << qp_config.check_termination() <<
       ", warm_start: " << qp_config.warm_start() <<
       ", scaling: " << qp_config.scaling() <<
       ", adaptive_rho: " << qp_config.adaptive_rho() <<
       ", adaptive_rho_interval: " << qp_config.adaptive_rho_interval() <<
       ", adaptive_rho_tolerance: " << qp_config.adaptive_rho_tolerance() <<
       ", adaptive_rho_fraction: " << qp_config.adaptive_rho_fraction() <<
       ", time_limit: " << qp_config.time_limit();
    return os;
}
