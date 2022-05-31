#pragma once
#include <eigen3/Eigen/Core>
#include "tracking.h"
#include "sqpmethod.h"
#include "tti_quadratic_optimizer.h"

#define TTI_OPTIMIZER_DEBUGGING 2

struct tti_result {
    tti_result() {
        valid = false;
        time_to_intercept = 0;
        position_to_intercept = {0, 0, 0};
    };
    bool valid;
    double time_to_intercept;
    cv::Point3f position_to_intercept;
};

class TTIOptimizerInterface {
public:
    void init(float *thrust);
    void sqp_setup(sqp_solver_configuration config) {
        sqpsolver.setup(config);
    }
#ifdef OCP_DEV
    void init_casadi(std::string problem_solver_path) {
        use_casadi = true;
        sqpsolver.init_casadi(problem_solver_path);
    }
#endif
    tti_result find_best_interception(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect);

private:
    SQPSolver sqpsolver;
    TtiQuadraticOptimizer qpsolver;
    float *_thrust;
    problem_parameters prob_params;
    double inf = std::numeric_limits<double>::infinity();

    bool use_casadi = false;
    bool print_warning_enabled = false;

    void init_const_variable_bounds();
    void update_initial_guess(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect);
    void update_variable_bounds(tracking::TrackData track_data_drone, tracking::TrackData track_data_insect);
    bool feasible_solution(Eigen::VectorXd opti_var);
    // void export_scenario(tracking::TrackData drone, tracking::TrackData insect);
};
