#pragma once
#ifdef OCP_DEV
#include <casadi/casadi.hpp>
#endif
#include <eigen3/Eigen/Core>
#include "tracking.h"
#include "plane.h"
#include "flightarea.h"
#include "sqpmethod.h"
#include "intercept_in_planes_index.h"
#include "intercept_in_planes_quadratic_optimizer.h"
#define COLORED_RESET   "\033[0m"
#define COLORED_RED     "\033[31m"
#define COLORED_YELLOW  "\033[33m"
#define COLORED_GREEN   "\033[92m"

struct intercept_in_planes_result {
    intercept_in_planes_result() {
        valid = false;
        time_to_intercept = 0;
        position_to_intercept = {0, 0, 0};
        acceleration_to_intercept = {0, 0, 0};
        position_stopped = {0, 0, 0};
    };
    bool valid;
    double time_to_intercept;
    cv::Point3f position_to_intercept;
    cv::Point3f acceleration_to_intercept;
    cv::Point3f position_stopped;
};

class InterceptInPlanesOptimizerInterface {
public:
    bool use_casadi = false;

    void init(float *thrust, FlightArea *flightarea, safety_margin_types safety_margin);
    intercept_in_planes_result find_best_interception(tracking::TrackData drone, tracking::TrackData insect);
    void export_scenario(tracking::TrackData drone, tracking::TrackData insect, std::vector<Plane> planes);
    std::vector<cv::Point3f> interception_trajectory();

    void sqp_setup(sqp_solver_configuration config) {
        sqpsolver.setup(config);
    };
#ifdef OCP_DEV
    void init_casadi(std::string problem_solver_path) {
        use_casadi = true;
        sqpsolver.init_casadi(problem_solver_path);
    };
#endif
    sqp_solver_configuration sqp_convergence_parameter() {
        return sqpsolver.convergence_parameter();
    };
    std::tuple<int, int, int> scenario_setup() {
        return std::tuple(N_STEPS_INTERCEPTING + 1, N_STEPS_BREAKING + 1, N_PLANES);
    };

    void max_cpu_time(double mcput) {
        sqpsolver.max_cpu_time(mcput);
    };

    std::string quadratic_solver_library() { return qpsolver.quadratic_solver_library();};

private:
    const double interception_error_threshold = 0.05;
    bool print_warning_enabled = true;

    SQPSolver sqpsolver;
    InterceptInPlanesQuadraticOptimizer qpsolver;
    FlightArea *_flight_area;
    safety_margin_types _safety_margin;
    problem_parameters prob_params;
    Eigen::VectorXd opti_var;
    bool opti_res_valid = false;
    Eigen::VectorXd slack_variables;
    float *_thrust;
    double inf = std::numeric_limits<double>::infinity();

    void init_box_constraints();
    void init_inequality_constraints();

    void update_initial_guess(tracking::TrackData drone, tracking::TrackData insect);
    void update_box_constraints(tracking::TrackData drone, tracking::TrackData insect);
    void update_plane_parameters(std::vector<Plane> planes);
    void report_scenario_result(tracking::TrackData drone, tracking::TrackData insect, intercept_in_planes_result res);

    bool feasible_trajectory(Eigen::VectorXd xopt);

    void print_success(std::string success) {
        if (print_warning_enabled)
            std::cout << COLORED_GREEN << success << COLORED_RESET << std::endl;
    }
    void print_warning(std::string warning) {
        if (print_warning_enabled)
            std::cout << COLORED_YELLOW << warning << COLORED_RESET << std::endl;
    }
    void print_error(std::string error) {
        std::cout << COLORED_RED << error << COLORED_RESET << std::endl;
    }

};
