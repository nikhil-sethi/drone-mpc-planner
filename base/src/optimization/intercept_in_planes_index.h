#pragma once
#define INTERCEPT_IN_PLANES_VERSION "1"
#define N_OPTVARS 65
#define N_CONSTRAINTS 47
#define N_SAMPLES_INTERCEPTING 2
#define N_STEPS_INTERCEPTING 1
#define N_SAMPLES_BREAKING 2
#define N_STEPS_BREAKING 1
#define N_PLANES 10
#define N_PLANE_PARAMETERS 6
#define N_DRONE_STATES 6
#define N_DRONE_INPUTS 3
#define N_INSECT_STATES 6
#define DRONE_POSX 0
#define DRONE_POSY 1
#define DRONE_POSZ 2
#define DRONE_VELX 3
#define DRONE_VELY 4
#define DRONE_VELZ 5
#define DRONE_ACCX 0
#define DRONE_ACCY 1
#define DRONE_ACCZ 2
#define INSECT_POSX 0
#define INSECT_POSY 1
#define INSECT_POSZ 2
#define INSECT_VELX 3
#define INSECT_VELY 4
#define INSECT_VELZ 5

enum optvar_idx {
    dt_intercepting = 0,
    dt_breaking = 1,
    drone_states_intercepting_first = 2,
    drone_states_intercepting_last = 13,
    drone_states_breaking_first = 14,
    drone_states_breaking_last = 25,
    drone_inputs_intercepting_first = 26,
    drone_inputs_intercepting_last = 28,
    drone_inputs_breaking_first = 29,
    drone_inputs_breaking_last = 31,
    drone_virtual_inputs_intercepting_first = 32,
    drone_virtual_inputs_intercepting_last = 37,
    drone_virtual_inputs_breaking_first = 38,
    drone_virtual_inputs_breaking_last = 43,
    insect_states_first = 44,
    insect_states_last = 55,
    interception_slacks_first = 56,
    interception_slacks_last = 58,
    drone_transition_slacks_first = 59,
    drone_transition_slacks_last = 64,
};

enum inequality_constraint_idx {
    dronedynamics_intercepting_first = 0,
    dronedynamics_intercepting_last = 5,
    insectdynamic_first = 6,
    insectdynamic_last = 11,
    intercepting_error_first = 12,
    intercepting_error_last = 14,
    transition_dronestate_first = 15,
    transition_dronestate_last = 20,
    dronedynamic_breaking_first = 21,
    dronedynamic_breaking_last = 26,
    plane_constraints_first = 27,
    plane_constraints_last = 46,
};

enum trjactory_parameter {
    n_steps = 2,
    state_trajectory_first = 2,
    input_trajectory_first = 26,
    virtual_input_trajectory_first = 32,
};
