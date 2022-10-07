#pragma once
#define TTI_VERSION "1"
#define N_OPTVARS 34
#define N_CONSTRAINTS 15
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
    delta_tI = 0,
    statesI_first = 1,
    statesI_last = 12,
    inputsI_first = 13,
    inputsI_last = 15,
    virtual_inputsI_first = 16,
    virtual_inputsI_last = 21,
    insect_states_first = 22,
    insect_states_last = 33,
};

enum inequality_constraint_idx {
    dronedynamicI_first = 0,
    dronedynamicI_last = 5,
    insectdynamic_first = 6,
    insectdynamic_last = 11,
    intercepting_first = 12,
    intercepting_last = 14,
};

enum trjactory_parameter {
    n_steps = 1,
    state_trajectory_first = 1,
    input_trajectory_first = 13,
    virtual_input_trajectory_first = 16,
};
