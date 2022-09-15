#pragma once
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

enum optvar_idx {
    dt_intercepting = 0,
    drone_posx0_intercepting = 1,
    drone_posy0_intercepting = 2,
    drone_posz0_intercepting = 3,
    drone_velx0_intercepting = 4,
    drone_vely0_intercepting = 5,
    drone_velz0_intercepting = 6,
    drone_posxF_intercepting = 7,
    drone_posyF_intercepting = 8,
    drone_poszF_intercepting = 9,
    drone_velzF_intercepting = 12,
    drone_accx0_intercepting = 13,
    drone_accy0_intercepting = 14,
    drone_accz0_intercepting = 15,
    drone_acczF_intercepting = 15,
    drone_virt_accx0_intercepting = 16,
    drone_virt_acczF_intercepting = 21,
    insect_posx0 = 22,
    insect_posy0 = 23,
    insect_posz0 = 24,
    insect_velx0 = 25,
    insect_vely0 = 26,
    insect_velz0 = 27,
    insect_velzF = 33,
    dt_breaking = 34,
    drone_posx0_breaking = 35,
    drone_velxF_breaking = 44,
    drone_velzF_breaking = 46,
    drone_accx0_breaking = 47,
    drone_acczF_breaking = 49,
    drone_virt_accx0_breaking = 50,
    drone_virt_acczF_breaking = 55,
    interception_slack0 = 56,
    interception_slackF = 58,
    state_transition_slack0 = 59,
    state_transition_slackF = 61,
};

enum inequality_constraint_idx {
    first_plane_constraint = 27,
    last_plane_constraint = 46
};
