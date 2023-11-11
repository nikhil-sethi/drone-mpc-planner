
#include <iostream>
#include "filtering.h"
#include <CppUTest/TestHarness.h> //include at last!

TEST_GROUP(Filtering) {
    float dt = 1.f/90.f;
    float K = 1.f;
};

TEST(Filtering, PT1_T_larger_dt) {
    filtering::Tf_PT1_f f;
    float T = 1.f/20.f;
    f.init(dt, K, T);
    f.reset(0.f);

    float y = f.new_sample(1.f);
    float t = dt;
    float error = y - (1-exp(-t/T));
    CHECK( abs(error)<0.1f);

    y = f.new_sample(1.f);
    t = 2*dt;
    error = y - (1-exp(-t/T));
    CHECK( abs(error)<0.1f);
}

TEST(Filtering, PT1_T_smaller_dt) {
    filtering::Tf_PT1_f f;
    float T = 1.f/360.f;
    f.init(dt, K, T);
    f.reset(0.f);

    float y = f.new_sample(1.f);
    float t = dt;
    float error = y - (1-exp(-t/T));
    CHECK( abs(error)<0.1f);

    y = f.new_sample(1.f);
    t = 2*dt;
    error = y - (1-exp(-t/T));
    CHECK( abs(error)<0.1f);
}

// TEST(Filtering, PT2_T_larger_dt) {
//     filtering::Tf_PT2_f f;
//     float T = 0.02f;
//     f.init(dt, K, T, T);
//     f.internal_states(0, 0);

//     filtering::Tf_PT1_f f1, f2;
//     f1.init(dt, K, T);
//     f2.init(dt, K, T);
//     f1.reset(0);
//     f2.reset(0);


//     float y;
//     float y1, y2;
//     for (int i=1; i<100; i++) {
//         y = f.new_sample(1.f);
//         y1 = f1.new_sample(1.);
//         y2 = f2.new_sample(y1);
//     }
// }
