#pragma once
#include <iostream>
#include <qpOASES.hpp>

struct QPSettings {

    QPSettings() {};
    QPSettings(qpOASES::Options opts[[maybe_unused]]) {
    }
};

std::ostream &operator<<(std::ostream &os, QPSettings &qp_config);
