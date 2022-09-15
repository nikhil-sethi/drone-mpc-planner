#include "qpoasesconfig.h"

std::ostream &operator<<(std::ostream &os, QPSettings &qp_config [[maybe_unused]]) {
    os << "qpoases config unchanged";
    return os;
}
