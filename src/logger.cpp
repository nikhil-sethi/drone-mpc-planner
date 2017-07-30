#include "logger.h"



void Logger::init (std::string logfile) {
    los.open (logfile);
    DoubleStreamer h(los, std::cout);
}


