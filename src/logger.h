//DOES NOT WORK AT THE MOMENT
#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>


struct DoubleStreamer
{
    DoubleStreamer(std::ostream& out1, std::ostream& out2) : out1_(out1), out2_(out2) {}
    std::ostream& out1_;
    std::ostream& out2_;
};

template <typename T>
DoubleStreamer& operator<<(DoubleStreamer& h, T const& t) {
    h.out1_ << t;
    h.out2_ << t;
    return h;
}

template <typename T>
DoubleStreamer& operator<<(DoubleStreamer& h, std::ostream&(*f)(std::ostream&)) {
    h.out1_ << f;
    h.out2_ << f;
    return h;
}

/*
 * This class is a logging tool
 *
 */
class Logger {
public:
    Logger(std::ostream& os) : _os(os), _verbose(1) {}
    void init(std::string logfile);

private:
    template<typename T> friend std::ostream& operator<<(Logger&, T);

    std::ostream& _os; // output stream, cout
    int _verbose; // whether to output to cout as well
    std::ofstream los; // log output stream
    std::ofstream both;
};

template<typename T>
std::ostream& operator<<(Logger& log, T op) {
    if (log._verbose)
        log._os << op;

    log.los << op;
    return log.both;
}



#endif // LOGGER_H
