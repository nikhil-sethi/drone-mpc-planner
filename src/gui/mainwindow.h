#ifndef MWINDOW_H
#define MWINDOW_H

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

#include <thread>
#include <mutex>

class MainWindow{

public:
    void init(int argc, char **argv);
    void close();

private:
    std::thread thread;
    void worker(int argc, char **argv);
};

#endif // MWINDOW_H
