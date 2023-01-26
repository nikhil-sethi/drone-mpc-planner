#pragma once
#include <string>
#include <vector>

struct BenchmarkEntry {
    std::string type;
    int id;

    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;

    int evasion_trigger;
    int evasion_type;
};

extern std::vector<BenchmarkEntry> benchmark_entries;

class BenchmarkReader {
public:
    void ParseBenchmarkCSV(std::string file);

};
