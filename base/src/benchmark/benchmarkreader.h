#pragma once
#include <string>
#include <vector>

struct BenchmarkEntry {
    std::string type;
    std::string id;

    double pos_x;
    double pos_y;
    double pos_z;
    double vel_x;
    double vel_y;
    double vel_z;
};

extern std::vector<BenchmarkEntry> benchmark_entries;

class BenchmarkReader {
public:
    void ParseBenchmarkCSV(std::string file);

};
