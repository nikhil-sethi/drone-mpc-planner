#pragma once
#include <string>
#include <vector>

struct BenchmarkEntry {
    std::string type;
    int id;

    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;

    int evasion_trigger;
    int evasion_type;
};

extern std::vector<BenchmarkEntry> benchmark_entries;

class BenchmarkReader {
public:
    size_t ParseBenchmarkCSV(std::string file);
    void WriteBenchmarkEntry(int benchmark_entry_id, std::string benchmark_time);
};
