#include "benchmarkreader.h"
#include<fstream>
#include <iostream>
#include <sstream>


struct HashableObj {
    std::string string;
    std::size_t previous_hash;
};

struct MyHash
{
    std::size_t operator()(HashableObj const &s) const noexcept
    {
        std::size_t h1 = std::hash<std::string> {}(s.string);
        std::size_t h2 = s.previous_hash;
        return h1 ^ (h2 << 1);
    }
};
// BenchmarkReader reader;
std::vector<BenchmarkEntry> benchmark_entries;

size_t BenchmarkReader::ParseBenchmarkCSV(std::string file) {
    std::ifstream fileStream(file);
    std::string line;
    std::string headers;

    std::getline(fileStream, headers);
    size_t new_benchmark_hash = 132;

    while (std::getline(fileStream, line)) {
        std::stringstream ss(line);
        std::vector<std::string> result;
        while (ss.good())
        {
            std::string substr;
            getline(ss, substr, ';');
            HashableObj obj =  {substr, new_benchmark_hash};
            new_benchmark_hash = MyHash{}(obj);
            result.push_back(substr);
        }

        BenchmarkEntry entry;
        entry = (BenchmarkEntry) {
            .type = result[0],
            .id = std::stoi(result[1]),
            .pos_x = std::stod(result[2]),
            .pos_y = std::stod(result[3]),
            .pos_z = std::stod(result[4]),
            .vel_x = std::stod(result[5]),
            .vel_y = std::stod(result[6]),
            .vel_z = std::stod(result[7]),
            .evasion_trigger = std::stoi(result[8]),
            .evasion_type = std::stoi(result[9])
        };
        benchmark_entries.push_back(entry);
    }
    return new_benchmark_hash;
}
