#include "benchmarkreader.h"
#include<fstream>
#include <iostream>
#include <sstream>

// BenchmarkReader reader;
std::vector<BenchmarkEntry> benchmark_entries;

void BenchmarkReader::ParseBenchmarkCSV(std::string file) {
    std::ifstream fileStream(file);
    std::string line;
    std::string headers;

    std::getline(fileStream, headers);

    while (std::getline(fileStream, line)) {
        std::stringstream ss(line);
        std::vector<std::string> result;

        while (ss.good())
        {
            std::string substr;
            getline(ss, substr, ';');
            result.push_back(substr);
        }

        BenchmarkEntry entry;
        entry = (BenchmarkEntry) {
            .type = result[0],
            .id = result[1],
            .pos_x = std::stod(result[2]),
            .pos_y = std::stod(result[3]),
            .pos_z = std::stod(result[4]),
            .vel_x = std::stod(result[5]),
            .vel_y = std::stod(result[6]),
            .vel_z = std::stod(result[7])
        };
        benchmark_entries.push_back(entry);
    }
    return;
}
