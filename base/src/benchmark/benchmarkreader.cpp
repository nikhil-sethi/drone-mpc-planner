#include "benchmarkreader.h"
#include "hash.h"
#include<fstream>
#include <iostream>
#include <sstream>

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
            .pos_x = std::stof(result[2]),
            .pos_y = std::stof(result[3]),
            .pos_z = std::stof(result[4]),
            .vel_x = std::stof(result[5]),
            .vel_y = std::stof(result[6]),
            .vel_z = std::stof(result[7]),
            .evasion_trigger = std::stoi(result[8]),
            .evasion_type = std::stoi(result[9])
        };
        benchmark_entries.push_back(entry);
    }
    return new_benchmark_hash;
}

void BenchmarkReader::WriteBenchmarkEntry(int benchmark_entry_id, std::string benchmark_time) {
    std::ofstream EntryFlag;
    EntryFlag.open("/home/pats/pats/flags/BenchmarkEntry.txt", std::ofstream::out | std::ofstream::trunc);
    EntryFlag << benchmark_entry_id << "\n";
    EntryFlag << benchmark_time << "\n";
    EntryFlag.close();
}
