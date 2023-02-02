from pathlib import Path
import pandas as pd
import numpy as np


pd.set_option('mode.use_inf_as_na', True)


class BenchmarkResults:
    def __init__(self):
        self.benchmark_timestamp = None

        self.number_of_flights = None
        self.number_of_flights_started_late = None

        # hunt accuracy
        self.mean_best_interception_distance = None
        self.var_best_interception_distance = None

        self.mean_best_interception_distance_virtual = None
        self.var_best_interception_distance_virtual = None

        self.mean_best_interception_distance_replay = None
        self.var_best_interception_distance_replay = None

        self.number_of_kills = None

        # flight time
        self.mean_flight_time = None
        self.var_flight_time = None

        self.mean_flight_time_virtual = None
        self.var_flight_time_virtual = None

        self.mean_flight_time_replay = None
        self.var_flight_time_replay = None

        # crashes
        self.number_of_crashes = None


class BenchmarkEntry:
    def __init__(self):
        self.lines = None
        self.entry_file_path = None

        self.take_off_datetime = None
        self.land_datetime = None
        self.flight_time = None

        self.crashed = False

        self.best_interception_distance = None
        self.kill = False
        self.time_to_best_interception = None
        self.pos_best_interception_x = None
        self.pos_best_interception_y = None
        self.pos_best_interception_z = None
        self.vel_best_interception_x = None
        self.vel_best_interception_y = None
        self.vel_best_interception_z = None
        self.acc_best_interception_x = None
        self.acc_best_interception_y = None
        self.acc_best_interception_z = None

        self.benchmark_type = None

        self.benchmark_timestamp = None

        self.benchmark_insect_pos_x = None
        self.benchmark_insect_pos_y = None
        self.benchmark_insect_pos_z = None
        self.benchmark_insect_vel_x = None
        self.benchmark_insect_vel_y = None
        self.benchmark_insect_vel_z = None

        self.voltage_reduction = None


class BenchmarkParser:
    def __init__(self, search_path):
        self.file_path = search_path
        self.benchmark_entries = []
        self.sorted_benchmark_entries = {}
        self.dataframe = None
        self.benchmark_results = {}

    def check_if_benchmark_entry(self, lines):
        for line in lines:
            if line.find("benchmark_timestamp") != -1:
                return True
        return False

    def find_benchmark_entries(self):
        for path in Path(self.file_path).rglob("flight_results*.txt"):
            file = open(path, "r")
            lines = file.readlines()
            if self.check_if_benchmark_entry(lines):
                entry = BenchmarkEntry()
                entry.entry_file_path = path
                entry.lines = lines
                self.benchmark_entries.append(entry)

    def fill_benchmark_entries(self):
        for entry in self.benchmark_entries:
            for line in entry.lines:
                if line.find("take_off_datetime") != -1:
                    take_off_datetime = line.strip().split(":")[1]
                    entry.take_off_datetime = take_off_datetime
                if line.find("land_datetime") != -1:
                    land_datetime = line.strip().split(":")[1]
                    entry.land_datetime = land_datetime
                if line.find("flight_time") != -1:
                    flight_time = line.strip().split(":")[1]
                    entry.flight_time = flight_time
                if line.find("crashed") != -1:
                    crashed = line.strip().split(":")[1]
                    if int(crashed) == 1:
                        entry.crashed = True
                if line.find("best_interception_distance") != -1:
                    best_interception_distance = line.strip().split(":")[1]
                    entry.best_interception_distance = best_interception_distance
                    if float(best_interception_distance) < MAX_KILL_DISTANCE:
                        entry.kill = True
                if line.find("time_to_best_interception") != -1:
                    time_to_best_interception = line.strip().split(":")[1]
                    entry.time_to_best_interception = time_to_best_interception
                if line.find("pos_best_interception_xyz") != -1:
                    pos_best_interception_x, pos_best_interception_y, pos_best_interception_z = line.strip().split(":")[
                        1].split(",")
                    entry.pos_best_interception_x = pos_best_interception_x
                    entry.pos_best_interception_y = pos_best_interception_y
                    entry.pos_best_interception_z = pos_best_interception_z
                if line.find("vel_best_interception_xyz") != -1:
                    vel_best_interception_x, vel_best_interception_y, vel_best_interception_z = line.strip().split(":")[
                        1].split(",")
                    entry.vel_best_interception_x = vel_best_interception_x
                    entry.vel_best_interception_y = vel_best_interception_y
                    entry.vel_best_interception_z = vel_best_interception_z
                if line.find("acc_best_interception_xyz") != -1:
                    acc_best_interception_x, acc_best_interception_y, acc_best_interception_z = line.strip().split(":")[
                        1].split(",")
                    entry.acc_best_interception_x = acc_best_interception_x
                    entry.acc_best_interception_y = acc_best_interception_y
                    entry.acc_best_interception_z = acc_best_interception_z
                if line.find("benchmark_type") != -1:
                    benchmark_type = line.strip().split(":")[1]
                    entry.benchmark_type = benchmark_type
                if line.find("benchmark_timestamp") != -1:
                    benchmark_timestamp = line.strip().split(":")[1]
                    entry.benchmark_timestamp = benchmark_timestamp
                if line.find("benchmark_insect_pos_xyz") != -1:
                    benchmark_insect_pos_x, benchmark_insect_pos_y, benchmark_insect_pos_z = line.strip().split(":")[
                        1].split(",")
                    entry.benchmark_insect_pos_x = benchmark_insect_pos_x
                    entry.benchmark_insect_pos_y = benchmark_insect_pos_y
                    entry.benchmark_insect_pos_z = benchmark_insect_pos_z
                if line.find("benchmark_insect_vel_xyz") != -1:
                    benchmark_insect_vel_x, benchmark_insect_vel_y, benchmark_insect_vel_z = line.strip().split(":")[
                        1].split(",")
                    entry.benchmark_insect_vel_x = benchmark_insect_vel_x
                    entry.benchmark_insect_vel_y = benchmark_insect_vel_y
                    entry.benchmark_insect_vel_z = benchmark_insect_vel_z
                if line.find("benchmark_replay_id") != -1:
                    benchmark_replay_id = line.strip().split(":")[1]
                    entry.benchmark_replay_id = benchmark_replay_id
                if line.find("benchmark_entry_id") != -1:
                    benchmark_entry_id = line.strip().split(":")[1]
                    entry.benchmark_entry_id = benchmark_entry_id
                if line.find("voltage_reduction") != -1:
                    voltage_reduction = line.strip().split(":")[1]
                    entry.voltage_reduction = voltage_reduction

    def assemble_dataframe(self):
        self.dataframe = pd.DataFrame(
            columns=[
                "benchmark_timestamp",
                "benchmark_type",
                "benchmark_insect_pos_x",
                "benchmark_insect_pos_y",
                "benchmark_insect_pos_z",
                "benchmark_insect_vel_x",
                "benchmark_insect_vel_y",
                "benchmark_insect_vel_z",
                "take_off_datetime",
                "land_datetime",
                "flight_time",
                "crashed",
                "best_interception_distance",
                "kill",
                "time_to_best_interception",
                "pos_best_interception_x",
                "pos_best_interception_y",
                "pos_best_interception_z",
                "vel_best_interception_x",
                "vel_best_interception_y",
                "vel_best_interception_z",
                "acc_best_interception_x",
                "acc_best_interception_y",
                "acc_best_interception_z",
                "benchmark_entry_id",
                "voltage_reduction",
            ]
        )
        for entry in self.benchmark_entries:
            _new_row = pd.DataFrame(
                [
                    {
                        "benchmark_timestamp": entry.benchmark_timestamp,
                        "benchmark_type": entry.benchmark_type,
                        "benchmark_insect_pos_x": float(entry.benchmark_insect_pos_x)
                        if entry.benchmark_insect_pos_x
                        else None,
                        "benchmark_insect_pos_y": float(entry.benchmark_insect_pos_y)
                        if entry.benchmark_insect_pos_y
                        else None,
                        "benchmark_insect_pos_z": float(entry.benchmark_insect_pos_z)
                        if entry.benchmark_insect_pos_z
                        else None,
                        "benchmark_insect_vel_x": float(entry.benchmark_insect_vel_x)
                        if entry.benchmark_insect_vel_x
                        else None,
                        "benchmark_insect_vel_y": float(entry.benchmark_insect_vel_y)
                        if entry.benchmark_insect_vel_y
                        else None,
                        "benchmark_insect_vel_z": float(entry.benchmark_insect_vel_z)
                        if entry.benchmark_insect_vel_z
                        else None,
                        "take_off_datetime": entry.take_off_datetime,
                        "land_datetime": entry.land_datetime,
                        "flight_time": float(entry.flight_time),
                        "crashed": int(entry.crashed),
                        "best_interception_distance": float(
                            entry.best_interception_distance
                        ),
                        "kill": int(entry.kill),
                        "time_to_best_interception": float(
                            entry.time_to_best_interception
                        ),
                        "pos_best_interception_x": float(entry.pos_best_interception_x),
                        "pos_best_interception_y": float(entry.pos_best_interception_y),
                        "pos_best_interception_z": float(entry.pos_best_interception_z),
                        "vel_best_interception_x": float(entry.vel_best_interception_x),
                        "vel_best_interception_y": float(entry.vel_best_interception_y),
                        "vel_best_interception_z": float(entry.vel_best_interception_z),
                        "acc_best_interception_x": float(entry.acc_best_interception_x),
                        "acc_best_interception_y": float(entry.acc_best_interception_y),
                        "acc_best_interception_z": float(entry.acc_best_interception_z),
                        "benchmark_entry_id": int(entry.benchmark_entry_id),
                        "voltage_reduction": float(entry.voltage_reduction),
                    }
                ]
            )
            self.dataframe = pd.concat([_new_row, self.dataframe.loc[:]]).reset_index(
                drop=True
            )
        if DROP_DUPLICATES and self.dataframe[
            ["benchmark_timestamp", "benchmark_entry_id"]
        ].drop_duplicates().shape[0] != len(self.dataframe):
            print("Duplicates found, dropping them...")
            self.dataframe = self.dataframe.drop_duplicates(
                subset=["benchmark_timestamp", "benchmark_entry_id"], keep="first"
            )

        if (self.dataframe["benchmark_entry_id"] > TOTAL_BENCHMARK_ENTRIES).any:
            self.dataframe = self.dataframe[
                self.dataframe["benchmark_entry_id"] < TOTAL_BENCHMARK_ENTRIES
            ]

    def calculate_count(self, benchmark_timestamp, column, insect_type=None):
        _filtered_dataframe = self.dataframe[
            self.dataframe["benchmark_timestamp"] == benchmark_timestamp
        ]

        if insect_type == "virtual" or insect_type == "replay":
            _filtered_dataframe = _filtered_dataframe[
                _filtered_dataframe["benchmark_type"] == insect_type
            ]
        elif insect_type is not None:
            raise Exception("Unknown insect type")

        _count = _filtered_dataframe[column].sum()
        return _count


    def calculate_mean_and_var(self, benchmark_timestamp, column, insect_type=None):
        _filtered_dataframe = self.dataframe[
            self.dataframe["benchmark_timestamp"] == benchmark_timestamp
        ]

        if insect_type == "virtual" or insect_type == "replay":
            _filtered_dataframe = _filtered_dataframe[
                _filtered_dataframe["benchmark_type"] == insect_type
            ]
        elif insect_type is not None:
            raise Exception("Unknown insect type")

        _mean = _filtered_dataframe[column].mean()
        _var = _filtered_dataframe[column].var()
        return _mean, _var

    def calculate_benchmark_score(self):
        for benchmark in self.dataframe["benchmark_timestamp"].drop_duplicates():
            results = BenchmarkResults()
            results.benchmark_timestamp = benchmark

            results.number_of_flights = len(
                parser.dataframe[parser.dataframe["benchmark_timestamp"] == benchmark]
            )
            results.number_of_flights_started_late = len(
                parser.dataframe[
                    (parser.dataframe["benchmark_timestamp"] == benchmark)
                    & (parser.dataframe["pos_best_interception_x"] == 0)
                ] + parser.dataframe[
                    (parser.dataframe["benchmark_timestamp"] == benchmark)
                    & (pd.isna(parser.dataframe["pos_best_interception_x"]))
                ]
            )

            (
                results.mean_best_interception_distance,
                results.var_best_interception_distance,
            ) = self.calculate_mean_and_var(benchmark, "best_interception_distance")
            (
                results.mean_best_interception_distance_replay,
                results.var_best_interception_distance_replay,
            ) = self.calculate_mean_and_var(
                benchmark, "best_interception_distance", "replay"
            )
            (
                results.mean_best_interception_distance_virtual,
                results.var_best_interception_distance_virtual,
            ) = self.calculate_mean_and_var(
                benchmark, "best_interception_distance", "virtual"
            )

            results.number_of_kills = self.calculate_count(benchmark, "kill")

            (
                results.mean_flight_time,
                results.var_flight_time,
            ) = self.calculate_mean_and_var(benchmark, "flight_time")
            (
                results.mean_flight_time_replay,
                results.var_flight_time_replay,
            ) = self.calculate_mean_and_var(benchmark, "flight_time", "replay")
            (
                results.mean_flight_time_virtual,
                results.var_flight_time_virtual,
            ) = self.calculate_mean_and_var(benchmark, "flight_time", "virtual")

            results.number_of_crashes = self.calculate_count(
                benchmark, "crashed"
            )
            results.number_of_crashes_replay = self.calculate_count(
                benchmark, "crashed", "replay"
            )
            results.number_of_crashes_virtual = self.calculate_count(
                benchmark, "crashed", "virtual"
            )
            if benchmark not in self.benchmark_results:
                self.benchmark_results[benchmark] = results
            else:
                raise Exception("Benchmark already exists")


def find_matching_benchmark_entry(list_of_benchmark_entries, entry_id):
    return


if __name__ == "__main__":
    TOTAL_BENCHMARK_ENTRIES = 15
    DROP_DUPLICATES = True
    MAX_KILL_DISTANCE = 0.1

    parser = BenchmarkParser("/home/gemenerik/Downloads/benchmark/20230131tti")

    parser.find_benchmark_entries()
    parser.fill_benchmark_entries()
    parser.assemble_dataframe()

    parser.calculate_benchmark_score()

    with open("benchmark_results.org", "w+") as f:
        results_table = f"{parser.file_path}\n\n"
        results_table += "| Timestamp | No. flights | No. late flights | Hunt error mean | Hunt error var | No. kills | Flight time mean | Flight time var | Crashes |\n|--------+-----+--------+---------+----------------+-----------------+-------|\n"
        for _benchmark_time_date in sorted(parser.benchmark_results.keys()):
            results_table += f"| {_benchmark_time_date} | {parser.benchmark_results[_benchmark_time_date].number_of_flights} / {TOTAL_BENCHMARK_ENTRIES} | {parser.benchmark_results[_benchmark_time_date].number_of_flights_started_late} / {TOTAL_BENCHMARK_ENTRIES} | {parser.benchmark_results[_benchmark_time_date].mean_best_interception_distance} | {parser.benchmark_results[_benchmark_time_date].var_best_interception_distance} | {parser.benchmark_results[_benchmark_time_date].number_of_kills} | {parser.benchmark_results[_benchmark_time_date].mean_flight_time} | {parser.benchmark_results[_benchmark_time_date].var_flight_time} | {parser.benchmark_results[_benchmark_time_date].number_of_crashes} |\n"
        results_table += "\n"

        for _entry_id in range(1, TOTAL_BENCHMARK_ENTRIES + 1):
            results_table += f"**Entry {_entry_id}**\n"
            results_table += "| Benchmark timestamp | Type | Hunt error | Time to hunt error | Flight time | Crashed | Voltage drop | Interception pos xyz |\n|------+----------+-----+-------+-------+------+----|\n"
            for _benchmark_time_date in sorted(parser.benchmark_results.keys()):
                _relevant_entry = parser.dataframe[
                    (parser.dataframe["benchmark_timestamp"]
                     == _benchmark_time_date)
                    & (parser.dataframe["benchmark_entry_id"] == _entry_id)
                ]
                if not _relevant_entry.empty:
                    results_table += f"| {_benchmark_time_date} | {_relevant_entry['benchmark_type'].values[0]} | {_relevant_entry['best_interception_distance'].values[0]} | {_relevant_entry['time_to_best_interception'].values[0]} |  {_relevant_entry['flight_time'].values[0]} | {_relevant_entry['crashed'].values[0]} | {_relevant_entry['voltage_reduction'].values[0]} | {_relevant_entry['pos_best_interception_x'].values[0]} {_relevant_entry['pos_best_interception_y'].values[0]} {_relevant_entry['pos_best_interception_z'].values[0]}  |\n"
            results_table += "\n"
        f.write(results_table)
