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

        self.number_of_monsters = None
        self.number_of_insects = None

        self.benchmark_hash = None
        self.executor_hash = None

        self.dataframe = None

    def assemble_dataframe(self):
        self.dataframe = pd.DataFrame(
            columns=[
                "benchmark_timestamp",
                "number_of_flights",
                "number_of_flights_started_late",
                "mean_best_interception_distance",
                "var_best_interception_distance",
                "mean_best_interception_distance_virtual",
                "var_best_interception_distance_virtual",
                "mean_best_interception_distance_replay",
                "var_best_interception_distance_replay",
                "number_of_kills",
                "mean_flight_time",
                "var_flight_time",
                "mean_flight_time_virtual",
                "var_flight_time_virtual",
                "mean_flight_time_replay",
                "var_flight_time_replay",
                "number_of_crashes",
                "number_of_monsters",
                "number_of_insects",
                "benchmark_hash",
                "executor_hash",
            ]
        )
        _new_row = pd.DataFrame(
            [
                {
                    "benchmark_timestamp": self.benchmark_timestamp,
                    "number_of_flights": self.number_of_flights,
                    "number_of_flights_started_late": self.number_of_flights_started_late,
                    "mean_best_interception_distance": self.mean_best_interception_distance,
                    "var_best_interception_distance": self.var_best_interception_distance,
                    "mean_best_interception_distance_virtual": self.mean_best_interception_distance_virtual,
                    "var_best_interception_distance_virtual": self.var_best_interception_distance_virtual,
                    "mean_best_interception_distance_replay": self.mean_best_interception_distance_replay,
                    "var_best_interception_distance_replay": self.var_best_interception_distance_replay,
                    "number_of_kills": self.number_of_kills,
                    "mean_flight_time": self.mean_flight_time,
                    "var_flight_time": self.var_flight_time,
                    "mean_flight_time_virtual": self.mean_flight_time_virtual,
                    "var_flight_time_virtual": self.var_flight_time_virtual,
                    "mean_flight_time_replay": self.mean_flight_time_replay,
                    "var_flight_time_replay": self.var_flight_time_replay,
                    "number_of_crashes": self.number_of_crashes,
                    "number_of_monsters": self.number_of_monsters,
                    "number_of_insects": self.number_of_insects,
                    "benchmark_hash": self.benchmark_hash,
                    "executor_hash": self.executor_hash,
                }
            ]
        )
        self.dataframe = pd.concat([_new_row, self.dataframe.loc[:]]).reset_index(
            drop=True
        )


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

        self.n_monsters = None
        self.n_insects = None


class BenchmarkParser:
    def __init__(self, search_path, benchmark_csv_path):
        self.file_path = search_path
        self.benchmark_csv_path = benchmark_csv_path
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
                    take_off_datetime = ':'.join(line.strip().split(":")[1:4])
                    entry.take_off_datetime = take_off_datetime
                if line.find("land_datetime") != -1:
                    land_datetime = ':'.join(line.strip().split(":")[1:4])
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
                if line.find("n_monsters") != -1:
                    n_monsters = line.strip().split(":")[1]
                    entry.n_monsters = n_monsters
                if line.find("n_insects") != -1:
                    n_insects = line.strip().split(":")[1]
                    entry.n_insects = n_insects
                if line.find("benchmark_hash") != -1:
                    benchmark_hash = line.strip().split(":")[1]
                    entry.benchmark_hash = benchmark_hash
                if line.find("executor_hash") != -1:
                    executor_hash = line.strip().split(":")[1]
                    entry.executor_hash = executor_hash

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
                "n_monsters",
                "n_insects",
                "benchmark_hash",
                "executor_hash",
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
                        "n_monsters": int(entry.n_monsters) if entry.n_monsters else None,
                        "n_insects": int(entry.n_insects) if entry.n_insects else None,
                        "benchmark_hash": entry.benchmark_hash,
                        "executor_hash": entry.executor_hash,
                    }
                ]
            )
            self.dataframe = pd.concat([_new_row, self.dataframe.loc[:]]).reset_index(
                drop=True
            )

        if self.dataframe[
            ["benchmark_timestamp", "benchmark_entry_id"]
        ].drop_duplicates().shape[0] != len(self.dataframe):
            if DROP_DUPLICATES:
                print("Duplicates found, dropping them...")
                self.dataframe = self.dataframe.drop_duplicates(
                    subset=["benchmark_timestamp", "benchmark_entry_id"], keep="first"
                )
            else:
                print("DUPLICATES FOUND!")


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

            _filtered_dataframe = self.dataframe[self.dataframe["benchmark_timestamp"] == benchmark]
            results.benchmark_hash = _filtered_dataframe["benchmark_hash"].iloc[0]
            results.executor_hash = _filtered_dataframe["executor_hash"].iloc[0]

            if benchmark not in self.benchmark_results:
                self.benchmark_results[benchmark] = results
            else:
                raise Exception("Benchmark already exists")

            results.assemble_dataframe()

    def parse_benchmark_csv(self):
        self.benchmark_csv = pd.read_csv(self.benchmark_csv_path, sep=";")


class Utils:
    def dataframe_to_orgmode_table(dataframe):
        dataframe = dataframe.to_csv(
            sep="|", header=True, index=False).strip('\n').split('\n')
        dataframe.insert(1, "-+-")
        dataframe_string = '|\n|'.join(dataframe)
        return f"|{dataframe_string}|\n"

    def write_moth_entry_type(entry):
        _entry_type = _entry["type"]
        result = f"{_entry_type} moth"
        if _entry_type == "replay":
            _entry_replay_id = _entry["id"]
            result += f" (replay id: {_entry_replay_id})\n"
        elif _entry_type == "virtual":
            _entry_pos_x = _entry["pos_x"]
            _entry_pos_y = _entry["pos_y"]
            _entry_pos_z = _entry["pos_z"]
            _entry_vel_x = _entry["vel_x"]
            _entry_vel_y = _entry["vel_y"]
            _entry_vel_z = _entry["vel_z"]

            _entry_evasion_trigger = "spinup" if _entry["evasion_trigger"] == 1 else "hunt error"
            _entry_evasion_type = "diving" if _entry[
                "evasion_type"] == 1 else "u-turn" if _entry["evasion_type"] == 2 else "none"
            result += f" (pos: {_entry_pos_x}, {_entry_pos_y}, {_entry_pos_z}, vel: {_entry_vel_x}, {_entry_vel_y}, {_entry_vel_z}, evasion trigger: {_entry_evasion_trigger}, evasion type: {_entry_evasion_type})\n"
        return result


if __name__ == "__main__":
    TOTAL_BENCHMARK_ENTRIES = 16
    DROP_DUPLICATES = False
    MAX_KILL_DISTANCE = 0.05
    MEAN_VAR_PER_INSECT = False

    parser = BenchmarkParser("/home/pats/Downloads/example_path/",
                             "/home/pats/code/pats/base/benchmark/benchmark_short.csv")

    parser.find_benchmark_entries()
    parser.fill_benchmark_entries()
    parser.assemble_dataframe()
    parser.calculate_benchmark_score()
    parser.parse_benchmark_csv()

    with open("benchmark_results.org", "w+") as f:
        results_string = f"{parser.file_path}\n\n"

        relevant_dataframes = []
        for _benchmark_time_date in sorted(parser.benchmark_results.keys()):
            relevant_dataframes.append(parser.benchmark_results[_benchmark_time_date].dataframe)
        relevant_dataframes = pd.concat(relevant_dataframes)
        results_string += Utils.dataframe_to_orgmode_table(relevant_dataframes[['benchmark_timestamp', 'number_of_flights', 'number_of_flights_started_late', 'mean_best_interception_distance', 'var_best_interception_distance', 'number_of_kills', 'number_of_crashes', 'benchmark_hash', 'executor_hash']])
        results_string += "\n"

        for _entry_id in range(0, TOTAL_BENCHMARK_ENTRIES):
            hunt_error_list = np.array([])
            results_string += f"**Entry {_entry_id}**\n"

            _entry = parser.benchmark_csv.iloc[_entry_id]

            results_string += Utils.write_moth_entry_type(_entry)

            relevant_dataframes = []
            for _benchmark_time_date in sorted(parser.benchmark_results.keys()):
                _relevant_entry = parser.dataframe[
                    (parser.dataframe["benchmark_timestamp"]
                     == _benchmark_time_date)
                    & (parser.dataframe["benchmark_entry_id"] == _entry_id+1)
                ]
                if not _relevant_entry.empty:
                    relevant_dataframes.append(_relevant_entry)
                    hunt_error_list = np.append(
                        hunt_error_list, _relevant_entry['best_interception_distance'].values[0])
            if relevant_dataframes:
                relevant_dataframes = pd.concat(relevant_dataframes)
                relevant_dataframes = relevant_dataframes[["benchmark_timestamp", "benchmark_type", "best_interception_distance", "time_to_best_interception", "flight_time", "crashed", "voltage_reduction", "n_insects", "n_monsters"]]
                results_string += Utils.dataframe_to_orgmode_table(relevant_dataframes)

            if MEAN_VAR_PER_INSECT:
                results_string += "|------+----------+-----+-------+-------+------+----|\n"
                results_string += f"| Mean | | {np.mean(np.array(hunt_error_list))} | | | | |\n"
                results_string += f"| Var | | {np.var(np.array(hunt_error_list))} | | | | |\n"

            results_string += "\n"

        f.write(results_string)
