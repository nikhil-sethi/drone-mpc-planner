#!/usr/bin/env python3
import argparse
import datetime
from pathlib import Path

import numpy as np
import pandas as pd


class EntryParser:
    def __init__(self, entry_root_path):
        self.entry_root_path = entry_root_path
        self.entries_df = pd.DataFrame(
            columns=[
                "entry_file_path",
                "take_off_datetime",
                "land_datetime",
                "flight_time",
                "crashed",
                "best_interception_distance",
                "time_to_best_interception",
                "pos_best_interception",
                "vel_best_interception",
                "acc_best_interception",
                "voltage_reduction",
                "n_monsters",
                "n_insects",
                "executor_hash",
                "benchmark_type",
                "benchmark_timestamp",
                "benchmark_entry_id",
                "benchmark_hash",
            ]
        )

    def find_entries(self):
        for path in Path(self.entry_root_path).rglob("flight_results*.txt"):
            file = open(path, "r")
            lines = file.readlines()
            if self.check_if_benchmark_entry(lines):
                self.fill_entry(lines, path)

    def check_if_benchmark_entry(self, lines):
        for line in lines:
            if line.find("benchmark_timestamp") != -1:
                return True
        return False

    def fill_entry(self, lines, entry_file_path):
        take_off_datetime, land_datetime, flight_time = None, None, None
        crashed = None
        best_interception_distance, time_to_best_interception = None, None
        pos_best_interception_x, pos_best_interception_y, pos_best_interception_z = None, None, None
        vel_best_interception_x, vel_best_interception_y, vel_best_interception_z = None, None, None
        acc_best_interception_x, acc_best_interception_y, acc_best_interception_z = None, None, None
        voltage_reduction = None
        n_monsters, n_insects = None, None
        executor_hash = None
        benchmark_type, benchmark_timestamp, benchmark_entry_id, benchmark_hash = None, None, None, None

        for line in lines:
            if line.find("take_off_datetime") != -1:
                take_off_datetime = datetime.datetime.strptime(
                    ":".join(line.strip().split(":")[1:]), "%Y/%m/%d %H:%M:%S")
            if line.find("land_datetime") != -1:
                land_datetime = datetime.datetime.strptime(
                    ":".join(line.strip().split(":")[1:]), "%Y/%m/%d %H:%M:%S")
            if line.find("flight_time") != -1:
                flight_time = line.strip().split(":")[1]
            if line.find("crashed") != -1:
                crashed = line.strip().split(":")[1]
                if int(crashed) == 1:
                    crashed = True
                else:
                    crashed = False
            if line.find("best_interception_distance") != -1:
                best_interception_distance = line.strip().split(":")[1]
            if line.find("time_to_best_interception") != -1:
                time_to_best_interception = line.strip().split(":")[1]
            if line.find("pos_best_interception_xyz") != -1:
                (
                    pos_best_interception_x,
                    pos_best_interception_y,
                    pos_best_interception_z,
                ) = (
                    line.strip().split(":")[1].split(",")
                )
            if line.find("vel_best_interception_xyz") != -1:
                (
                    vel_best_interception_x,
                    vel_best_interception_y,
                    vel_best_interception_z,
                ) = (
                    line.strip().split(":")[1].split(",")
                )
            if line.find("acc_best_interception_xyz") != -1:
                (
                    acc_best_interception_x,
                    acc_best_interception_y,
                    acc_best_interception_z,
                ) = (
                    line.strip().split(":")[1].split(",")
                )
            if line.find("voltage_reduction") != -1:
                voltage_reduction = line.strip().split(":")[1]
            if line.find("n_monsters") != -1:
                n_monsters = line.strip().split(":")[1]
            if line.find("n_insects") != -1:
                n_insects = line.strip().split(":")[1]
            if line.find("executor_hash") != -1:
                executor_hash = line.strip().split(":")[1]
            if line.find("benchmark_type") != -1:
                benchmark_type = line.strip().split(":")[1]
            if line.find("benchmark_timestamp") != -1:
                benchmark_timestamp = datetime.datetime.strptime(
                    line.strip().split(":")[1], '%Y%m%d_%H%M%S')
            if line.find("benchmark_entry_id") != -1:
                benchmark_entry_id = line.strip().split(":")[1]
            if line.find("benchmark_hash") != -1:
                benchmark_hash = line.strip().split(":")[1]

        _new_row = pd.DataFrame(
            [
                {
                    "entry_file_path": entry_file_path,
                    "take_off_datetime": take_off_datetime,
                    "land_datetime": land_datetime,
                    "flight_time": float(flight_time),
                    "crashed": crashed,
                    "best_interception_distance": float(best_interception_distance),
                    "time_to_best_interception": float(time_to_best_interception),
                    "pos_best_interception": [
                        pos_best_interception_x,
                        pos_best_interception_y,
                        pos_best_interception_z,
                    ],
                    "vel_best_interception": [
                        vel_best_interception_x,
                        vel_best_interception_y,
                        vel_best_interception_z,
                    ],
                    "acc_best_interception": [
                        acc_best_interception_x,
                        acc_best_interception_y,
                        acc_best_interception_z,
                    ],
                    "voltage_reduction": voltage_reduction,
                    "n_monsters": n_monsters,
                    "n_insects": n_insects,
                    "executor_hash": executor_hash,
                    "benchmark_type": benchmark_type,
                    "benchmark_timestamp": benchmark_timestamp,
                    "benchmark_entry_id": int(benchmark_entry_id),
                    "benchmark_hash": benchmark_hash,
                }
            ]
        )
        self.entries_df = pd.concat([_new_row, self.entries_df.loc[:]]).reset_index(
            drop=True
        )


class Results:
    def __init__(self):
        self.benchmark_results_df = None
        self.entry_results_df_dict = {}

    def verify_entries(self, entries_df):
        assert (len(entries_df) > 0), "No entries found."
        assert len(entries_df["benchmark_hash"].unique()
                   ) == 1, "Multiple benchmark types found."
        assert entries_df.isna().values.any() == False, "NaN values found."
        assert entries_df.duplicated(
            subset=['benchmark_timestamp', 'benchmark_entry_id']).any() != True, "Duplicate entries found."
        assert len(entries_df['benchmark_timestamp'].unique()) == len(
            entries_df.drop_duplicates(['benchmark_timestamp', 'executor_hash']).index), "Variation in executor hash during benchmark found."
        assert (entries_df['land_datetime'] > entries_df['take_off_datetime']
                ).all(), "Landing before take-off."
        assert (entries_df['take_off_datetime'] > entries_df['benchmark_timestamp']
                ).all(), "Take-off before benchmark initialization."
        assert (entries_df['flight_time'].astype(float) > 0).all(
        ), "Flight time is non-positive."

    def return_flight_fraction(self, df, total_count):
        return f"{len(df)}/{total_count}"

    def kill_count(self, df):
        return df[df < MAX_KILL_DISTANCE].count()

    def return_hash(self, df):
        _hash = np.unique(df)
        assert len(_hash) == 1, "a hash changed within single benchmark run"
        return _hash[0]

    def parse_benchmark_results(self, entries_df, benchmark_df):
        entries = entries_df.copy()
        entries.replace(
            [np.inf, -np.inf], np.nan, inplace=True)
        entries.dropna(inplace=True)
        self.benchmark_results_df = entries.groupby("benchmark_timestamp", as_index=False).agg(
            number_of_flights=pd.NamedAgg(
                column='best_interception_distance', aggfunc=lambda x: self.return_flight_fraction(x, len(benchmark_df))),
            mean_best_interception_distance=pd.NamedAgg(
                column='best_interception_distance', aggfunc=np.mean),
            std_best_interception_distance=pd.NamedAgg(
                column='best_interception_distance', aggfunc=np.std),
            number_of_kills=pd.NamedAgg(
                column='best_interception_distance', aggfunc=self.kill_count),
            number_of_crashes=pd.NamedAgg(
                column='crashed', aggfunc=np.sum),
            benchmark_hash=pd.NamedAgg(
                column='benchmark_hash', aggfunc=self.return_hash),
            executor_hash=pd.NamedAgg(
                column='executor_hash', aggfunc=self.return_hash),
        )

    def parse_entry_results(self, entries_df):
        entries = entries_df.copy()
        benchmark_entry_ids = sorted(entries['benchmark_entry_id'].unique())

        per_entry_dict = {elem: pd.DataFrame() for elem in benchmark_entry_ids}
        for key in per_entry_dict.keys():
            per_entry_dict[key] = entries[:][entries['benchmark_entry_id'] == key]

        self.entry_results_df_dict = per_entry_dict

    def parse_benchmark(self):
        benchmark_csv_path = "benchmark/benchmark.csv"
        self.benchmark_df = pd.read_csv(benchmark_csv_path, delimiter=";")


class Writer:
    def __init__(self):
        self.file = None
        self.results_string = None
        pass

    def write_ascii_tag(self):
        return """\
    /\                 /\\
   / \'._   (\_/)   _.'/ \\
 /_.''._'--('.')--'_.''._\\
 | \_ / `;=/ " \=;` \ _/ |
  \/ `\__|`\___/`|__/`  \/
pats`     \(/|\)/       `
           " ` "
           \n\n"""

    def form_entry_information_header(self, entry_id, benchmark_df):
        _relevant_df = benchmark_df.iloc[entry_id - 1]
        _type = _relevant_df['type']
        _string = f"**Entry {entry_id}**\n{_type} moth"
        if _type == 'virtual':
            _entry_evasion_trigger = "spinup" if _relevant_df["evasion_trigger"] == 1 else "hunt error"
            _entry_evasion_type = "diving" if _relevant_df[
                "evasion_type"] == 1 else "u-turn" if _relevant_df["evasion_type"] == 2 else "none"
            _string += f" (pos: [{_relevant_df['pos_x']}, {_relevant_df['pos_y']}, {_relevant_df['pos_z']}], "
            _string += f"vel: [{_relevant_df['vel_x']}, {_relevant_df['vel_y']}, {_relevant_df['vel_z']}], "
            _string += f"evasion trigger: {_entry_evasion_trigger}, evasion type: {_entry_evasion_type})\n"
        elif _type == 'replay':
            _string += f" (replay id: {benchmark_df['id'].values[entry_id-1]})\n"
        else:
            raise Exception(f"Unknown benchmark entry type: {_type}")
        return _string

    def form_string(self, benchmark_results_df, per_entry_results_df_dict, benchmark_df):
        self.results_string = f"Benchmark log path: {args.i}\n\n"
        self.results_string += self.write_ascii_tag()
        self.results_string += self.dataframe_to_orgmode_table(
            benchmark_results_df)
        for entry_id in range(1, len(benchmark_df) + 1):
            self.results_string += self.form_entry_information_header(
                entry_id, benchmark_df)
            try:
                self.results_string += self.dataframe_to_orgmode_table(
                    per_entry_results_df_dict[entry_id][['benchmark_timestamp', 'benchmark_type', 'best_interception_distance',
                                                         'time_to_best_interception', 'flight_time', 'crashed', 'voltage_reduction', 'n_insects', 'n_monsters', 'entry_file_path']]
                )
            except:
                self.results_string += "\n"
                pass
        self.results_string += "\n"

    def write_file(self):
        self.file = open("benchmark/benchmark_results.org", "w+")
        self.file.write(self.results_string)
        self.file.close()

    def dataframe_to_orgmode_table(self, dataframe):
        dataframe = (
            dataframe.to_csv(sep="|", header=True,
                             index=False).strip("\n").split("\n")
        )
        dataframe.insert(1, "-+-")
        dataframe_string = "|\n|".join(dataframe)
        return f"|{dataframe_string}|\n\n"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Script that parses flight logs of benchmarks into an overview.')
    parser.add_argument(
        '-i', help="Path to the folder with log folders", required=True)
    args = parser.parse_args()

    TOTAL_BENCHMARK_ENTRIES = 16
    DROP_DUPLICATES = False
    MAX_KILL_DISTANCE = 0.05
    MEAN_VAR_PER_INSECT = False

    entry_parser = EntryParser(args.i)
    entry_parser.find_entries()

    results = Results()
    results.parse_benchmark()
    results.verify_entries(entry_parser.entries_df)
    results.parse_benchmark_results(
        entry_parser.entries_df, results.benchmark_df)
    results.parse_entry_results(entry_parser.entries_df)

    writer = Writer()
    writer.form_string(results.benchmark_results_df,
                       results.entry_results_df_dict,
                       results.benchmark_df)
    writer.write_file()
