from pathlib import Path
import pandas as pd


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class BenchmarkResults():
    def __init__(self):
        self.benchmark_timestamp = None

        self.number_of_flights = None

        # hunt accuracy
        self.mean_best_interception_distance = None
        self.var_best_interception_distance = None

        self.mean_best_interception_distance_virtual = None
        self.var_best_interception_distance_virtual = None

        self.mean_best_interception_distance_replay = None
        self.var_best_interception_distance_replay = None

        # flight time
        self.mean_flight_time = None
        self.var_flight_time = None

        self.mean_flight_time_virtual = None
        self.var_flight_time_virtual = None

        self.mean_flight_time_replay = None
        self.var_flight_time_replay = None

        # crashes
        self.number_of_crashes = None


class BenchmarkEntry():
    def __init__(self):
        self.lines = None
        self.entry_file_path = None
        self.take_off_datetime = None
        self.land_datetime = None
        self.flight_time = None
        self.crashed = None
        self.best_interception_distance = None
        self.benchmark_type = None
        self.benchmark_timestamp = None
        self.benchmark_insect_pos_x = None
        self.benchmark_insect_pos_y = None
        self.benchmark_insect_pos_z = None
        self.benchmark_insect_vel_x = None
        self.benchmark_insect_vel_y = None
        self.benchmark_insect_vel_z = None


class BenchmarkParser():
    def __init__(self, search_path):
        self.file_path = search_path
        self.benchmark_entries = []
        self.sorted_benchmark_entries = {}
        self.dataframe = None
        self.benchmark_results = {}

    def find_benchmark_entries(self):
        for path in Path(self.file_path).rglob('flight_results*.txt'):
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
                if line.find('take_off_datetime') != -1:
                    take_off_datetime = line.strip().split(':')[1]
                    entry.take_off_datetime = take_off_datetime
                if line.find('land_datetime') != -1:
                    land_datetime = line.strip().split(':')[1]
                    entry.land_datetime = land_datetime
                if line.find('flight_time') != -1:
                    flight_time = line.strip().split(':')[1]
                    entry.flight_time = flight_time
                if line.find('crashed') != -1:
                    crashed = line.strip().split(':')[1]
                    entry.crashed = crashed
                if line.find('best_interception_distance') != -1:
                    best_interception_distance = line.strip().split(':')[1]
                    if best_interception_distance == 'inf':
                        best_interception_distance = 2
                    entry.best_interception_distance = best_interception_distance
                if line.find('benchmark_type') != -1:
                    benchmark_type = line.strip().split(':')[1]
                    entry.benchmark_type = benchmark_type
                if line.find('benchmark_timestamp') != -1:
                    benchmark_timestamp = line.strip().split(':')[1]
                    entry.benchmark_timestamp = benchmark_timestamp
                if line.find('benchmark_insect_pos_x') != -1:
                    benchmark_insect_pos_x = line.strip().split(':')[1]
                    entry.benchmark_insect_pos_x = benchmark_insect_pos_x
                if line.find('benchmark_insect_pos_y') != -1:
                    benchmark_insect_pos_y = line.strip().split(':')[1]
                    entry.benchmark_insect_pos_y = benchmark_insect_pos_y
                if line.find('benchmark_insect_pos_z') != -1:
                    benchmark_insect_pos_z = line.strip().split(':')[1]
                    entry.benchmark_insect_pos_z = benchmark_insect_pos_z
                if line.find('benchmark_insect_vel_x') != -1:
                    benchmark_insect_vel_x = line.strip().split(':')[1]
                    entry.benchmark_insect_vel_x = benchmark_insect_vel_x
                if line.find('benchmark_insect_vel_y') != -1:
                    benchmark_insect_vel_y = line.strip().split(':')[1]
                    entry.benchmark_insect_vel_y = benchmark_insect_vel_y
                if line.find('benchmark_insect_vel_z') != -1:
                    benchmark_insect_vel_z = line.strip().split(':')[1]
                    entry.benchmark_insect_vel_z = benchmark_insect_vel_z
                if line.find('benchmark_replay_id') != -1:
                    benchmark_replay_id = line.strip().split(':')[1]
                    entry.benchmark_replay_id = benchmark_replay_id
                if line.find('benchmark_entry_id') != -1:
                    benchmark_entry_id = line.strip().split(':')[1]
                    entry.benchmark_entry_id = benchmark_entry_id

    def check_if_benchmark_entry(self, lines):
        for line in lines:
            if line.find('benchmark_timestamp') != -1:
                return True
        return False

    def sort_benchmark_entries(self):
        for entry in self.benchmark_entries:
            if entry.benchmark_timestamp not in self.sorted_benchmark_entries:
                self.sorted_benchmark_entries[entry.benchmark_timestamp] = []
            self.sorted_benchmark_entries[entry.benchmark_timestamp].append(
                entry)

    def assemble_dataframe(self):
        self.dataframe = pd.DataFrame(columns=['benchmark_timestamp', 'benchmark_type', 'benchmark_insect_pos_x', 'benchmark_insect_pos_y', 'benchmark_insect_pos_z',
                                      'benchmark_insect_vel_x', 'benchmark_insect_vel_y', 'benchmark_insect_vel_z', 'take_off_datetime', 'land_datetime', 'flight_time', 'crashed', 'best_interception_distance', 'benchmark_replay_id', 'benchmark_entry_id'])
        for entry in self.benchmark_entries:
            _new_row = pd.DataFrame([{'benchmark_timestamp': entry.benchmark_timestamp, 'benchmark_type': entry.benchmark_type, 'benchmark_insect_pos_x': entry.benchmark_insect_pos_x, 'benchmark_insect_pos_y': entry.benchmark_insect_pos_y, 'benchmark_insect_pos_z': entry.benchmark_insect_pos_z, 'benchmark_insect_vel_x': entry.benchmark_insect_vel_x,
                                      'benchmark_insect_vel_y': entry.benchmark_insect_vel_y, 'benchmark_insect_vel_z': entry.benchmark_insect_vel_z, 'take_off_datetime': entry.take_off_datetime, 'land_datetime': entry.land_datetime, 'flight_time': entry.flight_time, 'crashed': entry.crashed, 'best_interception_distance': entry.best_interception_distance, 'benchmark_replay_id': entry.benchmark_replay_id, 'benchmark_entry_id': entry.benchmark_entry_id}])
            self.dataframe = pd.concat(
                [_new_row, self.dataframe.loc[:]]).reset_index(drop=True)

    def calculate_mean_and_var(self, benchmark_timestamp, column, insect_type=None):
        _filtered_dataframe = self.dataframe[self.dataframe['benchmark_timestamp']
                                             == benchmark_timestamp]

        if insect_type == 'virtual' or insect_type == 'replay':
            _filtered_dataframe = _filtered_dataframe[_filtered_dataframe['benchmark_type'] == insect_type]
        elif insect_type is not None:
            raise Exception('Unknown insect type')

        _filtered_dataframe[column] = pd.to_numeric(
            _filtered_dataframe[column])

        _mean = _filtered_dataframe[column].mean()
        _var = _filtered_dataframe[column].var()
        return _mean, _var


    def calculate_benchmark_score(self, kpi):
        for benchmark in self.sorted_benchmark_entries.keys():
            results = BenchmarkResults()
            results.benchmark_timestamp = benchmark

            _number_of_flights = 0
            print(f"{bcolors.OKBLUE}Calculating benchmark score for {benchmark}...{bcolors.ENDC}")

            for entry in self.sorted_benchmark_entries[benchmark]:
                _number_of_flights += 1

                if entry.benchmark_type == 'virtual':
                    pass
                elif entry.benchmark_type == 'replay':
                    pass
                else:
                    raise Exception('Unknown benchmark type')

            results.number_of_flights = _number_of_flights

            results.mean_best_interception_distance, results.var_best_interception_distance = self.calculate_mean_and_var(benchmark, 'best_interception_distance')
            results.mean_best_interception_distance_replay, results.var_best_interception_distance_replay = self.calculate_mean_and_var(benchmark, 'best_interception_distance', 'replay')
            results.mean_best_interception_distance_virtual, results.var_best_interception_distance_virtual = self.calculate_mean_and_var(benchmark, 'best_interception_distance', 'virtual')

            results.mean_flight_time, results.var_flight_time = self.calculate_mean_and_var(benchmark, 'flight_time')
            results.mean_flight_time_replay, results.var_flight_time_replay = self.calculate_mean_and_var(benchmark, 'flight_time', 'replay')
            results.mean_flight_time_virtual, results.var_flight_time_virtual = self.calculate_mean_and_var(benchmark, 'flight_time', 'virtual')

            results.mean_crashed, results.var_crashed = self.calculate_mean_and_var(benchmark, 'crashed')
            results.mean_crashed_replay, results.var_crashed_replay = self.calculate_mean_and_var(benchmark, 'crashed', 'replay')
            results.mean_crashed_virtual, results.var_crashed_virtual = self.calculate_mean_and_var(benchmark, 'crashed', 'virtual')
            if benchmark not in self.benchmark_results:
                self.benchmark_results[benchmark] = results
            else:
                raise Exception('Benchmark already exists')





"""WHAT TO DO WITH THE RESULTS"""
# we can do an excel like thing with color formatting with per column a A) benchmark and B) a benchmark entry and per row a kpi
# in the gathered results (A) we can do a variance std dev per kpi
# we can also do bar plots for the most important ones?


if __name__ == "__main__":
    TOTAL_BENCHMARK_ENTRIES = 5

    parser = BenchmarkParser("/home/gemenerik/Downloads/benchmarksmon23jan")

    print(f"{bcolors.HEADER}Finding benchmarks...{bcolors.ENDC}")
    parser.find_benchmark_entries()
    parser.fill_benchmark_entries()

    print(f"{bcolors.HEADER}Sorting benchmarks...{bcolors.ENDC}")
    parser.sort_benchmark_entries()

    print(f"{bcolors.HEADER}Assembling dataframe...{bcolors.ENDC}")
    parser.assemble_dataframe()

    print(f"{bcolors.HEADER}Calculating benchmark results...{bcolors.ENDC}\n")
    parser.calculate_benchmark_score(kpi=["best_interception_distance"])

    print(f"{bcolors.HEADER}Benchmark results:{bcolors.ENDC}\n")
    print(f"{bcolors.HEADER}------------------{bcolors.ENDC}\n")
    with open('benchmark_results.org', 'w') as f:
        results_table="| Timestamp | No. flights | Hunt error mean | Hunt error var | Flight time mean | Flight time var | Crashes |\n|--------+-----+--------+---------+----------------+-----------------+-------|\n"
        for _benchmark_time_date in sorted(parser.benchmark_results.keys()):
            print(f"{bcolors.OKGREEN}{_benchmark_time_date}{bcolors.ENDC}")
            print(f"Number of flights: {parser.benchmark_results[_benchmark_time_date].number_of_flights} / 5")
            print("")
            print(f"{bcolors.OKBLUE}Best interception distance:{bcolors.ENDC}")
            print("Mean, var best interception distance:", parser.benchmark_results[_benchmark_time_date].mean_best_interception_distance, parser.benchmark_results[_benchmark_time_date].var_best_interception_distance)
            print("Mean, var best interception distance replay:", parser.benchmark_results[_benchmark_time_date].mean_best_interception_distance_replay, parser.benchmark_results[_benchmark_time_date].var_best_interception_distance_replay)
            print("Mean, var best interception distance virtual:", parser.benchmark_results[_benchmark_time_date].mean_best_interception_distance_virtual, parser.benchmark_results[_benchmark_time_date].var_best_interception_distance_virtual)
            print("")
            print(f"{bcolors.OKBLUE}Flight time:{bcolors.ENDC}")
            print("Mean, var flight time:", parser.benchmark_results[_benchmark_time_date].mean_flight_time, parser.benchmark_results[_benchmark_time_date].var_flight_time)
            print("Mean, var flight time replay:", parser.benchmark_results[_benchmark_time_date].mean_flight_time_replay, parser.benchmark_results[_benchmark_time_date].var_flight_time_replay)
            print("Mean, var flight time virtual:", parser.benchmark_results[_benchmark_time_date].mean_flight_time_virtual, parser.benchmark_results[_benchmark_time_date].var_flight_time_virtual)
            print("")
            print(f"{bcolors.OKBLUE}Crashes:{bcolors.ENDC}")
            print("Mean, var crashes:", parser.benchmark_results[_benchmark_time_date].mean_crashed, parser.benchmark_results[_benchmark_time_date].var_crashed)
            print("Mean, var crashes replay:", parser.benchmark_results[_benchmark_time_date].mean_crashed_replay, parser.benchmark_results[_benchmark_time_date].var_crashed_replay)
            print("Mean, var crashes virtual:", parser.benchmark_results[_benchmark_time_date].mean_crashed_virtual, parser.benchmark_results[_benchmark_time_date].var_crashed_virtual)

            results_table += f"| {_benchmark_time_date} | {parser.benchmark_results[_benchmark_time_date].number_of_flights} / {TOTAL_BENCHMARK_ENTRIES} | {parser.benchmark_results[_benchmark_time_date].mean_best_interception_distance} | {parser.benchmark_results[_benchmark_time_date].var_best_interception_distance} | {parser.benchmark_results[_benchmark_time_date].mean_flight_time} | {parser.benchmark_results[_benchmark_time_date].var_flight_time} | {parser.benchmark_results[_benchmark_time_date].mean_crashed} |\n"
        f.write(results_table)