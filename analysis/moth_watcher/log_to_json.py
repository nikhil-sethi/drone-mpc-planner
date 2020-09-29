#!/usr/bin/env python3
import numpy as np
import pandas as pd
pd.options.mode.chained_assignment = None
import os, glob, json
import re, datetime
import argparse, socket
import pickle
from tqdm import tqdm
import datetime
from pathlib import Path

from scipy.interpolate import interp1d
from sklearn.base import BaseEstimator, TransformerMixin
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import Pipeline

def natural_sort(l):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(l, key=alphanum_key)

def rle(inarray):
        """ run length encoding. Partial credit to R rle function. 
            Multi datatype arrays catered for including non Numpy
            returns: tuple (runlengths, startpositions, values) """
        ia = np.asarray(inarray)                # force numpy
        n = len(ia)
        if n == 0: 
            return (None, None, None)
        else:
            y = np.array(ia[1:] != ia[:-1])     # pairwise unequal (string safe)
            i = np.append(np.where(y), n - 1)   # must include last element posi
            z = np.diff(np.append(-1, i))       # run lengths
            p = np.cumsum(np.append(0, z))[:-1] # positions
            return(z, p, ia[i])

def todatetime(string):
    return datetime.datetime.strptime(string, "%Y%m%d_%H%M%S")

def get_natural_cubic_spline_model(x, y, minval=None, maxval=None, n_knots=None, knots=None):
    """
    Get a natural cubic spline model for the data.

    For the knots, give (a) `knots` (as an array) or (b) minval, maxval and n_knots.

    If the knots are not directly specified, the resulting knots are equally
    space within the *interior* of (max, min).  That is, the endpoints are
    *not* included as knots.

    Parameters
    ----------
    x: np.array of float
        The input data
    y: np.array of float
        The outpur data
    minval: float 
        Minimum of interval containing the knots.
    maxval: float 
        Maximum of the interval containing the knots.
    n_knots: positive integer 
        The number of knots to create.
    knots: array or list of floats 
        The knots.

    Returns
    --------
    model: a model object
        The returned model will have following method:
        - predict(x):
            x is a numpy array. This will return the predicted y-values.
    """

    if knots:
        spline = NaturalCubicSpline(knots=knots)
    else:
        spline = NaturalCubicSpline(max=maxval, min=minval, n_knots=n_knots)

    p = Pipeline([
        ('nat_cubic', spline),
        ('regression', LinearRegression(fit_intercept=True))
    ])

    p.fit(x, y)

    return p

class AbstractSpline(BaseEstimator, TransformerMixin):
    """Base class for all spline basis expansions."""

    def __init__(self, max=None, min=None, n_knots=None, n_params=None, knots=None):
        if knots is None:
            if not n_knots:
                n_knots = self._compute_n_knots(n_params)
            knots = np.linspace(min, max, num=(n_knots + 2))[1:-1]
            max, min = np.max(knots), np.min(knots)
        self.knots = np.asarray(knots)

    @property
    def n_knots(self):
        return len(self.knots)

    def fit(self, *args, **kwargs):
        return self

class NaturalCubicSpline(AbstractSpline):
    """Apply a natural cubic basis expansion to an array.
    The features created with this basis expansion can be used to fit a
    piecewise cubic function under the constraint that the fitted curve is
    linear *outside* the range of the knots..  The fitted curve is continuously
    differentiable to the second order at all of the knots.
    This transformer can be created in two ways:
      - By specifying the maximum, minimum, and number of knots.
      - By specifying the cutpoints directly.  

    If the knots are not directly specified, the resulting knots are equally
    space within the *interior* of (max, min).  That is, the endpoints are
    *not* included as knots.
    Parameters
    ----------
    min: float 
        Minimum of interval containing the knots.
    max: float 
        Maximum of the interval containing the knots.
    n_knots: positive integer 
        The number of knots to create.
    knots: array or list of floats 
        The knots.
    """

    def _compute_n_knots(self, n_params):
        return n_params

    @property
    def n_params(self):
        return self.n_knots - 1

    def transform(self, X, **transform_params):
        X_spl = self._transform_array(X)
        if isinstance(X, pd.Series):
            col_names = self._make_names(X)
            X_spl = pd.DataFrame(X_spl, columns=col_names, index=X.index)
        return X_spl

    def _make_names(self, X):
        first_name = "{}_spline_linear".format(X.name)
        rest_names = ["{}_spline_{}".format(X.name, idx)
                      for idx in range(self.n_knots - 2)]
        return [first_name] + rest_names

    def _transform_array(self, X, **transform_params):
        X = X.squeeze()
        try:
            X_spl = np.zeros((X.shape[0], self.n_knots - 1))
        except IndexError: # For arrays with only one element
            X_spl = np.zeros((1, self.n_knots - 1))
        X_spl[:, 0] = X.squeeze()

        def d(knot_idx, x):
            def ppart(t): return np.maximum(0, t)

            def cube(t): return t*t*t
            numerator = (cube(ppart(x - self.knots[knot_idx]))
                         - cube(ppart(x - self.knots[self.n_knots - 1])))
            denominator = self.knots[self.n_knots - 1] - self.knots[knot_idx]
            return numerator / denominator

        for i in range(0, self.n_knots - 2):
            X_spl[:, i+1] = (d(i, X) - d(self.n_knots - 2, X)).squeeze()
        return X_spl

def interpolate_zeros_and_spline(vals):
    """Sometimes unexplainable zeros in a moth flight are found these zeros will be interpolated"""
    vals = np.trim_zeros(vals)
    idx = np.nonzero(vals)
    x = np.arange(len(vals))
    interp = interp1d(x[idx], vals[idx], kind="quadratic")
    y = interp(x)
    spline_model = get_natural_cubic_spline_model(x, y, minval=min(x), maxval=max(x), n_knots=7)
    return spline_model.predict(x)

def system_status_in_folder(folder):
    pats_xml_mode = ''
    pats_xml_path = Path(folder,'logging','pats.xml')
    if not os.path.exists(pats_xml_path):
        return ([],'error','')
    with open (pats_xml_path, "r") as pats_xml:
            xml_lines = pats_xml.readlines()
            for line in xml_lines:
                if line.find('op_mode') != -1:
                    pats_xml_mode = line.split('_')[3].split('<')[0]
                    break
    
    results_path = Path(folder,'logging','results.txt')
    if not os.path.exists(results_path):
        return ([],'error','')
    runtime = -1
    with open (results_path, "r") as results_txt:
            result_lines = results_txt.readlines()
            for line in result_lines:
                if line.find('Run_time') != -1:
                    runtime = float(line.split(':')[1])
                    break

    #open terminal log and check whether we was waiting for darkness
    terminal_log_path = Path(folder,'terminal.log')
    log_start = ''
    daylight_start = ''
    daylight_end = ''
    prev_line = ''
    with open (terminal_log_path, "r") as terminal_log:
        log_start = terminal_log.readline()
        while not daylight_end:
            line = terminal_log.readline()
            if 'Measured exposure' in line:
                daylight_start = line.split('Measured')[0]
                prev_line = line
                while True:
                    line = terminal_log.readline()
                    if not 'Measured exposure' in line:
                        daylight_end = prev_line.split('Measured')[0]
                        break
                    prev_line = line
            if line == '' and prev_line == '':
                daylight_start = log_start
                daylight_end = log_start
                break
    
    daylight_start = daylight_start.strip().replace('/','').replace(':','').replace(' ', '_')
    daylight_end = daylight_end.strip().replace('/','').replace(':','').replace(' ', '_')
    
    if 'Resetting cam' in log_start or log_start == '':
        return ([],'error','')
    log_start_datetime = datetime.datetime.strptime(log_start.strip(), '%d/%m/%Y %H:%M:%S')
    log_end_datetime = datetime.datetime.strptime( os.path.basename(folder), '%Y%m%d_%H%M%S')

    log_start = log_start_datetime.strftime('%Y%m%d_%H%M%S')
    operational_log_start = (log_end_datetime -  datetime.timedelta(seconds=runtime)).strftime('%Y%m%d_%H%M%S')
    
    if daylight_start != daylight_end:
            data_status1 = {"from" : daylight_start,
            "till" : daylight_end, 
            "mode" : 'wait_for_dark'
            }
            data_status2 = {"from" : operational_log_start,
            "till" : os.path.basename(folder), 
            "mode" : pats_xml_mode
            }
            return ([data_status1, data_status2],pats_xml_mode,operational_log_start)
    else:
        data_status = {"from" : operational_log_start,
        "till" : os.path.basename(folder), 
        "mode" : pats_xml_mode
        }
        return (data_status,pats_xml_mode,operational_log_start)

def hunts_in_folder(folder,operational_log_start):
    results_path = Path(folder,'logging','results.txt')
    if not os.path.exists(results_path):
        return []
    drone_flights = 0
    n_drone_detects = 0
    n_insects = 0
    n_takeoffs = 0
    n_landings = 0
    n_hunts = 0
    n_replay_hunts = 0
    best_interception_distance = -1
    drone_problem = 0
    with open (results_path, "r") as results_txt:
            result_lines = results_txt.readlines()
            for line in result_lines:
                if line.find('n_drone_detects') != -1:
                    n_drone_detects = int(line.strip().split(':')[1])
                if line.find('n_insects') != -1:
                    n_insects = int(line.strip().split(':')[1])
                if line.find('n_takeoffs') != -1:
                    n_takeoffs = int(line.strip().split(':')[1])
                if line.find('n_landings') != -1:
                    n_landings = int(line.strip().split(':')[1])
                if line.find('n_hunts') != -1:
                    n_hunts = int(line.strip().split(':')[1])
                if line.find('n_replay_hunts') != -1:
                    n_replay_hunts = int(line.strip().split(':')[1])
                if line.find('best_interception_distance') != -1:
                    best_interception_distance = float(line.strip().split(':')[1])
                if line.find('drone problem') != -1 or line.find('drone_problem') != -1 :
                    drone_problem = int(line.strip().split(':')[1])

    data_hunt = {"from" : operational_log_start,
    "till" : os.path.basename(folder), 
    "drone_flights" : drone_flights,
    "n_drone_detects" : n_drone_detects,
    "n_insects" : n_insects,
    "n_takeoffs" : n_takeoffs,
    "n_landings" : n_landings,
    "n_hunts" : n_hunts,
    "n_replay_hunts" : n_replay_hunts,
    "best_interception_distance" : best_interception_distance,
    "drone_problem" : drone_problem
    }

    return data_hunt


def moth_counts_in_folder(folder):

    #concat all csv files containing dates
    files = natural_sort([fp for fp in glob.glob(os.path.join(folder, "logging", "log*.csv")) if "itrk0" not in fp])
    try:
        header = files[-1] #last entry is the log.csv file, which contains the header for all other log*.csv files
        files.pop()  # remove the header file
        df = pd.read_csv(header, sep=";")
        df["filename"] = os.path.basename(header)
    except:
        print("No header file found") # normally happens when dir got no 'logging' dir
        return []

    #read_insect_file
    ins_path = os.path.join(folder, "logging", "log_itrk0.csv")
    df_ins = pd.read_csv(ins_path, sep=";")

    processed_df = df_ins[df_ins["foundL_insect"].notna()]
    
    # now calculate the mean and standard deviation of each suspected moth flight
    moth_found = processed_df["foundL_insect"].values
    
    elapsed_time = processed_df["time"].values
    RS_ID = processed_df["RS_ID"].values
    xs = processed_df["posX_insect"].values
    ys = processed_df["posY_insect"].values
    zs = processed_df["posZ_insect"].values

    moth_found[(xs == 0) | (ys == 0) | (zs == 0)] = 0 # if xyz values missing, say moth is not seen
    nums, inds, clas = rle(moth_found) # get all possible moth flight windows

    # add seperate moth flights together that are too close together to be two different flights
    seq_lens, start_ind = [], []
    if nums is not None:
        for i in range(len(nums)):
            if clas[i] == 1:

                #new
                start_ind.append(inds[i])
                seq_lens.append(nums[i])

    prev_rsid = None
    found_moths = []
    if start_ind is not None:
        for i in range(len(start_ind)):
            non_smooth_x_p = xs[start_ind[i]:start_ind[i]+seq_lens[i] - 1] # get the x coords for the moth flight
            non_smooth_y_p = ys[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            non_smooth_z_p = zs[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            if np.min([non_smooth_x_p.shape[0], non_smooth_y_p.shape[0], non_smooth_z_p.shape[0]]) < 50:
                continue
            x_p = interpolate_zeros_and_spline(non_smooth_x_p) # splines the tensor and removes trailing zeros
            y_p = interpolate_zeros_and_spline(non_smooth_y_p)
            z_p = interpolate_zeros_and_spline(non_smooth_z_p)

            t_p = elapsed_time[start_ind[i]:start_ind[i]+seq_lens[i] - 1][:x_p.shape[0]].astype('float64') # get all the elapsed time for this moth flight - the trailing zeros
            dt = t_p[1:]-t_p[:-1] # get time between elapsed times

            dx = (x_p[1:]-x_p[:-1]) / dt # get the derivative of x coords
            dy = (y_p[1:]-y_p[:-1]) / dt
            dz = (z_p[1:]-z_p[:-1]) / dt

            part_vel = np.sqrt(dx**2+dy**2+dz**2) 
            mean_vel = part_vel.mean()
            std_vel = part_vel.std()

            start = elapsed_time[start_ind[i]]
            end = elapsed_time[start_ind[i]+seq_lens[i] - 1]
            duration = end - start # total duration of the moth flight
            filename = 'unknown' #work around #455


            # # filter on image coords
            # x_img_p = x_img[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            # y_img_p = y_img[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            # dx_img = (x_img_p[1:]-x_img_p[:-1])
            # dy_img = (y_img_p[1:]-y_img_p[:-1])
            # correct_image_movement = np.abs(dx_img).mean() > 0.15 and np.abs(dy_img).mean() > 0.15

            FP_theshold = np.mean(np.ones(shape=(3,3)) - np.abs(np.corrcoef([x_p, y_p, z_p]))) >= 0.008
            correct_duration = duration >= 0.05 and duration < 25 # FP if moth flight duration is between certain seconds

            # if correct_dist_mean and correct_dist_std and correct_duration:
            if correct_duration and FP_theshold:
                v = np.vstack([dx,dy,dz])
                p1 = np.divide(np.sum(v[:,1:] * v[:,:-1], axis=0), part_vel[1:] * part_vel[:-1])
                p1[p1 > 1] = 1
                p1[p1 < -1] = -1
                turning_angle = np.arccos(p1)
                p = np.vstack([x_p,y_p,z_p])
                np.seterr(divide='ignore')
                radius_of_curve = np.divide(np.linalg.norm(p[:,2:] - p[:,:-2]), (2 * np.sin(turning_angle)))
                radial_accelaration = np.divide(np.sum(v[:,1:] * v[:,:-1], axis=0), radius_of_curve)

                moth_flight_time = (dts + datetime.timedelta(seconds=elapsed_time[start_ind[i]])).strftime("%m/%d/%Y, %H:%M:%S")
                
                if args.v:
                
                    print(f"RS_ID: {RS_ID[start_ind[i]]} - {RS_ID[start_ind[i]+seq_lens[i] - 1]} - {prev_rsid}")
                    print(filename, correct_duration, FP_theshold)
                    print(f"Start time: {moth_flight_time}")
                    print(f"Moth flight length: {duration} s, {x_p.shape}")
                    # print(f"Start: {x_p[0], y_p[0], z_p[0]}")
                    print(np.mean(np.ones(shape=(3,3)) - np.abs(np.corrcoef([x_p, y_p, z_p]))))
                    fig = plt.figure()
                    ax = fig.gca(projection='3d')
                    ax.plot(x_p, -y_p, z_p, label='smoothend moth flight')
                    ax.scatter(non_smooth_x_p, -non_smooth_y_p, non_smooth_z_p, label='raw moth flight', marker="x")
                    ax.scatter([-5], [5], [-5], label='anchor', marker="o")
                    ax.legend()
                    ax.set_xlabel('X axis')
                    ax.set_ylabel('Y axis')
                    ax.set_zlabel('Z axis')
                    plt.show()
                    prev_rsid = RS_ID[start_ind[i]+seq_lens[i] - 1]

                # add frames together
                moth_data = {"time" : moth_flight_time,
                                "duration": duration,
                                "RS_ID" : str(RS_ID[start_ind[i]]),
                                "Filename" : filename,
                                "Vel_mean" : mean_vel, # all mean max and std
                                "Vel_std" : std_vel,
                                "Vel_max" : part_vel.max(),
                                "TA_mean" : turning_angle.mean(),
                                "TA_std" : turning_angle.std(),
                                "TA_max" : turning_angle.max(),
                                "RA_mean" : radial_accelaration.mean(),
                                "RA_std" : radial_accelaration.std(),
                                "RA_max" : radial_accelaration.max()}
                found_moths.append(moth_data)

    return found_moths

parser = argparse.ArgumentParser(description='Script that counts the number of detected moths in a directory, bound by the minimum and maximum date.')
parser.add_argument('-i', help="Path to the folder with logs", required=True)
parser.add_argument('-s', help="Directory date to start from", default="20000101_000000", required=False)
parser.add_argument('-e', help="Directory date to end on", default="30000101_000000", required=False)
parser.add_argument('-v', help="View the path of the found moth flights in a plot", required=False, default=False, action='store_true')
parser.add_argument('--filename', help="Path and filename to store results in", default="./moth.json")
parser.add_argument('--system', help="Override system name", default=socket.gethostname())
args = parser.parse_args()

min_date = todatetime(args.s)
max_date = todatetime(args.e)

if args.v:
    import matplotlib as mpl
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    mpl.rcParams['legend.fontsize'] = 10

found_dirs = glob.glob(args.i + "/202*_*")
filtered_dirs = [d for d in found_dirs if todatetime(os.path.basename(os.path.normpath(d))) >= min_date and todatetime(os.path.basename(os.path.normpath(d))) <= max_date] # filter the list of dirs to only contain dirs between certain dates

moths = []
statuss = []
hunts = []

pbar = tqdm(filtered_dirs)
for folder in pbar:
    pbar.set_description(folder)

    top_folder = os.path.basename(folder)
    dts = datetime.datetime.strptime(top_folder,'%Y%m%d_%H%M%S')
    if dts > min_date and dts <= max_date:
        
        status_in_dir,mode,operational_log_start = system_status_in_folder(folder)
        if status_in_dir != []:
            statuss.append(status_in_dir) 

        if mode == 'monitoring':
            moths_in_dir = moth_counts_in_folder(folder)
            if moths_in_dir != []:
                [moths.append(moth) for moth in moths_in_dir]
        elif mode == 'hunt' or mode == 'deploy':
            hunts_in_dir = hunts_in_folder(folder,operational_log_start)
            if hunts_in_dir != []:
                hunts.append(hunts_in_dir) 

data_moth = {"from" : args.s,
        "till" : args.e, 
        "moth_counts" : len(moths),
        "moths" : moths,
        "hunts" : hunts,
        "mode" : statuss,
        "system" : args.system}
with open(args.filename, 'w') as outfile:
    json.dump(data_moth, outfile)
print("Counting complete, saved in {0}".format(args.filename))
