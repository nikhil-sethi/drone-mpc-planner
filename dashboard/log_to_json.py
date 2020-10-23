#!/usr/bin/env python3
import os, glob, json,math,re,datetime, argparse, socket, pickle
import numpy as np
import pandas as pd
pd.options.mode.chained_assignment = None
from tqdm import tqdm
from pathlib import Path

version = "1.2"

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
    """Sometimes unexplainable zeros in a insect flight are found these zeros will be interpolated"""
    vals = np.trim_zeros(vals)
    if len(vals) < 2:
        return vals
    idx = np.nonzero(vals)
    if len(idx) < 3:
        return vals
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


def detection_counts_in_folder(folder):

    #concat all csv files containing dates
    detection_fns = natural_sort([fp for fp in glob.glob(os.path.join(folder, "logging", "log_*.csv")) if "itrk0" not in fp])

    valid_detections = []
    prev_RS_ID = -1
    for detection_fn in detection_fns:
        with open (detection_fn, "r") as detection_log:
            log = pd.read_csv(detection_fn, sep=";")

            elapsed_time = log["time"].values
            lost = log['n_frames_lost_insect'].values > 0
            remove_ids = [i for i, x in enumerate(lost) if x]
            if len(elapsed_time) < 20 or len(elapsed_time) - len(remove_ids) < 5:
                continue

            RS_ID = log['RS_ID'].values
            xs = log['sposX_insect'].values
            ys = log['sposY_insect'].values
            zs = log['sposZ_insect'].values

            #FIXME: I don't think this works very well:
            x_ip = interpolate_zeros_and_spline(xs)
            y_ip = interpolate_zeros_and_spline(ys)
            z_ip = interpolate_zeros_and_spline(zs)

            time = elapsed_time.astype('float64')
            dt = time[-1]-time[0]

            vxs = log['svelX_insect'].values
            vys = log['svelY_insect'].values
            vzs = log['svelZ_insect'].values


            vx = np.delete(vxs,remove_ids)
            vy = np.delete(vys,remove_ids)
            vz = np.delete(vzs,remove_ids)


            v = np.sqrt(vx**2+vy**2+vz**2)
            v_mean = v.mean()
            v_std = v.std()

            filtered_elepased = np.delete(elapsed_time,remove_ids)

            start = filtered_elepased[0]
            end = filtered_elepased[-1]
            duration = end - start
            first_RS_ID = str(RS_ID[0])
            filename = os.path.basename(detection_fn)
            video_filename = os.path.dirname(detection_fn) + '/' + filename.replace('log_itrk','moth').replace('csv','mkv')
            if not os.path.exists(video_filename):
                video_filename = 'NA'
            else:
                video_filename = os.path.basename(video_filename)

            FP = np.mean(np.ones(shape=(3,3)) - np.abs(np.corrcoef([x_ip, y_ip, z_ip])))
            FP_OK = FP >= 0.008
            duration_OK = duration >= 0.05 and duration < 25 # FP if insect flight duration is between certain seconds

            # if correct_dist_mean and correct_dist_std and correct_duration:
            if duration_OK and FP_OK:
                #I don't know what this does, but it is incorrect #517
                v_stacked = np.vstack([vx,vy,vz])
                p1 = np.divide(np.sum(v_stacked[:,1:] * v_stacked[:,:-1], axis=0), v[1:] * v[:-1])
                p1[p1 > 1] = 1
                p1[p1 < -1] = -1
                turning_angle = np.arccos(p1)
                p = np.vstack([x_ip,y_ip,z_ip])
                np.seterr(divide='ignore')
                radius_of_curve = np.divide(np.linalg.norm(p[:,2:] - p[:,:-2]), (2 * np.sin(turning_angle)))
                radial_accelaration = np.divide(np.sum(v_stacked[:,1:] * v_stacked[:,:-1], axis=0), radius_of_curve)

                detection_time = (dts + datetime.timedelta(seconds=elapsed_time[0])).strftime('%Y%m%d_%H%M%S')

                if args.v:
                    print(filename)
                    print(f'RS_ID: {RS_ID[0]} - {RS_ID[-1]} - {prev_RS_ID}')
                    print(f'Insect flight length: {"{:.2f}".format(duration)} s')
                    print(f'FP: {"{:.2f}".format(FP)}')
                    fig = plt.figure()
                    ax = fig.gca(projection='3d')
                    ax.plot(x_ip, -y_ip, z_ip, label='Insect flight')
                    ax.scatter([-5], [5], [-5], label='anchor', marker="o")
                    ax.legend()
                    ax.set_xlabel('X axis')
                    ax.set_ylabel('Y axis')
                    ax.set_zlabel('Z axis')
                    plt.show()

                detection_data = {"time" : detection_time,
                                "duration": duration,
                                "RS_ID" : first_RS_ID,
                                "Vel_mean" : v_mean,
                                "Vel_std" : v_std,
                                "Vel_max" : v.max(),
                                "TA_mean" : str(turning_angle.mean()),
                                "TA_std" : str(turning_angle.std()),
                                "TA_max" : str(turning_angle.max()),
                                "RA_mean" : str(radial_accelaration.mean()),
                                "RA_std" : str(radial_accelaration.std()),
                                "RA_max" : str(radial_accelaration.max()),
                                "FP" : FP,
                                "Filename" : filename,
                                "Video_Filename" : video_filename,
                                "Mode" : 'monitoring',
                                "Version": version
                            }
                valid_detections.append(detection_data)

            prev_RS_ID = first_RS_ID

    return valid_detections

parser = argparse.ArgumentParser(description='Script that counts the number of valid insect detections in a directory, bound by the minimum and maximum date.')
parser.add_argument('-i', help="Path to the folder with logs", required=True)
parser.add_argument('-s', help="Directory date to start from", default="20000101_000000", required=False)
parser.add_argument('-e', help="Directory date to end on", default="30000101_000000", required=False)
parser.add_argument('-v', help="View the path of the found insect flights in a plot", required=False, default=False, action='store_true')
parser.add_argument('--filename', help="Path and filename to store results in", default="./detections.json")
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

detections = []
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
            detections_in_folder = detection_counts_in_folder(folder)
            if detections_in_folder != []:
                [detections.append(detection) for detection in detections_in_folder]
        elif mode == 'hunt' or mode == 'deploy':
            hunts_in_dir = hunts_in_folder(folder,operational_log_start)
            if hunts_in_dir != []:
                hunts.append(hunts_in_dir)

data_detections = {"from" : args.s,
        "till" : args.e,
        "moth_counts" : len(detections),
        "moths" : detections,
        "hunts" : hunts,
        "mode" : statuss,
        "system" : args.system,
        "version" : version}
with open(args.filename, 'w') as outfile:
    json.dump(data_detections, outfile)
print("Counting complete, saved in {0}".format(args.filename))
