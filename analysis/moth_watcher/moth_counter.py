#!/usr/bin/python3
import numpy as np
import pandas as pd
pd.options.mode.chained_assignment = None
import os, glob, json
import re, datetime
import argparse, socket
import pickle
from tqdm import tqdm

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
    """Sometimes unexplainable zeros in a flight are found these zeros will be interpolated"""
    vals = np.trim_zeros(vals)
    idx = np.nonzero(vals)
    x = np.arange(len(vals))
    interp = interp1d(x[idx], vals[idx], kind="quadratic")
    y = interp(x)
    spline_model = get_natural_cubic_spline_model(x, y, minval=min(x), maxval=max(x), n_knots=7)
    return spline_model.predict(x)

def get_moth_counts_for_dir(path_of_dir, mindate, max_date):
    #concat all csv files containing dates
    files = natural_sort([fp for fp in glob.glob(os.path.join(path_of_dir, "logging", "log*.csv")) if "itrk0" not in fp])
    try:
        header = files[-1]
        df = pd.read_csv(header, sep=";")
    except:
        print("No header file found") # normally happens when dir got no 'logging' dir
        return []

    for file_ in tqdm(files):
        df2 = pd.read_csv(file_, sep=";", names=df.columns)
        df = df.append(df2)
    #read_insect_file
    ins_path = os.path.join(path_of_dir, "logging", "log_itrk0.csv")
    df_ins = pd.read_csv(ins_path, sep=";")

    #merge time df and insect df
    try:
        comp_df = pd.merge(df, df_ins, on='RS_ID', how='outer')
        processed_df = comp_df[comp_df["foundL_insect"].notna()]
    except Exception as e:
        print("Merge csv error")
        print(e)
        return []
    
    # if df has rows that are above the min/max value, remove them
    processed_df.loc[:, "time_x"] =  pd.to_datetime(processed_df.loc[:, "time_x"], format='%Y/%m/%d %H:%M:%S') # convert string to datetime object 
    processed_df = processed_df[(processed_df.loc[:, "time_x"] >= min_date) & (processed_df.loc[:, "time_x"] <= max_date)] 

    # now calculate the mean and standard deviation of each suspected moth flight
    moth_found = processed_df["foundL_insect"].values
    
    time = processed_df["time_x"].values
    RS_ID = processed_df["RS_ID"].values
    xs = processed_df["posX_insect"].values
    ys = processed_df["posY_insect"].values
    zs = processed_df["posZ_insect"].values

    # x_img = processed_df["imLx_insect"].values
    # y_img = processed_df["imLy_insect"].values

    moth_found[(xs == 0) | (ys == 0) | (zs == 0)] = 0 # if xyz values missing, say moth is not seen
    nums, inds, clas = rle(moth_found) # get all possible flight windows

    # add seperate flights together that are too close together to be two different flights
    prev = -9001
    seq_lens, start_ind = [], []
    for i in range(len(nums)):
        if clas[i] == 1:
            if inds[i] - prev < 10: # if in less than 10 frames the moth is seen again, count as same flight.
                seq_lens[-1] = inds[i] - start_ind[-1] + nums[i]
            else:
                start_ind.append(inds[i])
                seq_lens.append(nums[i])
            prev = nums[i]+ inds[i]

    try:
        elapsed_time = processed_df["elapsed"].values
    except Exception as e:
        print("Error in accessing the elasped time var")
        return []
   
    found_flights = []
    if nums is not None:
        for i in range(len(seq_lens)):
            non_smooth_x_p = xs[start_ind[i]:start_ind[i]+seq_lens[i] - 1] # get the x coords for the moth flight
            non_smooth_y_p = ys[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            non_smooth_z_p = zs[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            if np.min([non_smooth_x_p.shape[0], non_smooth_y_p.shape[0], non_smooth_z_p.shape[0]]) < 50:
                continue
            x_p = interpolate_zeros_and_spline(non_smooth_x_p) # splines the tensor and removes trailing zeros
            y_p = interpolate_zeros_and_spline(non_smooth_y_p)
            z_p = interpolate_zeros_and_spline(non_smooth_z_p)

            t_p = elapsed_time[start_ind[i]:start_ind[i]+seq_lens[i] - 1][:x_p.shape[0]].astype('float64') # get all the elapsed time for this flight - the trailing zeros
            dt = t_p[1:]-t_p[:-1] # get time between elapsed times

            dx = (x_p[1:]-x_p[:-1]) / dt # get the derivative of x coords
            dy = (y_p[1:]-y_p[:-1]) / dt
            dz = (z_p[1:]-z_p[:-1]) / dt

            part_vel = np.sqrt(dx**2+dy**2+dz**2) 
            mean_vel = part_vel.mean()
            std_vel = part_vel.std()

            start = elapsed_time[start_ind[i]]
            end = elapsed_time[start_ind[i]+seq_lens[i] - 1]
            duration = end - start # total duration of the flight


            # # filter on image coords
            # x_img_p = x_img[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            # y_img_p = y_img[start_ind[i]:start_ind[i]+seq_lens[i] - 1]
            # dx_img = (x_img_p[1:]-x_img_p[:-1])
            # dy_img = (y_img_p[1:]-y_img_p[:-1])
            # correct_image_movement = np.abs(dx_img).mean() > 0.15 and np.abs(dy_img).mean() > 0.15

            FP_theshold = np.mean(np.ones(shape=(3,3)) - np.abs(np.corrcoef([x_p, -y_p, z_p]))) >= 0.0001

            # correct_dist_mean = mean_vel > 0.1 and mean_vel < 5 # else FP
            # correct_dist_std = std_vel < 2 # else FP
            correct_duration = duration >= 0.05 and duration < 25 # FP if flight duration is between certain seconds

            # if correct_dist_mean and correct_dist_std and correct_duration:
            if correct_duration and FP_theshold:
                v = np.vstack([dx,dy,dz])
                p1 = np.divide(np.sum(v[:,1:] * v[:,:-1], axis=0), part_vel[1:] * part_vel[:-1])
                p1[p1 > 1] = 1
                p1[p1 < -1] = -1
                turning_angle = np.arccos(p1)
                p = np.vstack([x_p,y_p,z_p])
                
                radius_of_curve = np.divide(np.linalg.norm(p[:,2:] - p[:,:-2]), (2 * np.sin(turning_angle)))
                radial_accelaration = np.divide(np.sum(v[:,1:] * v[:,:-1], axis=0), radius_of_curve)

                flight_time = datetime.datetime.utcfromtimestamp(time[inds[i]].tolist()/1e9).strftime("%m/%d/%Y, %H:%M:%S") #numpy dt to string

                if args.v:
                    
                    print(f"RS_ID: {RS_ID[start_ind[i]]} - {RS_ID[start_ind[i]+seq_lens[i] - 1]}")
                    print(f"Start time: {flight_time}")
                    print(f"Flight length: {duration} s, {x_p.shape}")
                    print(f"Start: {x_p[0], y_p[0], z_p[0]}")
                    print([correct_image_movement, np.abs(dx_img).mean(), np.abs(dx_img).max(), dx_img.std(), np.abs(dy_img).max(), np.abs(dy_img).mean(), dy_img.std()])
                    print()
                    fig = plt.figure()
                    ax = fig.gca(projection='3d')
                    ax.plot(x_p, -y_p, z_p, label='smoothend flight')
                    ax.scatter(non_smooth_x_p, -non_smooth_y_p, non_smooth_z_p, label='raw flight', marker="x")
                    ax.legend()
                    ax.set_xlabel('X axis')
                    ax.set_ylabel('Y axis')
                    ax.set_zlabel('Z axis')
                    plt.show()

                # add frames together
                flight_data = {"time" : flight_time,
                                "duration": duration,
                                "RS_ID" : RS_ID[start_ind[i]],
                                "Vel_mean" : mean_vel, # all mean max and std
                                "Vel_std" : std_vel,
                                "Vel_max" : part_vel.max(),
                                "TA_mean" : turning_angle.mean(),
                                "TA_std" : turning_angle.std(),
                                "TA_max" : turning_angle.max(),
                                "RA_mean" : radial_accelaration.mean(),
                                "RA_std" : radial_accelaration.std(),
                                "RA_max" : radial_accelaration.max()}
                found_flights.append(flight_data)

    return found_flights

parser = argparse.ArgumentParser(description='Script that counts the number of detected moths in a directory, bound by the minimum and maximum date\n\
    example: python3 moth_counter.py -i "{path_directory_of_log_dirs}" -s "20200729_190000" -e "20200729_230000" --send')

parser.add_argument('-i', help="Path to the folder with logs", required=True)
parser.add_argument('-s', help="Directory date to start from", required=True)
parser.add_argument('-e', help="Directory date to end on", required=True)
parser.add_argument('-v', help="View the path of the found flights in a plot", required=False, default=False, action='store_true')
parser.add_argument('--filename', help="Path and filename to store results in", default="./moth.json")
parser.add_argument('--send', help="Send results to main database", default=False, action='store_true')

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
flights = []
for folder in filtered_dirs:
    print(f"Analyzing folder {folder}")
    flights_in_dir = get_moth_counts_for_dir(folder, min_date, max_date)
    if flights_in_dir != []:
        [flights.append(flight) for flight in flights_in_dir]
    print(f"Total found flights increased with {len(flights_in_dir)} to {len(flights)}.")

data = {"from" : args.s,
        "till" : args.e, 
        "counts" : len(flights),
        "flights" : flights,
        "system" : socket.gethostname()}
with open(args.filename, 'w') as outfile:
    json.dump(data, outfile)
print("Counting complete, saved in {0}".format(args.filename))

if args.send: # send data to server to store values
    #TODO: sending data is now quite unsafe, especially because IP is hardcoded, solution should be found for this

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("145.94.54.216", 1243))
        HEADERSIZE = 10
        msg = pickle.dumps(data)
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
        s.send(msg)
        print("Send data to server")
    except Exception as e:
        print("Connection to server failed, could not store data in server database")
        print("The following error causes the error")
        print(e)