import numpy as np


def clean_data_to_df(df, size_included=False):
    '''This function cleans the given file and returns a cleaned dataframe, conatining only the valuable interpolated columns'''

    # The following lines remove redundant columns
    keep_list = ['sposX_insect', 'sposY_insect', 'sposZ_insect', 'svelX_insect', 'svelY_insect', 'svelZ_insect']

    if size_included:
        keep_list = ['sposX_insect', 'sposY_insect', 'sposZ_insect', 'svelX_insect', 'svelY_insect', 'svelZ_insect', 'size_insect']  # , 'motion_sum_insect']

    new_df = df[keep_list]
    new_df = new_df.iloc[:-11]  # removes the last 11 rows (lost the insect)

    # replace zeros with NaN, then interpolate and extrapolate
    new_df[keep_list] = new_df[keep_list].replace({0: np.nan})
    if size_included:
        new_df.loc[:0, 'svelX_insect':'size_insect'] = 0
    else:
        new_df.loc[:0, 'svelX_insect':'svelZ_insect'] = 0
    new_df = new_df.interpolate(method='linear', axis=0)

    new_df = new_df.T
    return new_df
