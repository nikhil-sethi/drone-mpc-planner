import torch
import numpy as np
from data_preprocess import clean_data_to_df

from conv1d import CNN


def use_the_model(model, data, amount_classes=2, restrict_var=0):
    '''
    This functions predicts the given flightpaths. It uses the same functions
    as the training, but it does not use the gound truth labels file. It outputs
    the file_name and prediction
    '''

    if model == "cnn":
        m_state_dict = torch.load('ai/models/2212_size_feature5.pt')
        model = CNN(amount_classes, 7)
        model.load_state_dict(m_state_dict)

    model.eval()
    list = []

    if restrict_var == 0:
        data = process_one_file_for_flightpath_prediction(data)
        for i in data:
            output, _ = model(i.float())
            if output.size(1) == 1:
                list.append(output.item())
            else:
                list.append(output.tolist())
        return list


def get_binary_class(df_list):
    if df_list:
        return round(sum(df_list) / len(df_list))
    return -1


def get_votings(df_list):
    if df_list:
        return max(df_list, key=df_list.count)
    else:
        return -1


def process_one_file_for_flightpath_prediction(log, sequence_size=90):
    """
    input: log file
    output: list of dataframes with the size of sequence_size with only the keep_list columns
    """
    df_list = []

    clean_df = clean_data_to_df(log, size_included=True)
    size_file = len(clean_df.columns)

    # only estimates flightpaths, shorter than 10 sec
    if size_file < 900:
        while len(clean_df.columns) > sequence_size:
            # determine the df with sequence size as size and remove this df from the to be processed df
            next_df = clean_df[clean_df.columns[0:sequence_size]]
            next_df = next_df.T
            clean_df = clean_df.iloc[:, sequence_size:]
            numpy_flightpath = next_df.to_numpy()
            numpy_flightpath = np.expand_dims(next_df, axis=0)
            torch_flightpath = torch.from_numpy(numpy_flightpath)
            df_list.append(torch_flightpath)
    return df_list
