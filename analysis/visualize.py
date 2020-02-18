#%%
import processloglib as lib
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

source_folder = "/home/houjebek/data_check/checked/"
source_folder = os.path.expanduser(source_folder.strip(" "))

fps = 30
n0 = 0
n1 = 3 * fps #take only the first 3 seconds, because that makes the human classification a  lot faster

data,csv_data,csv_col_names = lib.get_dataset(source_folder,n0,n1)
ground_truth = data[:,0].astype(int)
padded_data = []
for entry in csv_data:
    entry = lib.append_zeros_to_log(entry,n1-n0)
    entry = entry.flatten()
    padded_data.append(entry)

padded_data = np.asarray(padded_data, dtype=np.float32)

df = pd.DataFrame(padded_data)

#%%
import umap
import seaborn as sns
reducer = umap.UMAP()
embedding = reducer.fit_transform(df)

plt.scatter(embedding[:, 0], embedding[:, 1], c=[sns.color_palette()[x] for x in ground_truth])
plt.gca().set_aspect('equal', 'datalim')
plt.title('Moth UMAP projection', fontsize=24)
plt.show()

