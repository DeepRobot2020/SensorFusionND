import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

csv_file = 'camera_ttc.csv'


df = pd.read_csv(csv_file, header=None)
num_combinations = df.shape[0]

X =  np.arange(0, 18, dtype=np.int)
names = df[0].to_numpy()
for i in range(num_combinations):
    name = names[i]
    ttc = df.values[i][1:].astype(np.float)
    plt.plot(X, ttc, label=name)
    
plt.ylim(0, 20)
plt.xlabel('Frame ID')
# Set the y axis label of the current axis.
plt.ylabel('TTC Camera (s)')
plt.title('TTC Camera detector-descriptor Comparsion')
plt.xticks(X)    
# plt.legend()
plt.legend(loc="lower center", bbox_to_anchor=(0.5, -0.15), ncol= 5)
plt.show()

import pdb; pdb.set_trace()
