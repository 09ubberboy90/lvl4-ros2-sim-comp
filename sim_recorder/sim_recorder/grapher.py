from matplotlib import pyplot as plt
import os
from os import walk
import re
from collections import defaultdict
import numpy as np

f = []
exclude = []
for (dirpath, dirnames, filenames) in walk(os.path.join(os.path.dirname(__file__),".."), topdown=True):
    dirnames[:] = [d for d in dirnames if d not in exclude]
    f.extend([os.path.join(*dirpath.split("/"), s) for s in filenames])
#tmp = [el for el in f if el[-5:] == "ipynb"]
tmp = [el for el in f if el[-3:] == "csv"]
print(tmp)
types = defaultdict(list)
for el in tmp:
    if "ram" in el:
        types["ram"].append(el)
    else:
        types["cpu"].append(el)
print(types)

fig, axs = plt.subplots(2, )

procs = defaultdict(lambda: defaultdict(list))

for key,el in types.items():
    for name in el:    
        with open(name) as f:
            for lines in f.readlines():
                line = lines.split(",")
                p = line[0]
                val = line[1:]
                val = [float(x) for x in val]
                procs[key][p].append(val)
        
for axs, (type, proc) in zip(axs, procs.items()):
    for name, ls in proc.items():
        length = max(map(len, ls))
        arr=np.array([xi+[np.nan]*(length-len(xi)) for xi in ls])


        mean = np.nanmean(arr, axis=0)
        x = np.arange(0, mean.shape[0],1)/10 # because recording every 100 ms
        axs.plot(x, mean, label=name)
        axs.set_xlabel("Time (s)")
        if type == "ram":
            axs.set_ylabel("RAM usage (Mb)")
            axs.set_title("RAM usage against time")
        else:
            axs.set_title("CPU usage against time")
            axs.set_ylabel("CPU Usage (%)")
        axs.legend(bbox_to_anchor=(1,1), loc="upper left")
plt.subplots_adjust(left=0.05, right=0.75)
plt.show()