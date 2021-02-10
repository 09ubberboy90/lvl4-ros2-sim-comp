from matplotlib import pyplot as plt
import os
from os import walk
import re
from collections import defaultdict
import numpy as np
from scipy.interpolate import make_interp_spline, BSpline

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

fig, axs = plt.subplots(2,figsize=(12,7.5) )

procs = defaultdict(lambda: defaultdict(list))

for key,el in types.items():
    for name in el:    
        existing = []
        with open(name) as f:
            for lines in f.readlines():
                line = lines.split(",")
                p = line[0]
                val = line[1:]
                val = [float(x) for x in val]
                counter = 0
                new_p = p
                while new_p in existing:
                    counter += 1
                    new_p = new_p + "_" + str(counter)
                if counter != 0:
                    p = p+"_"+str(counter)
                procs[key][p].append(val)
                existing.append(p)
        
for axs, (type, proc) in zip(axs, procs.items()):
    for name, ls in proc.items():
        length = max(map(len, ls))
        arr=np.array([xi+[np.nan]*(length-len(xi)) for xi in ls])

        mean = np.nanmean(arr, axis=0)
        standard_dev = np.std(arr, axis=0)
        print(name, standard_dev)
        x = np.arange(0, mean.shape[0],1)/10 # because recording every 100 ms
        xnew = np.linspace(x.min(), x.max(), mean.shape[0]*10) 

        spl = make_interp_spline(x, mean, k=3)  # type: BSpline
        mean_smooth = spl(xnew)
        #axs.plot(x, mean, label=name)
        axs.plot(xnew, mean_smooth, label=name)

        axs.fill_between(x, mean-standard_dev, mean+standard_dev, alpha = 0.5, interpolate=True)
        axs.set_xlabel("Time (s)")
        if type == "ram":
            axs.set_ylabel("RAM usage (Mb)")
            axs.set_title("RAM usage against time")
        else:
            axs.set_title("CPU usage against time")
            axs.set_ylabel("CPU Usage (% of one core)")
        axs.legend(bbox_to_anchor=(1,1), loc="upper left")
plt.subplots_adjust(left=0.07, right=0.75, bottom=0.08, top=0.95, hspace=0.26)
plt.savefig(os.path.join(os.path.dirname(__file__),"../data/pick_place.png"))