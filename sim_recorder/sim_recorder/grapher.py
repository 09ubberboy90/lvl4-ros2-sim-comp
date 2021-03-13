from matplotlib import pyplot as plt
import os, sys
from os import walk
import re
from collections import defaultdict, OrderedDict
import numpy as np
from scipy.interpolate import make_interp_spline, BSpline
from scipy.interpolate import interp1d
from scipy import signal
# import random
from matplotlib import cm
from numpy import linspace

SMOOTH_INDEX = 21
POLY_INDEX = 3
if len(sys.argv) < 2:
    folder = ["data"]
else:
    folder = [sys.argv[1]]
f = []
exclude = ["data", "data_webots_org", "data_webots_throw", "data_webots", "data_gazebo", "data_gazebo_throw"]
exclude = [el for el in exclude if el not in folder]
# exclude = ["data", "data_webots"]
for (dirpath, dirnames, filenames) in walk(os.path.join(os.path.dirname(__file__),".."), topdown=True):
    dirnames[:] = [d for d in dirnames if d not in exclude]
    f.extend([os.path.join(*dirpath.split("/"), s) for s in filenames])
#tmp = [el for el in f if el[-5:] == "ipynb"]
tmp = [el for el in f if el[-3:] == "csv"]
print(f"Found {len(tmp)}")
types = defaultdict(list)
for el in tmp:
    if "ram" in el:
        types["ram"].append(el)
    else:
        types["cpu"].append(el)

fig, axs = plt.subplots(2,figsize=(12,7.5) )

procs = defaultdict(lambda: defaultdict(list))

for key,el in types.items():
    for name in el:    
        existing = []
        with open(name) as f:
            for lines in f.readlines():
                line = lines.split(",")
                p = line[0]
                if p == "ruby":
                    p = "ignition"
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
# colors = {}
# colors.update(mcolors.TABLEAU_COLORS)
# colors.update(mcolors.BASE_COLORS)
# colors.update(mcolors.CSS4_COLORS)
# colors = list(colors.values())
# random.shuffle(colors)


for axs, (type, proc) in zip(axs, procs.items()):
    cm_subsection = linspace(0.0, 1.0, len(proc.values())) 
    colors = [ cm.jet(x) for x in cm_subsection ]
    sorted_dict = OrderedDict()

    keys = sorted(proc.keys())
    for key in keys:
        sorted_dict[key] = proc[key]
    #colors.reverse()
    for color, (name, ls) in zip(colors[::1], sorted_dict.items()):
        length = max(map(len, ls))
        arr=np.array([xi+[np.nan]*(length-len(xi)) for xi in ls])

        mean = np.nanmean(arr, axis=0)
        standard_dev = np.nanstd(arr, axis=0)
        x = np.arange(0, mean.shape[0],1)/10 # because recording every 100 ms
        xnew = np.linspace(x.min(), x.max(), mean.shape[0]*1000) 
        # f2 = interp1d(x, mean, kind='cubic')
        # spl = make_interp_spline(x, mean, k=3)  # type: BSpline
        # mean_smooth = spl(xnew)
        # axs.plot(x, mean, label=name+"_org",color=color)

        y=signal.savgol_filter(mean,
                           SMOOTH_INDEX, # window size used for filtering
                           POLY_INDEX), # order of fitted polynomial
        axs.plot(x, y[0], label=name, color=color)
        # axs.plot(xnew, mean_smooth, label=name, color=color)

        lower = mean-standard_dev
        high = mean+standard_dev
        lower=signal.savgol_filter(lower,
                           SMOOTH_INDEX, # window size used for filtering
                           POLY_INDEX), # order of fitted polynomial
        high=signal.savgol_filter(high,
                           SMOOTH_INDEX, # window size used for filtering
                           POLY_INDEX), # order of fitted polynomial
        
        lower[0][lower[0] < 0] = 0
        high[0][high[0] < 0] = 0
        
        for i in range(10):
            if high[0][i] > 300 and type == "cpu":
                high[0][i] = y[0][i]
        axs.fill_between(x, lower[0], high[0], alpha = 0.5, interpolate=False,color=color)
        axs.set_xlabel("Time (s)")
        if type == "ram":
            axs.set_ylabel("RAM usage (Mb)")
            axs.set_title("RAM usage against time")
        else:
            axs.set_title("CPU usage against time")
            axs.set_ylabel("CPU Usage (% of one core)")
    axs.legend( bbox_to_anchor=(1,1.1), loc="upper left")
plt.subplots_adjust(left=0.07, right=0.75, bottom=0.08, top=0.95, hspace=0.26)
plt.savefig(os.path.join(os.path.dirname(__file__),f"../{folder[0]}/pick_place_smooth.svg"))

fig, axs = plt.subplots(2,figsize=(12,7.5) )

for axs, (type, proc) in zip(axs, procs.items()):
    cm_subsection = linspace(0.0, 1.0, len(proc.values())) 
    colors = [ cm.jet(x) for x in cm_subsection ]
    sorted_dict = OrderedDict()

    keys = sorted(proc.keys())
    for key in keys:
        sorted_dict[key] = proc[key]
    #colors.reverse()
    for color, (name, ls) in zip(colors[::1], sorted_dict.items()):
        length = max(map(len, ls))
        arr=np.array([xi+[np.nan]*(length-len(xi)) for xi in ls])

        mean = np.nanmean(arr, axis=0)
        standard_dev = np.nanstd(arr, axis=0)
        x = np.arange(0, mean.shape[0],1)/10 # because recording every 100 ms
        xnew = np.linspace(x.min(), x.max(), mean.shape[0]*1000) 
        # f2 = interp1d(x, mean, kind='cubic')
        # spl = make_interp_spline(x, mean, k=3)  # type: BSpline
        # mean_smooth = spl(xnew)
        # axs.plot(x, mean, label=name+"_org",color=color)

        axs.plot(x, mean, label=name, color=color)
        # axs.plot(xnew, mean_smooth, label=name, color=color)

        lower = mean-standard_dev
        high = mean+standard_dev
        lower[lower < 0] = 0
        high[high < 0] = 0
        for i in range(10):
            if high[i] > 400 and type == "cpu":
                high[i] = mean[i]

        axs.fill_between(x, lower, high, alpha = 0.5, interpolate=False,color=color)
        axs.set_xlabel("Time (s)")
        if type == "ram":
            axs.set_ylabel("RAM usage (Mb)")
            axs.set_title("RAM usage against time")
        else:
            axs.set_title("CPU usage against time")
            axs.set_ylabel("CPU Usage (% of one core)")
    axs.legend( bbox_to_anchor=(1,1.1), loc="upper left")
plt.subplots_adjust(left=0.07, right=0.75, bottom=0.08, top=0.95, hspace=0.26)
plt.savefig(os.path.join(os.path.dirname(__file__),f"../{folder[0]}/pick_place_no_smooth.svg"))
