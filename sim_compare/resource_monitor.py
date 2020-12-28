# Copyright 2019-2020 Florent AUDONNET, Michal BROOS, Bruce KERR, Ewan PANDELUS, Ruize SHEN

# This file is part of FPD-Explorer.

# FPD-Explorer is free software: you can redistribute it and / or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# FPD-Explorer is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY
# without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import pprint
from collections import defaultdict
import numpy as np
import psutil
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import matplotlib.colors as mcolors

class CpuFreqGraph(FigureCanvas, FuncAnimation):
    def __init__(self, parent=None):
        self._fig = Figure()
        self.ax1 = self._fig.add_subplot(211)
        self.procs = []
        self.ax2 = self._fig.add_subplot(212)
        FigureCanvas.__init__(self, self._fig)
        FuncAnimation.__init__(self, self._fig, self.animate, interval=100, blit=False)
        self.setParent(parent)
        # Needed to initialize the capture
        psutil.cpu_times_percent(interval=None)
        self.x_data = np.arange(0, 100)
        self.cpu_data = np.zeros((0,100))
        self.ram_data = np.zeros((0,100))
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)

        for ax in [self.ax1, self.ax2]:
            #ax.set_xlim(0, 100)
            ax.set_xlabel("Time")
            #ax.set_ylim(0, 100)
            ax.get_xaxis().set_ticks([])
            ax.spines['right'].set_color(None)
            ax.spines['top'].set_color(None)
        self.ax1.set_ylabel("Usage (%)")
        self.ax2.set_ylabel("Usage (MB)")

        self.ax1.set_title("CPU Usage")
        self.ax2.set_title("Memory (RAM) Usage")
        
    def animate(self, i):

        self.cpu_data = np.roll(self.cpu_data, -1, axis=1)
        self.ram_data = np.roll(self.ram_data, -1, axis=1)
        labels = []
        for idx, p in enumerate(self.procs):
            with p.oneshot():
                cpu_usage = p.cpu_percent()
                self.cpu_data[idx+1][-1]= cpu_usage
                ram_usage = p.memory_info().rss / (1024*1024)
                self.ram_data[idx+1][-1]= ram_usage
                labels.append(p.name())
                self.cpu_dict[p.name()].append(cpu_usage)
                self.ram_dict[p.name()].append(ram_usage)
        cpu_stack = np.cumsum(self.cpu_data, axis=0)
        ram_stack = np.cumsum(self.ram_data, axis=0)
        self.ax1.collections.clear()
        self.ax2.collections.clear()
        for idx, (p, color) in enumerate(zip(self.procs, mcolors.TABLEAU_COLORS)):
            self.ax1.fill_between(self.x_data, cpu_stack[idx,:], cpu_stack[idx+1,:], color = color, label=labels[idx])
            self.ax2.fill_between(self.x_data, ram_stack[idx,:], ram_stack[idx+1,:], color = color, label=labels[idx])

        if self.procs:
            self.ax1.legend(loc='upper left')
            self.ax2.legend(loc='upper left')


    def update_proc(self, selected_proc, text_widget):
        # TODO: Figure out what to do if a process disapear
        text = ""
        for proc in selected_proc:
            try:
                data = proc.as_dict(attrs=['num_ctx_switches', 'cpu_times', 'name', 'num_threads'])  
                data["connections"] = len(proc.connections("inet"))
                name = data.pop("name")
                text += f"{name} : \n {pprint.pformat(data, indent=4).replace('{', '').replace('}', '')}\n"
            except psutil.AccessDenied:
                pass
        text_widget.setPlainText(text)
        self.procs = selected_proc

        # Reset arrays
        self.cpu_data = np.zeros((len(self.procs)+1,100))
        self.ram_data = np.zeros((len(self.procs)+1,100))
        self.cpu_dict = defaultdict(list)
        self.ram_dict = defaultdict(list)

    
    def dump_values(self):
        print("Dumping")
        with open("/home/ubb/Documents/PersonalProject/VrController/sim_compare/data/cpu_out.csv", "w") as f:
            for key, el in self.cpu_dict.items():
                f.write(f"{key},{','.join(str(v) for v in el)}\n")
        with open("/home/ubb/Documents/PersonalProject/VrController/sim_compare/data/ram_out.csv", "w") as f:
            for key, el in self.ram_dict.items():
                f.write(f"{key},{','.join(str(v) for v in el)}\n")
