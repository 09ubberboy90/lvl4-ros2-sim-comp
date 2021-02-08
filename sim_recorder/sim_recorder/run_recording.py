import subprocess
import time
import os
import sys
import psutil
from multiprocessing import Process, Pipe, Queue, Event
import signal

try:
    import proc_monitor
    import proc_monitor_gui
except ModuleNotFoundError:
    from . import proc_monitor
    from . import proc_monitor_gui

import threading
import _thread


def kill_proc_tree(pids, procs,interrupt_event, including_parent=False):
    interrupt_event.set()
    for pid in pids:
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            if including_parent:
                parent.kill()
        except:
            print("Unkown Pid : " + str(pid))
    for proc in procs[:-1]:
        proc.kill()

# Reference : https://stackoverflow.com/a/40281422
def interrupt_handler(interrupt_event):
    interrupt_event.wait()
    print("Interupting")
    _thread.interrupt_main()


def run_com(w, q, com):
    os.dup2(w.fileno(), 1)
    proc = subprocess.Popen("exec " + com, shell=True)
    q.put(proc.pid)


def run_recorder(q, interrupt_event, simulator, gui=False):
    task = threading.Thread(target=interrupt_handler, args=(interrupt_event,))
    task.start()
    simulator = "webots" if simulator == "webots" else "gazebo"
    try:
        if gui:
            proc_monitor_gui.run(simulator)  # launch recording
        else:
            proc_monitor.run(simulator)
    except:
        q.put("Exit")


def generate_procs(simulator, commands, r, w, q, interrupt_event):
    procs = []
    for com in commands:
        procs.append(Process(target=run_com, args=(w, q, com)))
    procs.append(Process(target=run_recorder, args=(
        q, interrupt_event, simulator), daemon=True))
    return procs


def start_proces(delay, procs, q):
    pids = []
    delay.append(0)  # Otherwise out of range
    for idx, p in enumerate(procs):
        p.start()
        time.sleep(delay[idx])

    for proc in range(len(procs)-1):
        pids.append(q.get())

    return pids


class Webots():
    def __init__(self):
        self.name = "webots"
        self.commands = [
            "ros2 launch webots_simple_arm pick_place.launch.py",
            "ros2 launch webots_simple_arm moveit_webots.launch.py",
        ]
        self.delays = [10]
class Gazebo():
    def __init__(self):
        self.name = "gazebo"
        self.commands = [
            "ros2 launch run_move_group run_move_group.launch.py",
            "ros2 launch webots_simple_arm panda_trajectory.launch.py",
            "ros2 launch webots_simple_arm moveit_webots.launch.py",
        ]
        self.delays = [5, 10, 0]


def main(args=None):
    if len(sys.argv) == 1 or sys.argv[1] == "webots":
        sim = Webots()
    else:
        sim = Gazebo()

    r, w = Pipe()
    q = Queue()
    reader = os.fdopen(r.fileno(), 'r')
    interrupt_event = Event()
    procs = generate_procs(sim.name, sim.commands, r, w, q, interrupt_event)
    pids = start_proces(sim.delays, procs, q)
    signal.signal(signal.SIGALRM, lambda x: print("Timeout"))
    signal.alarm(60)
    with open("/home/ubb/Documents/PersonalProject/VrController/sim_recorder/data/out.txt", "w") as f:
        while True:
            text = reader.readline()
            f.write(text)
            if "Task completed Succesfully" in text:
                print("Completed")
                signal.alarm(0)
                kill_proc_tree(pids, procs, interrupt_event)
                if q.get() == "Exit":
                    sys.exit(0)

if __name__ == "__main__":
    main()
