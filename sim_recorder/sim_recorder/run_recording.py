import subprocess, time, os, sys, psutil
from multiprocessing import Process, Pipe, Queue, Event
import signal
import proc_monitor
import proc_monitor_gui
import threading
import _thread


def kill_proc_tree(data, procs, including_parent=False):
    interrupt_event.set()
    for pid in data:
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            if including_parent:
                parent.kill()
        except:
            print("Unkown Pid : "+ str(pid))
    for proc in procs:
        proc.kill()


def signal_handler(sig, frame):
    print("signal")
    kill_proc_tree(data, procs)
    sys.exit(0)

#signal.signal(signal.SIGINT, signal_handler)

## Reference : https://stackoverflow.com/a/40281422
def interrupt_handler(interrupt_event):
    interrupt_event.wait()
    print("Interupting")
    _thread.interrupt_main()


def run1(w, q):
    os.dup2(w.fileno(), 1)
    proc = subprocess.Popen("exec ros2 launch run_move_group run_move_group.launch.py", shell=True)
    q.put(proc.pid)
    
def run2(w, q):
    os.dup2(w.fileno(), 1)
    proc = subprocess.Popen("exec ros2 launch webots_simple_arm panda_trajectory.launch.py", shell=True)
    q.put(proc.pid)
    
def run3(w, q):
    os.dup2(w.fileno(), 1)
    proc = subprocess.Popen("exec ros2 launch webots_simple_arm moveit_webots.launch.py", shell=True)
    q.put(proc.pid)
    
def run4(q, interrupt_event, gui=False):
    task = threading.Thread(target=interrupt_handler, args=(interrupt_event,))
    task.start()
    try:
            
        if gui:
            proc_monitor_gui.direct_call("webots") # launch recording
        else:
            proc_monitor.main()
    except:
        q.put("Exit")
        
r, w = Pipe()
q = Queue()
reader = os.fdopen(r.fileno(), 'r')
interrupt_event = Event()

p1 = Process(target=run1, args=(w,q))
p2 = Process(target=run2, args=(w,q))
p3 = Process(target=run3, args=(w,q))
p4 = Process(target=run4, args=(q, interrupt_event, True,), daemon=True)

procs= [p1,p2,p3]
data = []
p1.start()
time.sleep(5)
p2.start()
time.sleep(10)
p3.start()
p4.start()

for proc in range(3):
    data.append(q.get())

with open("out.txt", "w") as f:
    while True:
        text = reader.readline()
        f.write(text)
        if "Plan and Execute request complete!" in text or "Goal Succeeded" in text:
            print("Completed")
            kill_proc_tree(data, procs)
            if q.get() == "Exit":
                sys.exit(0)



