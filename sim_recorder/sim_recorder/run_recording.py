import subprocess, time, os, sys, psutil
from multiprocessing import Process, Pipe, Queue
import signal
import proc_monitor 
def kill_proc_tree(data, including_parent=False):    
    for pid in data:
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            if including_parent:
                parent.kill()
        except:
            print("Unkown Pid : "+ str(pid))


def signal_handler(sig, frame):
    kill_proc_tree(data)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


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
    proc_monitor.main() # launch recording
    q.put(proc.pid)
    

r, w = Pipe()
q = Queue()
reader = os.fdopen(r.fileno(), 'r')
p1 = Process(target=run1, args=(w,q))
p2 = Process(target=run2, args=(w,q))
p3 = Process(target=run3, args=(w,q))

#data= [p1,p2,p3]
data = []
p1.start()
time.sleep(5)
p2.start()
time.sleep(5)
p3.start()

for proc in range(3):
    data.append(q.get())

with open("out.txt", "w") as f:
    while True:
        text = reader.readline()
        f.write(text)
        if "Plan and Execute request complete!" in text or "Goal Succeeded" in text:
            print("Completed")
            kill_proc_tree(data)
            sys.exit(0)



