import psutil

process = [proc for proc in psutil.process_iter() if "gz" in proc.name()]

print(process)
# data = process.as_dict(attrs=['status', 'cpu_num', 'num_ctx_switches', 'pid', 'memory_full_info', 'connections', 'create_time', 'ionice', 'num_fds', 'cpu_percent', 'ppid', 'nice', 'cpu_times', 'io_counters', 'memory_info', 'name', 'num_threads', 'memory_percent',])

# for key, el in data.items():
#     print(f"{key}: {el}" )