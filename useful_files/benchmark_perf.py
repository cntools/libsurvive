import sys
import subprocess
from time import process_time, time
import json
import git
import hashlib
import os
import resource
from matplotlib import pyplot as plt
import mplcursors

repo = git.Repo('.', search_parent_directories=True)

os.makedirs(repo.git_dir + "/benchmarks", exist_ok=True)

def checkprocess_rtn(proc):
    if proc.returncode != 0:
        print(proc.stdout)
        print(proc.stderr)
        raise Exception("Error running process", proc.args)

def checkout_and_build(commit, cmd):
    print(f"Checkout...   {commit}")
    checkprocess_rtn(subprocess.run(['git', 'checkout', '-f', commit, '--recurse-submodules=off'], capture_output=True))
    print(f"Update...     {commit}")
    checkprocess_rtn(subprocess.run(['git', 'submodule', 'update', '--init', '--recursive'], capture_output=True))

    tree = repo.tree()

    cmd_digest = hashlib.sha1(cmd.encode()).hexdigest()
    key = f"{tree.hexsha}_{cmd_digest}"
    cache_file = f"{repo.git_dir}/benchmarks/{key}.json"

    try:
        with open(cache_file, 'r') as f:
            print(f"Using cache for {commit} ({key})")
            return json.load(f)
    except:
        pass

    print(f"Building...   {commit}")
    rtn = subprocess.run(["make", "-j4"], capture_output=True).returncode
    if rtn != 0:
        print(f"Skipping {commit}")
        with open(cache_file, 'w') as f:
            json.dump(None, f)

        return None
    print(f"Running...    {commit}/{tree}")

    usage_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    proc = subprocess.run(cmd.split(' '), capture_output=True)
    usage_end = resource.getrusage(resource.RUSAGE_CHILDREN)
    
    user_time = usage_end.ru_utime - usage_start.ru_utime

    data = {"time": user_time, "tree_id": tree.hexsha}
    with open(cache_file, 'w') as f:
        json.dump(data, f)
    
    return data
    

if __name__ == '__main__':
    cmd = "./survive-cli --playback ./src/test_cases/libsurvive-extras-data/tests/drone.rec.gz --playback-factor 0 --no-threaded-posers --v 100"
    #if len(sys.argv) > 1:
    #    cmd = sys.argv[1]
    hashs = []
    times = []
    msgs = []
    for commit in reversed(sys.argv[1:]):
        d = None
        try:
            d = checkout_and_build(commit, cmd)
        except Exception:
            print("Skipping", commit)
        if d is None:
            continue
        
        print(d)
        hashs.append(commit[0:8])
        times.append(d["time"])
        msgs.append(repo.commit(commit).message)
    
    dataplot = plt.plot(hashs, times, "-o")
    plt.ylim(bottom=0)
    plt.xticks(rotation=90)
    plt.xlabel("Commits")
    plt.ylabel("Seconds")
    plt.title("Time Benchmarks")

    cursor = mplcursors.cursor(dataplot, multiple=True)

    cursor.connect("add", lambda sel: sel.annotation.set_text(msgs[int(round(sel.index))]))
    
    plt.show()

    #git rev-list --ancestry-path 7b4a07a..ecf5891

    #subprocess.run(["ls", "-l"])
