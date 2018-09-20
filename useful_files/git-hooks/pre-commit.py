import subprocess

try:
    run_args = ["git", "clang-format", "--"]

    new_files = subprocess.check_output(["git", "diff", "--cached", "--name-only", "--diff-filter=ACM"]).strip().split("\n")

    run_args = run_args + new_files
    print("Running clang-format...")

    output = subprocess.check_output(run_args)
    print output
    changed_list = output.split('\n')
    if changed_list[0] == 'changed files:':
        changed_list.pop(0)
        for changed in changed_list:
            if len(changed.strip()) > 0:
                add_output = subprocess.check_output(['git', 'add', changed.strip()])
                if len(add_output) > 0:
                    print(add_output)
    exit(0)
except subprocess.CalledProcessError as e:
    print(e.output)
    exit(1)
except Exception as e:
    print "Clang format not installed; please install: ", e
    exit(0)
