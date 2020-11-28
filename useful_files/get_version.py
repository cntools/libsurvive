import subprocess

version = subprocess.check_output(["git", "describe", "--tags", "--long"]).strip().decode('utf-8').replace("-", ".")[1:]
version = version[:version.rfind("g")-1]

print(version)