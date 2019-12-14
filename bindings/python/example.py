import pysurvive
import sys

actx = pysurvive.SimpleContext(sys.argv)

for obj in actx.Objects():
    print(obj.Name())

while actx.Running():
    updated = actx.NextUpdated()
    if updated:
        print(updated.Name(), updated.Pose())

