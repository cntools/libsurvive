# -*- python -*-

import common_math
import gen1
import gen2
from codegen import *

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--aux":
        print("#include \"survive_reproject.generated.h\"")

        for f in common_math.generate:
            generate_ccode(f)
            generate_jacobians(f)
    else:
        print("#pragma once")
        print("#include \"common.h\"")
        for mode in [True, False]:
            common_math.axis_angle_mode = mode
            suffix = "axis_angle" if common_math.axis_angle_mode else None
            for f in gen2.generate + gen1.generate:
                generate_ccode(f, suffix=suffix)
                generate_jacobians(f, suffix=suffix)

