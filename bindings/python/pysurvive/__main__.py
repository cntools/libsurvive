import subprocess

import pysurvive
import sys
from gooey import Gooey, GooeyParser

@Gooey(tabbed_groups=True,
       image_dir="images",
       use_cmd_args=True,
       program_name="pysurvive",
       richtext_controls=True,
       clear_before_run=True)
def main():
    parser = pysurvive.create_argument_parser(GooeyParser())

    args = parser.parse_args()

    if args.websocketd:
        subprocess.run(["survive-websocketd"] + sys.argv[1:])
        return

    print(" ".join(sys.argv))
    actx = pysurvive.SimpleContext(sys.argv)

    while actx.Running():
        pass


if __name__ == '__main__':
    main()
