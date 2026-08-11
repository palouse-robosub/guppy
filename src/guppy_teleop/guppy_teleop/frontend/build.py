import sys, subprocess
from importlib.resources import files

ASSETS_PATH = files("guppy_teleop.frontend").joinpath("assets.qrc")
ASSETS_OUTPUT = files("guppy_teleop.frontend").joinpath("rc_assets.py")

# builds assets
def rcc_main():
    #result = subprocess.run(["pyside6-rcc", "--output", ASSETS_PATH, "-o", ASSETS_OUTPUT])
    result = subprocess.run(["rcc", "-g", "python", ASSETS_PATH, "-o", ASSETS_OUTPUT])

    print(f"resource build exited with code {result.returncode}")

    if (x := result.returncode) != 0:
         sys.exit(x)

def run():
    if sys.argv[0].endswith('.exe'):
        sys.argv[0] = sys.argv[0][:-4]

    rcc_main()

if __name__ == '__main__':
    run()
