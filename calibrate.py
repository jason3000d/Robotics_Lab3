# calibrate.py -- run this FIRST on the real robot, in the real maze.
# It prints raw sensor values so you can pick thresholds for main.py.
# Open the Pybricks web editor console to see the output.
#
# Things to write down:
#   * Ultrasonic distance when the robot sits ~10 cm from a wall, and with no wall nearby
#   * Color sensor reading on: bare floor tile, each sample paper, tape lines (if any)
#   * Force sensor value when the bumper touches a wall gently

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Port
from pybricks.tools import wait

hub = PrimeHub()

# --- change these to match how YOU wired the robot ---
color_sensor = ColorSensor(Port.E)      # pointing DOWN, ~1 cm above floor
side_sonar = UltrasonicSensor(Port.D)   # pointing to the RIGHT side
bumper = ForceSensor(Port.F)            # front bumper

while True:
    h, s, v = color_sensor.hsv()        # hue 0-359, saturation 0-100, value 0-100
    print(
        "color:", color_sensor.color(),
        " refl:", color_sensor.reflection(),
        " hsv:", h, s, v,
        " | side mm:", side_sonar.distance(),      # 2000 = nothing detected
        " | bump:", bumper.pressed(), bumper.force(),
    )
    wait(200)
