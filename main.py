from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Port, Direction, Color, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from urandom import randint

# ----------------------------------------------------------------------------
# 1. HARDWARE SETUP
# ----------------------------------------------------------------------------
hub = PrimeHub()

left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
try:
    fan_motor = Motor(Port.C)
except OSError:
    fan_motor = None                    # fan not attached -> extinguish skips it

color_sensor = ColorSensor(Port.E)      # facing down
side_sonar = UltrasonicSensor(Port.D)   # facing right
bumper = ForceSensor(Port.F)            # facing front

drive = DriveBase(left_motor, right_motor, wheel_diameter=60, axle_track=115)
try:
    drive.use_gyro(True)
except AttributeError:
    pass

# ----------------------------------------------------------------------------
# 2. TUNING CONSTANTS
# ----------------------------------------------------------------------------
LOOP_MS = 20

# Wander
WANDER_SPEED = 300          # mm/s
WANDER_TURN_RATE = 60      # deg/s; negative = left (biases robot toward walls on its right)
WANDER_TURN_MIN_MS = 1000   # how long to go straight before a turn
WANDER_TURN_MAX_MS = 3000

# Wall follow (wall kept on the RIGHT)
WALL_TARGET_MM = 50        # desired distance to the wall
WALL_DETECT_MM = 100        # start following when the wall is this close
WALL_LOST_MM = 150          # bigger than this = no wall (doorway / outside corner)
WALL_SPEED = 200            # mm/s
WALL_TURN_RATE = 0.3               # degrees of heading offset per mm of distance error
WALL_MAX_TURN = 60          # deg/s
LOST_GIVEUP_MS = 4000       # look for a lost wall this long, then give up
MAX_FOLLOW_MS = 20000       # leave the wall eventually to explore other rooms
FOLLOW_COOLDOWN_MS = 2000   # ignore walls for a while after leaving one

# Fire detection
FIRE_COLOR = Color.BLACK    # paper color -- make sure the plain floor never reads this!
FIRE_CONFIRM = 3            # consecutive readings needed (debounce)
color_sensor.detectable_colors([Color.RED, Color.BLUE, Color.GREEN,
                                Color.YELLOW, Color.WHITE, Color.BLACK, Color.NONE])

# Extinguish
EXTINGUISH_PASSES = 3
FAN_SPEED = 1000            # deg/s (flip sign if the fan blows the wrong way)
SWEEP_ANGLE = 40            # degrees to each side
SWEEP_PASSES = 3
APPROACH_MM = 60            # creep forward after detecting the paper edge

# ----------------------------------------------------------------------------
# 3. SENSOR SNAPSHOT -- read every sensor ONCE per loop
# ----------------------------------------------------------------------------
def read_sensors():
    return {
        "side": side_sonar.distance(),   # mm
        "bump": bumper.pressed(force=1),
        "color": color_sensor.color(),
    }


def alarm():
    hub.light.on(Color.BLUE)
    triumph = [
    "C5/16", "C5/16", "C5/16", "C5/4.", "G#4/4.", "A#4/4.",
    "C5/8", "R/16", "A#4/16", "C5/2"
]
    hub.speaker.play(triumph, tempo=140)


# ----------------------------------------------------------------------------
# 4. BEHAVIORS -- active(s): want control?   step(s): one short action
# ----------------------------------------------------------------------------
class Behavior:
    name = "base"

    def active(self, s):
        return False

    def step(self, s):
        pass


class Wander(Behavior):
    name = "wander"

    def __init__(self):
        self.timer = StopWatch()
        self.next_change = randint(WANDER_TURN_MIN_MS, WANDER_TURN_MAX_MS)
        self.turning_until = 0

    def active(self, s):
        return True     # lowest priority: always true

    def step(self, s):
        t = self.timer.time()
        if t > self.next_change:
            self.turning_until = t + randint(300, 900)
            self.next_change = t + randint(WANDER_TURN_MIN_MS, WANDER_TURN_MAX_MS)
        if t < self.turning_until:
            drive.drive(WANDER_SPEED // 2, randint(-WANDER_TURN_RATE, WANDER_TURN_RATE))
        else:
            drive.drive(WANDER_SPEED, 0)


class WallFollow(Behavior):
    name = "wall_follow"

    def __init__(self):
        self.following = False
        self.follow_timer = StopWatch()
        self.lost_timer = StopWatch()
        self.cooldown = StopWatch()
        self.cooling = False

    def release(self):
        self.following = False
        self.cooling = True
        self.cooldown.reset()

    def active(self, s):
        d = s["side"]
        if not self.following:
            if self.cooling and self.cooldown.time() < FOLLOW_COOLDOWN_MS:
                return False
            self.cooling = False
            if d < WALL_DETECT_MM:
                self.following = True
                self.follow_timer.reset()
                self.lost_timer.reset()
        else:
            if self.follow_timer.time() > MAX_FOLLOW_MS:
                self.release()
            elif d < WALL_LOST_MM:
                self.lost_timer.reset()
            elif self.lost_timer.time() > LOST_GIVEUP_MS:
                self.release()
        return self.following

    def step(self, s):
        d = s["side"]
        if d >= WALL_LOST_MM:
            # Wall disappeared (doorway / outside corner): arc right to wrap around it.
            drive.drive(WALL_SPEED // 2, 90)
            return
        # P-controller. In Pybricks a POSITIVE turn rate turns RIGHT (clockwise).
        error = d - WALL_TARGET_MM          # >0: too far from wall -> turn right
        turn = WALL_TURN_RATE * error
        turn = max(-WALL_MAX_TURN, min(WALL_MAX_TURN, turn))
        drive.drive(WALL_SPEED, turn)


class Escape(Behavior):
    name = "escape"

    def active(self, s):
        return s["bump"]

    def step(self, s):
        # Hit a wall in front. Back up, turn LEFT so the wall ends up on the RIGHT,
        # which hands control naturally to WallFollow.
        drive.stop()
        drive.straight(-60)
        drive.turn(-90)


class FireDetect(Behavior):
    name = "fire_detect"

    def __init__(self, world):
        self.world = world
        self.count = 0

    def active(self, s):
        if self.world["fire_found"]:
            return False
        self.count = self.count + 1 if s["color"] == FIRE_COLOR else 0
        return self.count >= FIRE_CONFIRM

    def step(self, s):
        drive.stop()
        print("FIRE DETECTED")
        alarm()
        self.world["fire_found"] = True


class Extinguish(Behavior):
    name = "extinguish"

    def __init__(self, world):
        self.world = world

    def active(self, s):
        return self.world["fire_found"] and not self.world["done"]

    def step(self, s):
        # We only know we're on the paper edge, not where the candle is,
        # so creep in a little and sweep the fan side to side.
        drive.straight(APPROACH_MM)
        for _ in range(EXTINGUISH_PASSES):
            if fan_motor:
                fan_motor.run(FAN_SPEED)
            for _ in range(SWEEP_PASSES):
                drive.turn(SWEEP_ANGLE)
                drive.turn(-2 * SWEEP_ANGLE)
                drive.turn(SWEEP_ANGLE)
            if fan_motor:
                fan_motor.stop()
        drive.stop()
        self.world["done"] = True
        print("Extinguish attempt finished. Press LEFT button to retry.")


# ----------------------------------------------------------------------------
# 5. ARBITER -- the main loop
# ----------------------------------------------------------------------------
def main():
    world = {"fire_found": False, "done": False}

    behaviors = [               # highest priority FIRST
        Extinguish(world),
        FireDetect(world),
        Escape(),
        WallFollow(),
        Wander(),
    ]

    hub.light.on(Color.GREEN)
    current = None

    while True:
        if world["done"]:
            if Button.CENTER in hub.buttons.pressed():
                world["done"] = False
                hub.light.on(Color.ORANGE)
                wait(500)
            else:
                wait(LOOP_MS)
                continue

        s = read_sensors()
        for b in behaviors:
            if b.active(s):
                if b is not current:
                    print("->", b.name, s)      # log every switch
                    current = b
                b.step(s)
                break
        wait(LOOP_MS)


main()
