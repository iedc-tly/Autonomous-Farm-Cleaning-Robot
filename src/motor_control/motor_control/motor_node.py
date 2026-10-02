import time
import threading
import pigpio
from typing import Tuple

# ---- Pin map ----------------------------------------------------------
LF_PWM, LF_DIR = 18, 23
RF_PWM, RF_DIR = 19, 24
LR_PWM, LR_DIR = 12, 5
RR_PWM, RR_DIR = 13, 6

LEFT_GROUP  = ((LF_PWM, LF_DIR), (LR_PWM, LR_DIR))
RIGHT_GROUP = ((RF_PWM, RF_DIR), (RR_PWM, RR_DIR))
PWM_PINS = [LF_PWM, RF_PWM, LR_PWM, RR_PWM]
DIR_PINS = [LF_DIR, RF_DIR, LR_DIR, RR_DIR]

PWM_FREQ = 20000
RANGE = 10000

# ---- Ramp / safety config ----------------------------------------------
RAMP_STEP   = 0.10   # max change in speed (0-1 scale) per tick — higher = more responsive
RAMP_HZ     = 50      # ticks per second
RAMP_DT     = 1.0 / RAMP_HZ

# ---- Setup ---------------------------------------------------------------
pi = pigpio.pi()
if not pi.connected:
    raise SystemExit("pigpio daemon not running. Start: sudo systemctl start pigpiod")

for p in PWM_PINS:
    pi.set_mode(p, pigpio.OUTPUT)
    pi.set_PWM_frequency(p, PWM_FREQ)
    pi.set_PWM_range(p, RANGE)
for p in DIR_PINS:
    pi.set_mode(p, pigpio.OUTPUT)

# current + target speed per side, [-1, 1]
_lock = threading.Lock()
_current = {"left": 0.0, "right": 0.0}
_target  = {"left": 0.0, "right": 0.0}
_running = True
_single_side_turn = False  # False = Tank Turn (symmetric dual-side zero-radius spin), True = Single-Side Pivot

MIN_DUTY = 1500  # 15% minimum PWM threshold to overcome static gearbox friction

def set_single_side_turn(enabled: bool):
    global _single_side_turn
    _single_side_turn = bool(enabled)

def get_single_side_turn() -> bool:
    return _single_side_turn

def _apply_motor(pwm_pin: int, dir_pin: int, speed: float):
    s = max(-1.0, min(1.0, speed))
    forward = s >= 0.0
    if abs(s) < 0.01:
        duty = 0
    else:
        duty = int(MIN_DUTY + abs(s) * (RANGE - MIN_DUTY))
    pi.write(dir_pin, 1 if forward else 0)
    pi.set_PWM_dutycycle(pwm_pin, min(RANGE, duty))

def _apply_group(group: Tuple[Tuple[int, int], Tuple[int, int]], speed: float):
    for pwm, d in group:
        _apply_motor(pwm, d, speed)

def _step_toward(current: float, target: float, step: float) -> float:
    if abs(target - current) <= step:
        return target
    return current + (step if target > current else -step)

def _ramp_loop():
    """Background thread: gently moves current speed toward target speed each tick."""
    while _running:
        with _lock:
            for side in ("left", "right"):
                _current[side] = _step_toward(_current[side], _target[side], RAMP_STEP)
            l, r = _current["left"], _current["right"]
        _apply_group(LEFT_GROUP, l)
        _apply_group(RIGHT_GROUP, r)
        time.sleep(RAMP_DT)

_ramp_thread = threading.Thread(target=_ramp_loop, daemon=True)
_ramp_thread.start()

def _set_targets(left: float, right: float):
    with _lock:
        _target["left"]  = max(-1.0, min(1.0, left))
        _target["right"] = max(-1.0, min(1.0, right))

def stop_all(immediate: bool = False):
    """immediate=True bypasses ramp — use only for emergency stop."""
    _set_targets(0.0, 0.0)
    if immediate:
        with _lock:
            _current["left"] = 0.0
            _current["right"] = 0.0
        for p in PWM_PINS:
            pi.set_PWM_dutycycle(p, 0)

# ---- Drive API (all motions go through ramp) -----------------------------
def drive(throttle: float, steer: float, single_side_turn: bool = None):
    t = max(-1.0, min(1.0, float(throttle)))
    s = max(-1.0, min(1.0, float(steer)))

    use_single_side = _single_side_turn if single_side_turn is None else single_side_turn

    if use_single_side and abs(s) > 0.001:
        # Single-side motor turning:
        # Instead of spinning left and right sides in opposite directions (a one-point turn),
        # the inner side is held stationary as a pivot point and only the outer side motors drive.
        # This provides high precision without tire scrub or odometry slip on farm terrain.
        if abs(t) < 0.05:
            # In-place pivot turn using ONLY one side motors:
            # s > 0 (Turn Left): Left side = 0.0 (anchor), Right side = forward (+|s|)
            # s < 0 (Turn Right): Right side = 0.0 (anchor), Left side = forward (+|s|)
            if s > 0.0:
                left_speed = 0.0
                right_speed = abs(s)
            else:
                left_speed = abs(s)
                right_speed = 0.0
        elif t > 0.0:
            # Moving forward while turning: ensure inner wheel never reverses
            if s > 0.0:  # Turn Left
                left_speed = max(0.0, t - s)
                right_speed = t + s
            else:        # Turn Right
                left_speed = t + abs(s)
                right_speed = max(0.0, t - abs(s))
        else:
            # Moving backward while turning: ensure inner wheel never goes forward
            if s > 0.0:  # Turn Left in reverse
                left_speed = min(0.0, t + s)
                right_speed = t - s
            else:        # Turn Right in reverse
                left_speed = t - abs(s)
                right_speed = min(0.0, t + abs(s))
    else:
        # Standard symmetric differential drive (one-point turn)
        left_speed  = t - s
        right_speed = t + s

    m = max(1.0, abs(left_speed), abs(right_speed))
    _set_targets(left_speed / m, right_speed / m)

def forward(speed: float):
    drive(max(0.0, min(1.0, speed)), 0.0)

def backward(speed: float):
    drive(-max(0.0, min(1.0, speed)), 0.0)

def turn_left(speed: float, arc: float = 0.6):
    sp = max(0.0, min(1.0, speed))
    drive(sp * (1.0 - arc), sp * arc)

def turn_right(speed: float, arc: float = 0.6):
    sp = max(0.0, min(1.0, speed))
    drive(sp * (1.0 - arc), -sp * arc)

def pivot_left(speed: float):
    """Tank turn left: left wheels reverse, right wheels forward."""
    sp = max(0.0, min(1.0, speed))
    drive(0.0, sp, single_side_turn=False)

def pivot_right(speed: float):
    """Tank turn right: left wheels forward, right wheels reverse."""
    sp = max(0.0, min(1.0, speed))
    drive(0.0, -sp, single_side_turn=False)

def shutdown():
    global _running
    _running = False
    _ramp_thread.join(timeout=1.0)
    stop_all(immediate=True)
    pi.stop()
