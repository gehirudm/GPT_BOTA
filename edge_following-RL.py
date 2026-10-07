#!/usr/bin/env python3

# Import the necessary ev3dev2 classes
import argparse
from collections import deque
import json
import math
import os
import random
from time import sleep, monotonic

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_2, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, InfraredSensor
from ev3dev2.button import Button

# ==========================================
# 1. Hardware Initialization
# ==========================================

# Hardware is initialized after command-line arguments have been checked.
left_motor = right_motor = None
def initialize_hardware():
    global left_motor, right_motor, color_sensor, ir_sensor, btn
    # Preserve the port assignments that work on this physical robot.
    left_motor = LargeMotor(OUTPUT_D)
    right_motor = LargeMotor(OUTPUT_A)
    color_sensor = ColorSensor(INPUT_2)
    ir_sensor = InfraredSensor(INPUT_4)
    btn = Button()
    color_sensor.mode = 'COL-REFLECT'

# Set default motor speeds for smooth control and sharp corner turns
BASE_SPEED = 22 
Q_FILE = "edge_q_table.json"

# ==========================================
# 2. Sensor Reading Functions (For States)
# ==========================================

def categorize_reflection(reflection):
    """
    Reads the color sensor and categorizes it into a discrete state for Edge-Following Q-learning
    (White Line on Black Background):
    - State 0: Outside Line / Lost on Black Mat (Reflected light < 8)
    - State 1: On Edge of White Line / Target Zone (8 <= Reflected light <= 25)
    - State 2: Inside White Line (Reflected light > 25)
    """
    if reflection < 8:
        return 0  # State 0: Outside Line (Black Background)
    elif reflection <= 25:
        return 1  # State 1: On Edge of White Line (Target Zone)
    else:
        return 2  # State 2: Inside White Line (White Tape)

def get_current_state():
    return categorize_reflection(color_sensor.reflected_light_intensity)

def check_for_obstacles():
    """
    Reads the infrared sensor proximity (0 to 100).
    A value of ~20 corresponds to approximately 14 cm distance.
    """
    return ir_sensor.proximity

# ==========================================
# 3. Motor Control Functions (For Actions)
# ==========================================

# Track consecutive turns to balance smooth straight lines with sharp 90-degree cornering
consecutive_turns = 0
last_turn_action = None

def execute_action(action_id):
    """
    Executes a movement based on the chosen Q-learning action:
    0: Forward (Straight)
    1: Left Turn (Steers left towards edge when inside white line)
    2: Right Turn (Steers right towards edge when outside on black mat)
    3: Reverse
    
    Adaptive Turning Strategy:
    - 1st turn cycle (Straight line drift): Soft curve (inner speed +4) for zero wobble.
    - 2+ consecutive turn cycles (90-degree corner): Sharp pivot (inner speed -10, outer speed 18)
      to spin in place without overshooting.
    """
    global consecutive_turns, last_turn_action
    
    if action_id == 0:
        # Reset turn tracking when moving straight along the edge
        consecutive_turns = 0
        last_turn_action = None
        left_motor.on(SpeedPercent(BASE_SPEED))
        right_motor.on(SpeedPercent(BASE_SPEED))
        
    elif action_id in (1, 2):
        if action_id == last_turn_action:
            consecutive_turns += 1
        else:
            consecutive_turns = 1
            last_turn_action = action_id
            
        # Determine turning severity:
        if consecutive_turns >= 2:
            # 90-degree corner detected: sharp pivot to stay on track
            inner_speed = -10
            outer_speed = 18
        else:
            # Minor straight-line drift: gentle curve to eliminate wobble
            inner_speed = 4
            outer_speed = BASE_SPEED
            
        if action_id == 1:
            # Left Turn (left wheel is inner)
            left_motor.on(SpeedPercent(inner_speed))
            right_motor.on(SpeedPercent(outer_speed))
        else:
            # Right Turn (right wheel is inner)
            left_motor.on(SpeedPercent(outer_speed))
            right_motor.on(SpeedPercent(inner_speed))
        
    elif action_id == 3:
        consecutive_turns = 0
        last_turn_action = None
        # Reverse
        left_motor.on(SpeedPercent(-20))
        right_motor.on(SpeedPercent(-20))

def stop_motors():
    """Halts both motors and resets turn tracking."""
    global consecutive_turns, last_turn_action
    consecutive_turns = 0
    last_turn_action = None
    if left_motor is not None:
        left_motor.off()
    if right_motor is not None:
        right_motor.off()

# ==========================================
# 4. Non-RL Obstacle Avoidance & Path Finding
# ==========================================

POLL_SECONDS = 0.01
IR_POLL_SECONDS = 0.02
OBSTACLE_THRESHOLD = 20  # IR relative units; preserve restored trigger distance.
RECOVERY_TIMEOUT_SECONDS = 25.0
TURN_SPEED = 12
TURN_TRACK_MM = 101.25  # Effective spacing verified by the physical 90-degree test.
TURN_WHEEL_DIAMETER_MM = 20.0
TURN_IGNORE_DEGREES = 20.0
TURN_CONFIRM_SAMPLES = 3
TURN_STALL_SECONDS = 2.0
REVERSE_CLEARANCE_SPEED = 12
REVERSE_CLEARANCE_TIMEOUT_SECONDS = 6.0
REVERSE_CLEAR_SAMPLES = 3


def wait_for_motion(duration, watch_obstacles=False, recovery_deadline=None):
    """Wait until a deadline while polling stop/IR; return an interruption reason."""
    end = monotonic() + max(0, duration)
    next_ir = monotonic()
    while True:
        if btn.enter:
            return 'CENTER stop'
        now = monotonic()
        if recovery_deadline is not None and now >= recovery_deadline:
            return 'recovery timeout'
        if now >= end:
            return None
        if watch_obstacles and now >= next_ir and end - now > POLL_SECONDS:
            if check_for_obstacles() <= OBSTACLE_THRESHOLD:
                return 'obstacle stop'
            next_ir = monotonic() + IR_POLL_SECONDS
        remaining = end - monotonic()
        if remaining > 0:
            sleep(min(POLL_SECONDS, remaining))


def reverse_to_turn_clearance():
    """Back up only below threshold; stop and confirm clearance before turning."""
    started = monotonic()
    moving = False
    stable = 0
    try:
        if btn.enter:
            return 'CENTER stop'
        if check_for_obstacles() >= OBSTACLE_THRESHOLD:
            return None
        origin = (left_motor.position, right_motor.position)
        anchors = [0.0, 0.0]
        changed = [started, started]
        while True:
            now = monotonic()
            if btn.enter:
                return 'CENTER stop'
            if now - started >= REVERSE_CLEARANCE_TIMEOUT_SECONDS:
                return 'Reverse clearance timeout; stopped'
            if check_for_obstacles() >= OBSTACLE_THRESHOLD:
                if moving:
                    stop_motors()
                    moving = False
                stable += 1
                if stable >= REVERSE_CLEAR_SAMPLES:
                    return None
            else:
                stable = 0
                progress = (origin[0] - left_motor.position,
                            origin[1] - right_motor.position)
                if not moving:
                    anchors = list(progress)
                    changed = [now, now]
                    left_motor.on(SpeedPercent(-REVERSE_CLEARANCE_SPEED))
                    right_motor.on(SpeedPercent(-REVERSE_CLEARANCE_SPEED))
                    moving = True
                for i in range(2):
                    if progress[i] >= anchors[i] + 2:
                        anchors[i], changed[i] = progress[i], now
                    elif now - changed[i] >= TURN_STALL_SECONDS:
                        return 'Reverse motor stalled; stopped'
            sleep(IR_POLL_SECONDS)
    finally:
        stop_motors()


def avoid_obstacle_and_find_path():
    """Clockwise in-place U-turn; accept a line only after leaving the old one."""
    stop_motors()
    clearance_failure = reverse_to_turn_clearance()
    if clearance_failure:
        print('[recovery] {}'.format(clearance_failure))
        return False
    started = monotonic()
    target = TURN_TRACK_MM * 180.0 / TURN_WHEEL_DIAMETER_MM
    found = False
    reason = 'turn timeout'
    try:
        origin = (left_motor.position, right_motor.position)
        anchors = [0.0, 0.0]
        changed = [started, started]
        black_samples = line_samples = 0
        departed = False
        previous_command = None
        while True:
            now = monotonic()
            if btn.enter:
                reason = 'CENTER stop'
                break
            if now - started >= RECOVERY_TIMEOUT_SECONDS:
                break
            progress = (left_motor.position - origin[0],
                        origin[1] - right_motor.position)
            done = [value >= target for value in progress]
            heading = sum(progress) * TURN_WHEEL_DIAMETER_MM / (2 * TURN_TRACK_MM)
            reflection = color_sensor.reflected_light_intensity
            clear = check_for_obstacles() >= OBSTACLE_THRESHOLD
            # Require confirmed black AFTER the initial angular exclusion zone.
            # This prevents accepting the starting tape or a single noisy sample.
            if not departed:
                black_samples = black_samples + 1 if reflection < 8 and heading >= TURN_IGNORE_DEGREES else 0
                if black_samples >= TURN_CONFIRM_SAMPLES:
                    departed = True
            line_samples = line_samples + 1 if departed and reflection >= 8 and clear else 0
            if line_samples >= TURN_CONFIRM_SAMPLES:
                found = True
                reason = 'New line detected during turn; resuming'
                break
            if all(done):
                found = clear
                reason = '180-degree turn complete; resuming' if clear else 'Obstacle still ahead after turn; stopped'
                break
            stalled = False
            for i in range(2):
                if progress[i] >= anchors[i] + 2:
                    anchors[i], changed[i] = progress[i], now
                elif not done[i] and now - changed[i] >= TURN_STALL_SECONDS:
                    stalled = True
            if stalled:
                reason = 'Turn motor stalled; stopped'
                break
            # Correct encoder imbalance while maintaining opposite tread motion.
            correction = max(-3, min(3, (progress[0] - progress[1]) * 0.05))
            command = (0 if done[0] else round(TURN_SPEED - correction, 1),
                       0 if done[1] else -round(TURN_SPEED + correction, 1))
            if command != previous_command:
                left_motor.on(SpeedPercent(command[0]))
                right_motor.on(SpeedPercent(command[1]))
                previous_command = command
            sleep(IR_POLL_SECONDS)
    finally:
        stop_motors()
    print('[recovery] {}'.format(reason))
    return found

# ==========================================
# 5. Q-Learning Initialization & Rewards
# ==========================================

# 3 States: 0 (Outside Black), 1 (On Edge), 2 (Inside White)
# 4 Actions: 0 (Forward), 1 (Left), 2 (Right), 3 (Reverse)
NUM_STATES = 3
NUM_ACTIONS = 4

ACTION_NAMES = {0: "Forward", 1: "Left Turn", 2: "Right Turn", 3: "Reverse"}
STATE_NAMES = {0: "Outside (Black)", 1: "On Edge (Target)", 2: "Inside (White Line)"}

# Initialize Q-table with zeros
Q_table = [[0.0 for _ in range(NUM_ACTIONS)] for _ in range(NUM_STATES)]

# Hyperparameters
alpha = 0.25    # Learning rate
gamma = 0.9     # Discount factor
epsilon = 0  # Exploration rate 0.12 to learn, 0 to exploit


def get_reward(prev_state, action, next_state):
    """
    Edge-Following Reward logic (White Line on Black Background):
    - State 1 (On Edge): Target zone! Forward (+10) is strongly favored. Corrective turns landing on edge (+8). Reverse (-5).
    - State 0 (Outside on Black Mat): Continuing forward (-10). Drifting off edge onto black (-8). Turning Right (Action 2) towards line edge (+7).
    - State 2 (Inside White Line): Drifting inside white line (-2). Turning Left (Action 1) towards edge (+7).
    """
    # Target state: Reached or stayed on edge (State 1)
    if next_state == 1:
        if action == 0:
            return 10    # Max reward for going straight along the edge
        elif action in (1, 2):
            return 8     # High reward for corrective turn landing back on edge
        else:
            return -5    # Reverse on edge
            
    # Outside on black mat (State 0)
    elif next_state == 0:
        if action == 0:
            return -10   # Heavy penalty for continuing forward when lost on black mat!
        elif prev_state == 1:
            return -8    # Drifting off edge onto black background
        elif action == 2:
            return 7     # Turning Right from black mat towards line edge
        else:
            return -6

    # Inside white line (State 2)
    else:
        if prev_state == 1:
            return -2    # Drifting off edge into center of white line
        elif action == 1:
            return 7     # Turning Left from white line towards edge
        else:
            return 1

def select_action(state):
    """Epsilon-greedy action selection."""
    if random.uniform(0, 1) < epsilon:
        return random.randint(0, NUM_ACTIONS - 1)  # Explore
    else:
        max_val = max(Q_table[state])
        return Q_table[state].index(max_val)       # Exploit

def save_q_table(path):
    with open(path, 'w') as f:
        json.dump(Q_table, f)
    print('Q-table saved to {}'.format(path))


def load_q_table(path):
    global Q_table
    with open(path) as f:
        table = json.load(f)
    if (not isinstance(table, list) or len(table) != NUM_STATES
            or any(not isinstance(row, list) or len(row) != NUM_ACTIONS for row in table)
            or any(type(v) not in (int, float) or not -1e100 < v < 1e100
                   for row in table for v in row)):
        raise ValueError('Q-table must contain 3 rows of 4 finite numbers')
    Q_table = table
    print('Loaded Q-table from {}'.format(path))


class TimingStats:
    def __init__(self):
        self.count = self.total = self.overruns = 0
        self.minimum = float('inf')
        self.maximum = 0
        self.last_command = None
        self.cadence_count = self.cadence_total = self.cadence_max = 0
        self.samples = deque(maxlen=200)

    def command(self, now):
        if self.last_command is not None:
            interval = now - self.last_command
            self.cadence_count += 1
            self.cadence_total += interval
            self.cadence_max = max(self.cadence_max, interval)
        self.last_command = now

    def record(self, elapsed, missed_deadline):
        self.count += 1
        self.total += elapsed
        self.minimum = min(self.minimum, elapsed)
        self.maximum = max(self.maximum, elapsed)
        self.overruns += int(missed_deadline)

    def report(self):
        if not self.count:
            print('[timing] No completed control steps')
            return
        print('[timing] steps={} mean_ms={:.2f} min_ms={:.2f} max_ms={:.2f} deadline_overruns={}'.format(
            self.count, self.total * 1000 / self.count, self.minimum * 1000,
            self.maximum * 1000, self.overruns))
        if self.cadence_count:
            print('[timing] command_interval_mean_ms={:.2f} command_interval_max_ms={:.2f}'.format(
                self.cadence_total * 1000 / self.cadence_count, self.cadence_max * 1000))
        print('[trace] Last {} sampled transitions (one per 5 steps)'.format(len(self.samples)))
        for step, state, action, reflection, reward, elapsed in self.samples:
            print('[trace] step={} state={} action={} reflection={} reward={} cycle_ms={:.2f}'.format(
                step, state, ACTION_NAMES[action], reflection, reward, elapsed * 1000))


def control_loop(training, period, max_steps, stats, timing='original'):
    current_state = get_current_state()
    deadline = monotonic()
    while not max_steps or stats.count < max_steps:
        started = monotonic()
        if btn.enter:
            break
        if check_for_obstacles() <= OBSTACLE_THRESHOLD:
            if not avoid_obstacle_and_find_path():
                break
            current_state = get_current_state()
            deadline = monotonic()  # Recovery is outside control-loop timing.
            stats.last_command = None
            continue
        chosen_action = select_action(current_state)
        stats.command(monotonic())
        execute_action(chosen_action)
        missed = False
        if timing == 'original':
            # Restore the working controller's full sleep after motor commands.
            interrupted = wait_for_motion(period, watch_obstacles=True)
        else:
            deadline += period
            remaining = deadline - monotonic()
            missed = remaining < 0
            interrupted = wait_for_motion(max(0, remaining), watch_obstacles=True)
        if interrupted:
            stop_motors()
            # Do not learn from an action interrupted by CENTER or an obstacle.
            if interrupted == 'CENTER stop':
                break
            if not avoid_obstacle_and_find_path():
                break
            current_state = get_current_state()
            deadline = monotonic()
            stats.last_command = None
            continue
        # Read once: the logged reflection and next state describe the same sample.
        reflection = color_sensor.reflected_light_intensity
        next_state = categorize_reflection(reflection)
        reward = get_reward(current_state, chosen_action, next_state)
        if training:
            Q_table[current_state][chosen_action] += alpha * (
                reward + gamma * max(Q_table[next_state]) - Q_table[current_state][chosen_action])
        elapsed = monotonic() - started
        stats.record(elapsed, missed)
        if stats.count % 5 == 0:
            stats.samples.append((stats.count, current_state, chosen_action, reflection, reward, elapsed))
        current_state = next_state
        # Never issue a burst of catch-up actions after a large scheduling delay.
        if timing == 'fixed' and monotonic() >= deadline + period:
            deadline = monotonic()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='EV3 edge follower: frozen run or explicit training')
    parser.add_argument('--train', action='store_true', help='Update Q values and save to a separate training table')
    parser.add_argument('--table', default=Q_FILE, help='Input policy (default: edge_q_table.json)')
    parser.add_argument('--training-table', default='edge_q_table_training.json',
                        help='Training checkpoint: resume if present, save on exit')
    parser.add_argument('--epsilon', type=float, default=0.0,
                        help='Training exploration probability (default 0 preserves current behavior)')
    parser.add_argument('--timing', choices=('original', 'fixed'), default='original',
                        help='Original post-command sleep (default), or experimental fixed period')
    parser.add_argument('--period-ms', type=float, default=100.0, help='Sleep/period duration (default 100 ms)')
    parser.add_argument('--steps', type=int, default=0, help='Stop after this many control steps; 0 runs until stopped')
    args = parser.parse_args(argv)
    if not 0 <= args.epsilon <= 1 or args.period_ms <= 0 or not math.isfinite(args.period_ms) or args.steps < 0:
        parser.error('Require epsilon in [0,1], finite positive period, and nonnegative steps')
    if not args.train and args.epsilon:
        parser.error('--epsilon requires --train')
    if args.train and os.path.realpath(args.table) == os.path.realpath(args.training_table):
        parser.error('Training output must differ from the input policy; use the separate checkpoint')
    return args


def main(argv=None):
    global epsilon
    args = parse_args(argv)
    epsilon = args.epsilon if args.train else 0.0
    source = args.training_table if args.train and os.path.exists(args.training_table) else args.table
    # Missing/invalid tables fail before motors are initialized.
    load_q_table(source)
    print('[mode] {} timing={} period_ms={} epsilon={} table={}'.format(
        'TRAIN' if args.train else 'RUN (frozen)', args.timing, args.period_ms, epsilon, source))
    if args.train:
        print('[mode] Training checkpoint: {}'.format(args.training_table))
    stats = TimingStats()
    initialized = False
    try:
        initialize_hardware()
        initialized = True
        control_loop(args.train, args.period_ms / 1000.0, args.steps, stats, args.timing)
    except KeyboardInterrupt:
        pass
    finally:
        stop_motors()
        # Terminal/disk IO stays outside the moving control loop.
        if initialized and args.train and stats.count:
            save_q_table(args.training_table)
        stats.report()


if __name__ == '__main__':
    main()
