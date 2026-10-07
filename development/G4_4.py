#!/usr/bin/env python3

# Import the necessary ev3dev2 classes
import json
import os
import random
from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_3
from ev3dev2.sensor.lego import ColorSensor, InfraredSensor

# ==========================================
# 1. Hardware Initialization
# ==========================================

# Initialize the motors (Assuming left motor is on Port C, right on Port B)
left_motor = LargeMotor(OUTPUT_C)
right_motor = LargeMotor(OUTPUT_B)

# Initialize the sensors
# Color sensor facing down on Port 1, Infrared facing forward on Port 3
color_sensor = ColorSensor(INPUT_1)
ir_sensor = InfraredSensor(INPUT_3)

# Set the color sensor to measure reflected light intensity (0 to 100)
color_sensor.mode = 'COL-REFLECT'

# Set default motor speeds for smooth control and sharp 90-degree corners
BASE_SPEED = 22 
Q_FILE = "q_table.json"

# ==========================================
# 2. Sensor Reading Functions (For States)
# ==========================================

def get_current_state():
    """
    Reads the color sensor and categorizes it into a discrete state for Q-learning (5cm White Tape on Black Mat):
    - State 0: Fully on 5cm white tape (Reflected light > 30)
    - State 1: On edge of 5cm tape / corner entry (8 <= Reflected light <= 30)
    - State 2: Lost on black background (Reflected light < 8)
    """
    reflection = color_sensor.reflected_light_intensity
    
    if reflection > 30:
        return 0  # State 0: On 5cm white tape
    elif reflection >= 8:
        return 1  # State 1: On edge / corner transition
    else:
        return 2  # State 2: Lost (black background)

def check_for_obstacles():
    """
    Reads the infrared sensor proximity (0 to 100).
    A value of ~20 corresponds to approximately 14 cm distance.
    """
    return ir_sensor.proximity

# ==========================================
# 3. Motor Control Functions (For Actions)
# ==========================================

def execute_action(action_id):
    """
    Executes a movement based on the chosen Q-learning action:
    0: Forward (Straight)
    1: Left Pivot Turn (Sharp 90-degree corner execution)
    2: Right Pivot Turn (Sharp 90-degree corner execution)
    3: Reverse
    """
    if action_id == 0:
        # Forward
        left_motor.on(SpeedPercent(BASE_SPEED))
        right_motor.on(SpeedPercent(BASE_SPEED))
        
    elif action_id == 1:
        # Left Pivot Turn (Pivots inner wheel to execute sharp 90° left corner)
        left_motor.on(SpeedPercent(-5))
        right_motor.on(SpeedPercent(BASE_SPEED))
        
    elif action_id == 2:
        # Right Pivot Turn (Pivots inner wheel to execute sharp 90° right corner)
        left_motor.on(SpeedPercent(BASE_SPEED))
        right_motor.on(SpeedPercent(-5))
        
    elif action_id == 3:
        # Reverse
        left_motor.on(SpeedPercent(-20))
        right_motor.on(SpeedPercent(-20))

def stop_motors():
    """Halts both motors."""
    left_motor.off()
    right_motor.off()

# ==========================================
# 4. Non-RL Obstacle Avoidance & Path Finding
# ==========================================

def avoid_obstacle_and_find_path():
    """
    Non-RL routine for obstacle avoidance and line re-acquisition:
    1. Stop motors.
    2. Turn 90 degrees right to bypass obstacle.
    3. Move forward past the obstacle.
    4. Turn 90 degrees left.
    5. Move forward alongside obstacle.
    6. Turn 90 degrees left towards line.
    7. FIND THE PATH (non-RL): Drive forward scanning until white line is detected.
    """
    print("\n[OBSTACLE AVOIDANCE] Obstacle detected! Starting non-RL bypass maneuver...")
    stop_motors()
    sleep(0.5)
    
    # 1. Turn Right 90 degrees
    left_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 1.0, block=False)
    right_motor.on_for_seconds(SpeedPercent(-BASE_SPEED), 1.0, block=True)
    
    # 2. Drive forward past obstacle front
    left_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 1.5, block=False)
    right_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 1.5, block=True)
    
    # 3. Turn Left 90 degrees
    left_motor.on_for_seconds(SpeedPercent(-BASE_SPEED), 1.0, block=False)
    right_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 1.0, block=True)
    
    # 4. Drive forward past obstacle side
    left_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 2.0, block=False)
    right_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 2.0, block=True)
    
    # 5. Turn Left 90 degrees towards the line
    left_motor.on_for_seconds(SpeedPercent(-BASE_SPEED), 1.0, block=False)
    right_motor.on_for_seconds(SpeedPercent(BASE_SPEED), 1.0, block=True)
    
    # 6. PATH FINDING (non-RL)
    print("[PATH FINDING] Searching for white line...")
    left_motor.on(SpeedPercent(15))
    right_motor.on(SpeedPercent(15))
    
    # Keep driving until white line (> 20 reflection) is detected
    while color_sensor.reflected_light_intensity < 20:
        sleep(0.05)
        
    stop_motors()
    print("[PATH FINDING] White line re-acquired! Resuming RL control.\n")
    sleep(0.5)

# ==========================================
# 5. Q-Learning Initialization & Rewards
# ==========================================

# 3 States: 0 (On Line), 1 (Edge), 2 (Lost)
# 4 Actions: 0 (Forward), 1 (Left), 2 (Right), 3 (Reverse)
NUM_STATES = 3
NUM_ACTIONS = 4

ACTION_NAMES = {0: "Forward", 1: "Left Turn", 2: "Right Turn", 3: "Reverse"}
STATE_NAMES = {0: "On White Line", 1: "Edge/Drifting", 2: "Lost (Black)"}

# Initialize Q-table with zeros
Q_table = [[0.0 for _ in range(NUM_ACTIONS)] for _ in range(NUM_STATES)]

# Hyperparameters
alpha = 0.25    # Learning rate
gamma = 0.9     # Discount factor
epsilon = 0.12  # Exploration rate

def get_reward(prev_state, action, next_state):
    """
    Reward logic:
    - State 0 (On White Line): Forward (+10) is strongly favored. Turns (+5). Reverse (-5).
    - Recovering (Edge/Lost -> White Line): Turning (Actions 1/2) gets HIGH reward (+10). Forward when lost gets (-8).
    - Remaining Lost (State 2 -> State 2): Penalty (-6).
    """
    # Reached or stayed on white line (State 0)
    if next_state == 0:
        if action == 0:
            return 10    # Max reward for going forward on white line
        elif action in (1, 2):
            return 8     # High reward for corrective turn landing back on white line!
        else:
            return -5    # Reverse on line
            
    # On edge (State 1)
    elif next_state == 1:
        if prev_state == 0:
            return -2    # Drifting off line
        elif prev_state == 2 and action in (1, 2):
            return 7     # Turning back from Lost to Edge
        else:
            return 1
            
    # Completely lost (State 2)
    else:
        if action == 0:
            return -10   # Heavy penalty for continuing forward when lost!
        elif prev_state == 1:
            return -8    # Driving off line from Edge
        else:
            return -6

def select_action(state):
    """Epsilon-greedy action selection."""
    if random.uniform(0, 1) < epsilon:
        return random.randint(0, NUM_ACTIONS - 1)  # Explore
    else:
        max_val = max(Q_table[state])
        return Q_table[state].index(max_val)       # Exploit

def save_q_table():
    """Saves Q-table to JSON file for persistent trained performance."""
    try:
        with open(Q_FILE, 'w') as f:
            json.dump(Q_table, f)
        print("\nQ-table saved to {}".format(Q_FILE))
    except Exception as e:
        print("Failed to save Q-table: {}".format(e))

def load_q_table():
    """Loads pre-trained Q-table if available."""
    global Q_table
    if os.path.exists(Q_FILE):
        try:
            with open(Q_FILE, 'r') as f:
                Q_table = json.load(f)
            print("Loaded pre-trained Q-table from {}".format(Q_FILE))
        except Exception as e:
            print("Could not load Q-table: {}".format(e))

# ==========================================
# 6. Main Loop Execution
# ==========================================

if __name__ == '__main__':
    try:
        print("==========================================")
        print("   EV3 Q-Learning Line Follower Starting   ")
        print("==========================================")
        
        load_q_table()
        current_state = get_current_state()
        
        step_count = 0
        while True:
            # 1. Non-RL Obstacle Avoidance Override (IR Proximity < 20)
            if check_for_obstacles() < 20:
                avoid_obstacle_and_find_path()
                current_state = get_current_state()
                continue
            
            # 2. RL Decision Pipeline
            chosen_action = select_action(current_state)
            execute_action(chosen_action)
            
            # Action execution duration
            sleep(0.12) 
            
            # Observe environment transition
            next_state = get_current_state()
            reward = get_reward(current_state, chosen_action, next_state)
            
            # Q-learning Bellman Update Equation: 
            # Q(s, a) = Q(s, a) + alpha * [Reward + gamma * max(Q(s', a')) - Q(s, a)]
            best_next_action_val = max(Q_table[next_state])
            Q_table[current_state][chosen_action] += alpha * (
                reward + gamma * best_next_action_val - Q_table[current_state][chosen_action]
            )
            
            step_count += 1
            if step_count % 5 == 0:
                refl = color_sensor.reflected_light_intensity
                print("Refl: {:<2} | State: {:<15} | Action: {:<10} | Reward: {:<3}".format(
                    refl,
                    STATE_NAMES[current_state],
                    ACTION_NAMES[chosen_action],
                    reward
                ))
            
            current_state = next_state
            
    except KeyboardInterrupt:
        print("\nStopping robot...")
        stop_motors()
        save_q_table()
        
        print("\nFinal Learned Q-Table:")
        print("{:<15} {:<10} {:<10} {:<10} {:<10}".format("State", "Forward", "Left", "Right", "Reverse"))
        for s in range(NUM_STATES):
            print("{:<15} {:<10.2f} {:<10.2f} {:<10.2f} {:<10.2f}".format(
                STATE_NAMES[s],
                Q_table[s][0],
                Q_table[s][1],
                Q_table[s][2],
                Q_table[s][3]
            ))