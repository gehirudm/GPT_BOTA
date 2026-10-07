#!/usr/bin/env python3

# Import the necessary ev3dev2 classes
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor
from time import sleep
import random

# ==========================================
# 1. Hardware Initialization
# ==========================================

# Initialize the motors (Assuming left motor is on Port B, right on Port C)
left_motor = LargeMotor(OUTPUT_C)
right_motor = LargeMotor(OUTPUT_B)

# Initialize the sensors
# Color sensor facing down on Port 4
color_sensor = ColorSensor(INPUT_4)

# Set the color sensor to measure reflected light intensity (0 to 100)
color_sensor.mode = 'COL-REFLECT'

# Set a default speed for your actions
BASE_SPEED = 30 

# ==========================================
# 2. Sensor Reading Functions (For States)
# ==========================================

def get_current_state():
    """
    Reads the color sensor and categorizes it into a discrete state for Q-learning.
    You will need to calibrate these threshold values based on your room's lighting.
    """
    reflection = color_sensor.reflected_light_intensity
    
    if reflection < 15:
        return 0  # State 0: On the black line
    elif reflection < 40:
        return 1  # State 1: On the edge (drifting)
    else:
        return 2  # State 2: Lost on white background

# ==========================================
# 3. Motor Control Functions (For Actions)
# ==========================================

def execute_action(action_id):
    """
    Executes a movement based on the chosen Q-learning action.
    """
    if action_id == 0:
        # Forward
        left_motor.on(SpeedPercent(BASE_SPEED))
        right_motor.on(SpeedPercent(BASE_SPEED))
        
    elif action_id == 1:
        # Left Turn (Left stops/reverses, Right goes forward)
        left_motor.off()
        right_motor.on(SpeedPercent(BASE_SPEED))
        
    elif action_id == 2:
        # Right Turn (Left goes forward, Right stops/reverses)
        left_motor.on(SpeedPercent(BASE_SPEED))
        right_motor.off()
        
    elif action_id == 3:
        # Reverse
        left_motor.on(SpeedPercent(-BASE_SPEED))
        right_motor.on(SpeedPercent(-BASE_SPEED))

def stop_motors():
    """Halts both motors."""
    left_motor.off()
    right_motor.off()

# ==========================================
# 4. Q-Learning Initialization
# ==========================================

# 3 States: 0 (On Line), 1 (Edge), 2 (Lost)
# 4 Actions: 0 (Forward), 1 (Left), 2 (Right), 3 (Reverse)
NUM_STATES = 3
NUM_ACTIONS = 4

# Initialize Q-table with zeros
Q_table = [[0.0 for _ in range(NUM_ACTIONS)] for _ in range(NUM_STATES)]

# Hyperparameters
alpha = 0.1     # Learning rate
gamma = 0.9     # Discount factor
epsilon = 0.2   # Exploration rate

def get_reward(state):
    """Assigns a reward based on the state."""
    if state == 0:
        return 10   # High reward for staying on the line
    elif state == 1:
        return -1   # Small penalty for drifting to the edge
    else:
        return -5   # Larger penalty for being lost

def select_action(state):
    """Epsilon-greedy action selection."""
    if random.uniform(0, 1) < epsilon:
        return random.randint(0, NUM_ACTIONS - 1)  # Explore
    else:
        # Exploit: max Q-value
        max_val = max(Q_table[state])
        return Q_table[state].index(max_val)


# ==========================================
# 5. Main Loop Structure
# ==========================================

if __name__ == '__main__':
    try:
        print("Starting robot with Q-learning...")
        
        # Initial state
        current_state = get_current_state()
        
        while True:
            # RL Pipeline
            # Select action based on current state
            chosen_action = select_action(current_state)
            
            execute_action(chosen_action)
            
            # Allow the motor to run for a fraction of a second before taking the next reading
            sleep(0.1) 
            
            # Observe new state and reward
            next_state = get_current_state()
            reward = get_reward(next_state)
            
            # Q-learning update formula: 
            # Q(s, a) = Q(s, a) + alpha * [Reward + gamma * max(Q(s', a')) - Q(s, a)]
            best_next_action_val = max(Q_table[next_state])
            Q_table[current_state][chosen_action] += alpha * (reward + gamma * best_next_action_val - Q_table[current_state][chosen_action])
            
            print("State: {} | Action: {} | Reward: {} | Next State: {}".format(current_state, chosen_action, reward, next_state))
            
            # Update current state for next iteration
            current_state = next_state
            
    except KeyboardInterrupt:
        # Gracefully stop the motors if you press Ctrl+C in the terminal
        print("Stopping robot...")
        stop_motors()
        
        # Optional: Print Q-table at the end
        print("\nFinal Q-table:")
        for s in range(NUM_STATES):
            print("State {}: {}".format(s, Q_table[s]))
