import time
import math
import numpy as np
import threading
from tkinter import Tk, Scale, HORIZONTAL
from hexapod_dynamixel.lib.servo import *
from hexapod_dynamixel.lib.hexapod_constant import *

start_time = time.time()

# Initialize the servo
servo = DynamixelServo(device_name="COM10", baudrate=1000000)
servo_ids = list(range(1, 19))  # Servo IDs from 1 to 18
goal_positions = [512 for _ in range(18)]  # All servos to position 512
servo.enable_torque(servo_ids)

# Define trajectory function with vx and vy for wave gait
def trajectory(t, p, phase, step_height, step_duration, vx, vy, v_rot, leg_index):
    # Add phase shift to create staggered leg movement
    cycle_time = (t + phase) % step_duration  # Ensure that the cycle repeats every step_duration
    progress = cycle_time / step_duration  # Normalize progress between 0 and 1

    # Debugging: Check progress and cycle time
    # print(f"leg: {leg_index}, t: {t}, phase: {phase}, cycle_time: {cycle_time}, progress: {progress}")

    # Calculate the offset based on the leg's rotational component
    angle = math.pi / 2 + math.atan2(leg_base_positions[leg_index][0], leg_base_positions[leg_index][1])
    offset_x = v_rot * math.sin(angle)
    offset_y = v_rot * math.cos(angle)

    # Trajectory movement based on progress:
    if progress < p:  # Lifting phase (legs are lifting and moving forward)
        z = step_height * math.sin(2 * math.pi * progress)
        x = (vx + offset_x) * (progress / p - 0.5)
        y = (vy + offset_y) * (progress / p - 0.5)
    else:  # Lowering phase (legs are lowering and moving backward)
        z = 0
        x = (vx + offset_x) * (0.5 - (progress - p) / (1-p))
        y = (vy + offset_y) * (0.5 - (progress - p) / (1-p))
    

    # If no movement is desired (e.g., stationary), set z to 0
    if vx == 0 and vy == 0 and v_rot == 0:
        z = 0

    return x, y, z

# Define inverse kinematics function
def inverse_kinematics(x, y, z):
    coxa_angle = math.pi / 2 + math.atan2(x, y)
    r = math.sqrt(x**2 + y**2)
    d = math.sqrt(r**2 + z**2)

    if d > femur_length + tibia_length:
        d = femur_length + tibia_length

    a1 = math.atan2(z, r)
    A = math.acos((d**2 + femur_length**2 - tibia_length**2) / (2 * d * femur_length))
    femur_angle = math.pi / 2 - (A + a1)

    B = math.acos((femur_length**2 + tibia_length**2 - d**2) / (2 * femur_length * tibia_length))
    tibia_angle = math.pi - B

    return coxa_angle, femur_angle, tibia_angle

# Define body kinematics with rotation
def body_kinematics(body_position, body_orientation):
    x_trans, y_trans, z_trans = body_position
    r, p, y = body_orientation
    roll = math.radians(r)
    pitch = math.radians(p)
    yaw = math.radians(y)

    # Compute the rotation matrix for the given roll, pitch, and yaw
    c_r, s_r = math.cos(roll), math.sin(roll)
    c_p, s_p = math.cos(pitch), math.sin(pitch)
    c_y, s_y = math.cos(yaw), math.sin(yaw)

    # Rotation matrix combining roll, pitch, and yaw
    rotation_matrix = np.array([
        [c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r],
        [s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r],
        [-s_p, c_p * s_r, c_p * c_r]
    ])

    leg_positions = []

    for leg_base in leg_base_positions:
        # Apply rotation to the leg base position
        rotated_leg_base = np.dot(rotation_matrix, np.array(leg_base))

        # Apply translation to the rotated position
        leg_x = rotated_leg_base[0] + x_trans
        leg_y = rotated_leg_base[1] + y_trans
        leg_z = rotated_leg_base[2] + z_trans

        leg_positions.append((leg_x, leg_y, leg_z))

    return leg_positions

def dxl_pos(radians):
    """Map a value from 0 to 300 degrees (in radians) to 0-1023 for Dynamixel."""
    max_radians = math.radians(300)  # Convert 300 degrees to radians (~5.24)
    
    if radians < 0 or radians > max_radians:
        print(f"Radians must be between 0 and {max_radians:.2f} (300 degrees)")
        if radians < 0:
            radians = 0
        elif radians>300:
            radians = 300

    return int((radians / max_radians) * 1023)

# Control each leg joint
def joint_control(joint_index, pos):
    target_x, target_y, target_z = pos
    if joint_index>=10:
        target_x*=-1
        target_y*=-1
    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(target_x, target_y, target_z)
    
    # print("joint_index: ", joint_index)
    if(joint_index==1):
        coxa_servo = math.radians(180+15) - leg_angle - coxa_angle
        femur_servo = math.radians(180+30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        # print(math.degrees(coxa_servo), math.degrees(femur_servo), math.degrees(tibia_servo))
        goal_positions[0] = dxl_pos(coxa_servo)
        goal_positions[1] = dxl_pos(femur_servo)
        goal_positions[2] = dxl_pos(tibia_servo)

    if(joint_index==4):
        coxa_servo = math.radians(180+15) - coxa_angle
        femur_servo = math.radians(180+30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        # print(math.degrees(coxa_servo), math.degrees(femur_servo), math.degrees(tibia_servo))
        goal_positions[3] = dxl_pos(coxa_servo)
        goal_positions[4] = dxl_pos(femur_servo)
        goal_positions[5] = dxl_pos(tibia_servo)
    
    if(joint_index==7):
        coxa_servo = math.radians(180+15) + leg_angle - coxa_angle
        femur_servo = math.radians(180+30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        # print(math.degrees(coxa_servo), math.degrees(femur_servo), math.degrees(tibia_servo))
        goal_positions[6] = dxl_pos(coxa_servo)
        goal_positions[7] = dxl_pos(femur_servo)
        goal_positions[8] = dxl_pos(tibia_servo)

    if(joint_index==10):
        coxa_servo = math.radians(180+15) - leg_angle - coxa_angle
        femur_servo = math.radians(180+30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        # print(math.degrees(coxa_servo), math.degrees(femur_servo), math.degrees(tibia_servo))
        goal_positions[9] = dxl_pos(coxa_servo)
        goal_positions[10] = dxl_pos(femur_servo)
        goal_positions[11] = dxl_pos(tibia_servo)

    if(joint_index==13):
        coxa_servo = math.radians(180+15) - coxa_angle
        femur_servo = math.radians(180+30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        # print(math.degrees(coxa_servo), math.degrees(femur_servo), math.degrees(tibia_servo))
        goal_positions[12] = dxl_pos(coxa_servo)
        goal_positions[13] = dxl_pos(femur_servo)
        goal_positions[14] = dxl_pos(tibia_servo)
    
    if(joint_index==16):
        coxa_servo = math.radians(180+15) + leg_angle - coxa_angle
        femur_servo = math.radians(180+30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        # print(math.degrees(coxa_servo), math.degrees(femur_servo), math.degrees(tibia_servo))
        goal_positions[15] = dxl_pos(coxa_servo)
        goal_positions[16] = dxl_pos(femur_servo)
        goal_positions[17] = dxl_pos(tibia_servo)

# Create a Tkinter window
root = Tk()
root.title("Hexapod Control")

# Create sliders
slider_length = 400

vx_slider = Scale(root, from_=-0.05, to=0.05, resolution=0.01, orient=HORIZONTAL, label="vx", length=slider_length)
vx_slider.set(0.0)
vx_slider.pack()

vy_slider = Scale(root, from_=-0.05, to=0.05, resolution=0.01, orient=HORIZONTAL, label="vy", length=slider_length)
vy_slider.set(0.0)
vy_slider.pack()

v_rot_slider = Scale(root, from_=-0.05, to=0.05, resolution=0.01, orient=HORIZONTAL, label="v_rot", length=slider_length)
v_rot_slider.set(0.0)
v_rot_slider.pack()

step_height_slider = Scale(root, from_=0, to=0.2, resolution=0.01, orient=HORIZONTAL, label="step height", length=slider_length)
step_height_slider.set(0.05)
step_height_slider.pack()

step_duration_slider = Scale(root, from_=0.1, to=10, resolution=0.1, orient=HORIZONTAL, label="step duration", length=slider_length)
step_duration_slider.set(1)
step_duration_slider.pack()

cpg_slider = Scale(root, from_=0.01, to=20, resolution=0.1, orient=HORIZONTAL, label="cpg", length=slider_length)
cpg_slider.set(2)
cpg_slider.pack()

x_slider = Scale(root, from_=-0.03, to=0.03, resolution=0.001, orient=HORIZONTAL, label="pos x", length=slider_length)
x_slider.set(0)
x_slider.pack()

y_slider = Scale(root, from_=-0.03, to=0.03, resolution=0.001, orient=HORIZONTAL, label="pos y", length=slider_length)
y_slider.set(0)
y_slider.pack()

z_slider = Scale(root, from_=-0.03, to=0.03, resolution=0.001, orient=HORIZONTAL, label="pos z", length=slider_length)
z_slider.set(0)
z_slider.pack()

r_slider = Scale(root, from_=-30, to=30, resolution=0.01, orient=HORIZONTAL, label="roll", length=slider_length)
r_slider.set(0)
r_slider.pack()

p_slider = Scale(root, from_=-30, to=30, resolution=0.01, orient=HORIZONTAL, label="pitch", length=slider_length)
p_slider.set(0)
p_slider.pack()

yaw_slider = Scale(root, from_=-30, to=30, resolution=0.01, orient=HORIZONTAL, label="yaw", length=slider_length)
yaw_slider.set(0)
yaw_slider.pack()

# Function to update the robot's movement
def update_robot():
    last_update_time = 0
    while True:
        try:
            t_start = time.time()
            t = time.time() - start_time

            if t_start - last_update_time >= 0.01:
                last_update_time = t_start
                # Read slider values for vx and vy
                vx = vx_slider.get()
                vy = vy_slider.get()
                v_rot = v_rot_slider.get()
                step_height = step_height_slider.get()
                step_duration = step_duration_slider.get()
                cpg = cpg_slider.get()

                body_position = (
                    x_slider.get(),
                    y_slider.get(),
                    z_slider.get(),
                )

                body_orientation = (
                    r_slider.get(),
                    p_slider.get(),
                    yaw_slider.get(),
                )

            # Compute leg positions
            leg_pos_body = body_kinematics(body_position, body_orientation)

            for leg_index in range(6):  # Iterate over all 6 legs
                # phase_shifts = [i * (step_duration / 6) for i in range(6)]
                i = 1/cpg
                # phase_shifts = [0, i, 0, i, 0, i]
                phase_shifts = [0, step_duration/cpg, 2*step_duration/cpg, 3*step_duration/cpg, 4*step_duration/cpg, 5*step_duration/cpg]
                phase = phase_shifts[leg_index]
                # print(phase_shifts)
                
                # Compute trajectory
                pos = trajectory(t, i, phase, step_height, step_duration, vx, vy, v_rot, leg_index)
                leg_base = leg_pos_body[leg_index]
                leg_pos = (leg_base[0] + pos[0], leg_base[1] + pos[1], leg_base[2] + pos[2])
                
                # Control leg joint angles
                joint_control(joint_index=leg_index * 3 + 1, pos=leg_pos)
            
            # Move servos
            # print(servo_ids)
            # print(goal_positions)
            servo.write(servo_ids, goal_positions)
            time.sleep(1 / 240)  # Adjust simulation speed if necessary

            t_end = time.time()
            t_total = t_end-t_start + 1/1e10
            # print(f"Robot: time: {t_total:.2f} seconds, fps: {1/t_total:.2f}")

        except Exception as e:
            print(e)
            servo.disable_torque(servo_ids)
            servo.close()
            break

# Start the robot control thread
robot_thread = threading.Thread(target=update_robot, daemon=True)
robot_thread.start()

# Start the Tkinter main loop
root.mainloop()

servo.disable_torque(servo_ids)
servo.close()