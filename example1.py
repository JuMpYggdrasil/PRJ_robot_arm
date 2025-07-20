import pybullet as p
import time
import pybullet_data
import numpy as np

# 1. Setup the simulation environment
physicsClient = p.connect(p.GUI)  # Or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Set simulation parameters for better control
SIMULATION_TIME_STEP = 1./240. # Store the time step in a variable
p.setTimeStep(SIMULATION_TIME_STEP)
p.setPhysicsEngineParameter(numSolverIterations=1000) # More solver iterations for accuracy

# 2. Load the robot arm and a plane
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
plane_id = p.loadURDF("plane.urdf")

# 3. Get joint information for the Kuka IIWA
num_joints = p.getNumJoints(robot_id)
controllable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        controllable_joints.append(i)
        print(f"Found controllable joint: {joint_name} (Index: {i})")

# Define initial and target joint angles
initial_joint_angles = [0.0, -0.7, 0.0, 1.57, 0.0, 0.7, 0.0] # A 'home' or safe pose
target_joint_angles_1 = [np.pi/4, -np.pi/4, np.pi/4, -np.pi/2, np.pi/4, -np.pi/4, np.pi/4]
target_joint_angles_2 = [-np.pi/4, np.pi/4, -np.pi/4, np.pi/2, -np.pi/4, np.pi/4, -np.pi/4]

# Ensure joint angle lists match the number of controllable joints
if len(initial_joint_angles) != len(controllable_joints):
    print(f"Warning: Initial joint angles count ({len(initial_joint_angles)}) does not match controllable joints ({len(controllable_joints)}). Adjusting.")
    initial_joint_angles = initial_joint_angles[:len(controllable_joints)]
if len(target_joint_angles_1) != len(controllable_joints):
    print(f"Warning: Target joint angles 1 count ({len(target_joint_angles_1)}) does not match controllable joints ({len(controllable_joints)}). Adjusting.")
    target_joint_angles_1 = target_joint_angles_1[:len(controllable_joints)]
if len(target_joint_angles_2) != len(controllable_joints):
    print(f"Warning: Target joint angles 2 count ({len(target_joint_angles_2)}) does not match controllable joints ({len(controllable_joints)}). Adjusting.")
    target_joint_angles_2 = target_joint_angles_2[:len(controllable_joints)]

# Set initial pose
for i, joint_pos in zip(controllable_joints, initial_joint_angles):
    p.resetJointState(robot_id, i, joint_pos)

# Define PID gains for position control
KP = 0.5 # Proportional gain
KD = 1.0 # Derivative gain
MAX_FORCE = 500 # Maximum force the motor can apply
MAX_VELOCITY = 2 # Maximum velocity for position control

# 4. Simulation Loop with Joint Position Control
print("\n--- Moving to Target Pose 1 ---")
for t in range(240 * 3): # Simulate for 3 seconds
    p.stepSimulation()
    for i, target_angle in zip(controllable_joints, target_joint_angles_1):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            positionGain=KP,
            velocityGain=KD,
            maxVelocity=MAX_VELOCITY,
            force=MAX_FORCE
        )
    time.sleep(SIMULATION_TIME_STEP) # Use the stored variable here

print("\n--- Moving to Target Pose 2 ---")
for t in range(240 * 3): # Simulate for another 3 seconds
    p.stepSimulation()
    for i, target_angle in zip(controllable_joints, target_joint_angles_2):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_angle,
            positionGain=KP,
            velocityGain=KD,
            maxVelocity=MAX_VELOCITY,
            force=MAX_FORCE
        )
    time.sleep(SIMULATION_TIME_STEP) # Use the stored variable here

# Hold position for a moment
print("\n--- Holding final pose ---")
for _ in range(240 * 1):
    p.stepSimulation()
    time.sleep(SIMULATION_TIME_STEP) # Use the stored variable here

# --- THIS IS THE CRUCIAL PART TO KEEP THE WINDOW OPEN ---
print("\n--- Simulation finished. Keeping GUI open. Press 'Ctrl+C' in terminal or close window. ---")
try:
    while p.isConnected(): # Keep running as long as PyBullet GUI is connected
        p.stepSimulation() # Continue stepping to keep GUI responsive
        time.sleep(SIMULATION_TIME_STEP) # Use the stored variable here
except p.error:
    # This exception is caught when the user closes the PyBullet GUI window
    print("PyBullet GUI disconnected by user.")
except KeyboardInterrupt:
    # This exception is caught when the user presses Ctrl+C in the terminal
    print("Simulation interrupted by user.")
finally:
    # 5. Disconnect
    if p.isConnected():
        p.disconnect()
    print("PyBullet disconnected.")