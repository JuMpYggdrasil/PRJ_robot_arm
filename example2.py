import pybullet as p
import time
import pybullet_data
import numpy as np

# 1. Setup the simulation environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

SIMULATION_TIME_STEP = 1./240. # Store the time step in a variable
p.setTimeStep(SIMULATION_TIME_STEP)
p.setPhysicsEngineParameter(numSolverIterations=1000)

# 2. Load the robot arm and a plane
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
plane_id = p.loadURDF("plane.urdf")

# 3. Get joint information and identify end-effector link
num_joints = p.getNumJoints(robot_id)
controllable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        controllable_joints.append(i)

end_effector_link_index = 6 # lbr_iiwa_link_7

# Set initial safe joint positions
initial_joint_positions = [0.0, 0.0, 0.0, 1.57, 0.0, -0.78, 0.0]
for i, joint_pos in zip(controllable_joints, initial_joint_positions):
    p.resetJointState(robot_id, i, joint_pos)

# 4. Define target end-effector poses
# Pose 1: Straight up and a bit forward
target_pos_1 = [0.0, 0.0, 1.0]
target_orn_1 = p.getQuaternionFromEuler([0, np.pi/2, 0]) # End-effector pointing downwards

# Pose 2: Forward and to the side
target_pos_2 = [0.6, 0.3, 0.4]
target_orn_2 = p.getQuaternionFromEuler([0, 0, np.pi/4]) # Some rotation

# Add debug visualization for target positions
# Use addUserDebugLine to create a small cross or point marker
# For target_pos_1 (red)
p.addUserDebugLine(target_pos_1, [target_pos_1[0], target_pos_1[1], target_pos_1[2] + 0.01], [1, 0, 0], 2, lifeTime=0)
p.addUserDebugLine(target_pos_1, [target_pos_1[0], target_pos_1[1], target_pos_1[2] - 0.01], [1, 0, 0], 2, lifeTime=0)
p.addUserDebugLine([target_pos_1[0]-0.01, target_pos_1[1], target_pos_1[2]], [target_pos_1[0]+0.01, target_pos_1[1], target_pos_1[2]], [1, 0, 0], 2, lifeTime=0)
p.addUserDebugLine([target_pos_1[0], target_pos_1[1]-0.01, target_pos_1[2]], [target_pos_1[0], target_pos_1[1]+0.01, target_pos_1[2]], [1, 0, 0], 2, lifeTime=0)

# For target_pos_2 (green)
p.addUserDebugLine(target_pos_2, [target_pos_2[0], target_pos_2[1], target_pos_2[2] + 0.01], [0, 1, 0], 2, lifeTime=0)
p.addUserDebugLine(target_pos_2, [target_pos_2[0], target_pos_2[1], target_pos_2[2] - 0.01], [0, 1, 0], 2, lifeTime=0)
p.addUserDebugLine([target_pos_2[0]-0.01, target_pos_2[1], target_pos_2[2]], [target_pos_2[0]+0.01, target_pos_2[1], target_pos_2[2]], [0, 1, 0], 2, lifeTime=0)
p.addUserDebugLine([target_pos_2[0], target_pos_2[1]-0.01, target_pos_2[2]], [target_pos_2[0], target_pos_2[1]+0.01, target_pos_2[2]], [0, 1, 0], 2, lifeTime=0)


# 5. Simulation Loop with IK Control
print("\n--- Moving to Target Pose 1 (IK) ---")
for _ in range(240 * 4): # Simulate for 4 seconds
    p.stepSimulation()

    # Calculate IK for target pose 1
    joint_states = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=end_effector_link_index,
        targetPosition=target_pos_1,
        targetOrientation=target_orn_1,
        jointDamping=[0.1] * len(controllable_joints),
        solver=p.IK_SDLS
    )

    if len(joint_states) >= len(controllable_joints):
        for i, joint_idx in enumerate(controllable_joints):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_states[i],
                maxVelocity=3.0,
                force=500
            )

    time.sleep(SIMULATION_TIME_STEP)

print("\n--- Moving to Target Pose 2 (IK) ---")
for _ in range(240 * 4): # Simulate for another 4 seconds
    p.stepSimulation()

    # Calculate IK for target pose 2
    joint_states = p.calculateInverseKinematics(
        bodyUniqueId=robot_id,
        endEffectorLinkIndex=end_effector_link_index,
        targetPosition=target_pos_2,
        targetOrientation=target_orn_2,
        jointDamping=[0.1] * len(controllable_joints),
        solver=p.IK_SDLS
    )

    if len(joint_states) >= len(controllable_joints):
        for i, joint_idx in enumerate(controllable_joints):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_states[i],
                maxVelocity=3.0,
                force=500
            )

    time.sleep(SIMULATION_TIME_STEP)

# Hold position
print("\n--- Holding final pose ---")
for _ in range(240 * 1):
    p.stepSimulation()
    time.sleep(SIMULATION_TIME_STEP)

# --- Keep the window open ---
print("\n--- Simulation finished. Keeping GUI open. Press 'Ctrl+C' in terminal or close window. ---")
try:
    while p.isConnected():
        p.stepSimulation()
        time.sleep(SIMULATION_TIME_STEP)
except p.error:
    print("PyBullet GUI disconnected by user.")
except KeyboardInterrupt:
    print("Simulation interrupted by user.")
finally:
    if p.isConnected():
        p.disconnect()
    print("PyBullet disconnected.")