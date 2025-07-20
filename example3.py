import pybullet as p
import time
import pybullet_data
import numpy as np

# 1. Setup the simulation environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.setTimeStep(1./240.)
p.setPhysicsEngineParameter(numSolverIterations=1000)

# 2. Load the robot arm, a plane, and an object
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
plane_id = p.loadURDF("plane.urdf")

# Create a small cube to pick up
cube_start_pos = [0.5, 0.0, 0.05] # On the plane
cube_start_orn = p.getQuaternionFromEuler([0, 0, 0])
cube_id = p.loadURDF("cube.urdf", cube_start_pos, cube_start_orn, globalScaling=0.05) # Small cube

# 3. Get joint information and end-effector link
num_joints = p.getNumJoints(robot_id)
controllable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    joint_type = info[2]
    if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
        controllable_joints.append(i)

end_effector_link_index = 6 # lbr_iiwa_link_7

# Set initial safe joint positions
initial_joint_positions = [0.0, 0.0, 0.0, 1.57, 0.0, -0.78, 0.0]
for i, joint_pos in zip(controllable_joints, initial_joint_positions):
    p.resetJointState(robot_id, i, joint_pos)

# State variables for pick and place
GRIPPER_OPEN_POS = 0.05 # A conceptual value for a real gripper
GRIPPER_CLOSE_POS = 0.0 # A conceptual value for a real gripper
GRIPPER_STATE = GRIPPER_OPEN_POS
object_grasped = False
kuka_constraint = None # To hold the object

# Target poses for pick and place
pick_approach_pos = [0.5, 0.0, 0.2]  # Above the cube
pick_pos = [0.5, 0.0, 0.03]          # At the cube (adjusted for cube height)
place_pos = [0.0, 0.5, 0.5]          # Target place position
place_approach_pos = [0.0, 0.5, 0.6] # Above the place position

# Fixed orientation for simplicity (pointing down)
gripper_orn = p.getQuaternionFromEuler([0, np.pi/2, 0])

# --- Helper function for moving to a pose ---
def move_to_pose(target_position, target_orientation, robot_id, ee_link_idx, controllable_joints, sim_steps=240):
    for _ in range(sim_steps):
        p.stepSimulation()
        joint_states = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=ee_link_idx,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            jointDamping=[0.1] * len(controllable_joints)
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
        time.sleep(p.getTimeStep())

# --- Simulation Logic ---

print("\n--- Moving to Pick Approach Position ---")
move_to_pose(pick_approach_pos, gripper_orn, robot_id, end_effector_link_index, controllable_joints)

print("\n--- Moving to Pick Position ---")
move_to_pose(pick_pos, gripper_orn, robot_id, end_effector_link_index, controllable_joints)

# Check for "grasp" condition (e.g., end-effector close to object)
# This is a simplified check; in reality, you might use distance sensors or visual feedback
ee_state = p.getLinkState(robot_id, end_effector_link_index)
distance_to_cube = np.linalg.norm(np.array(ee_state[0]) - np.array(p.getBasePositionAndOrientation(cube_id)[0]))
print(f"Distance to cube: {distance_to_cube:.3f}m")

if distance_to_cube < 0.08: # If close enough (adjust threshold)
    print("--- Grasping Object ---")
    # Simulate closing gripper (if robot had one)
    # GRIPPER_STATE = GRIPPER_CLOSE_POS

    # Create a fixed constraint to "grasp" the object
    # Attach cube base to end-effector link (link 6) of robot
    cube_current_pos, cube_current_orn = p.getBasePositionAndOrientation(cube_id)
    ee_current_pos, ee_current_orn = p.getLinkState(robot_id, end_effector_link_index)[0:2]

    # Calculate the offset from the end-effector link to the object's origin
    # This offset is crucial for maintaining the object's relative position when grasped
    inv_ee_pos, inv_ee_orn = p.invertTransform(ee_current_pos, ee_current_orn)
    obj_in_ee_frame_pos, obj_in_ee_frame_orn = p.multiplyTransforms(inv_ee_pos, inv_ee_orn, cube_current_pos, cube_current_orn)

    kuka_constraint = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=end_effector_link_index,
        childBodyUniqueId=cube_id,
        childLinkIndex=-1, # -1 for base link of child
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=obj_in_ee_frame_pos,
        childFramePosition=[0, 0, 0], # Object's origin is its frame origin
        childFrameOrientation=obj_in_ee_frame_orn
    )
    object_grasped = True
else:
    print("--- Failed to grasp object: Not close enough ---")

time.sleep(1) # Small pause to confirm grasp

print("\n--- Moving to Place Approach Position ---")
move_to_pose(place_approach_pos, gripper_orn, robot_id, end_effector_link_index, controllable_joints)

print("\n--- Moving to Place Position ---")
move_to_pose(place_pos, gripper_orn, robot_id, end_effector_link_index, controllable_joints)

if object_grasped:
    print("--- Releasing Object ---")
    p.removeConstraint(kuka_constraint)
    object_grasped = False
    # Simulate opening gripper
    # GRIPPER_STATE = GRIPPER_OPEN_POS
else:
    print("--- No object to release ---")

time.sleep(1) # Small pause to confirm release

print("\n--- Moving back to Home Pose ---")
# Reset to initial_joint_positions if you want to explicitly go back
# Or simply move to another safe pose
move_to_pose(pick_approach_pos, gripper_orn, robot_id, end_effector_link_index, controllable_joints)


# Final simulation step to ensure everything settles
for _ in range(240):
    p.stepSimulation()
    time.sleep(p.getTimeStep())

# Disconnect
p.disconnect()