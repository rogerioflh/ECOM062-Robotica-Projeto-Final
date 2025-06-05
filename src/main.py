import build as sim # CoppeliaSim remote API client
import time
import math
import numpy as np # For potential future kinematic calculations

# --- Simulation Configuration ---
COPPELIASIM_IP = '127.0.0.1'
COPPELIASIM_PORT = 19997 # Default port, use 19999 for non-blocking

# --- Robot and Gripper Configuration ---
# !!! IMPORTANT: Replace these with the actual names from your CoppeliaSim scene !!!
ROBOT_JOINT_NAMES = [
    'joint1', 'joint2', 'joint3',
    'joint4', 'joint5', 'joint6'
]
# For Robotiq 2F-85, often one joint controls the synchronized finger movement.
# Or, you might have a script function in CoppeliaSim to control it.
# This example assumes a simple joint control for the gripper.
GRIPPER_JOINT_NAME = 'Revolute_joint' # Example name
GRIPPER_OPEN_POS_DEG = 0    # Gripper joint angle for open state (degrees)
GRIPPER_CLOSED_POS_DEG = 35 # Gripper joint angle for closed state (degrees)

TARGET_OBJECT_NAME = 'goal' # Name of the object to pick
GRIPPER_TCP_NAME = 'robotiq_gripper' # Name of the Tool Center Point dummy/object

# --- Predefined Robot Configurations (Joint Angles in Degrees) ---
# These would typically be found using Inverse Kinematics from desired Cartesian poses.
# Values are illustrative and NEED to be adjusted for your specific setup and task.
Q_REST = [0, 30, 60, 0, -90, 0] # Ponto A: Repouso

# Ponto B (Pegar Objeto - frente, na linha do braço estendido)
# These Q values assume the object is directly in front at a certain height and extension
Q_APPROACH_PICK = [0, 30, 60, 0, -90, 0]  # Approach above object
Q_GRASP_OBJECT = [0, 45, 75, 0, -75, 0]   # Grasping position

# Ponto C (Retornar para trás com objeto) - Could be same as Q_APPROACH_PICK or slightly higher
Q_RETREAT_BACK = [0, 30, 60, 0, -90, 0]

# Ponto D (Rotacionar base à direita com objeto)
# Assuming alpha = 90 degrees for rotation to the right. Adjust as needed.
ALPHA_ROTATION_DEG = -90 # Negative for right-hand rotation around Z-axis for many robots
Q_ROTATE_BASE = [ALPHA_ROTATION_DEG, 30, 60, 0, -90, 0] # q1 changed, others from Q_RETREAT_BACK

# Ponto E (Avançar e soltar o objeto) - Similar to Q_GRASP_OBJECT but with rotated base
Q_APPROACH_RELEASE = [ALPHA_ROTATION_DEG, 30, 60, 0, -90, 0] # Approach above drop-off
Q_RELEASE_OBJECT = [ALPHA_ROTATION_DEG, 45, 75, 0, -75, 0] # Release position

# --- Helper Functions ---

def connect_to_coppeliasim():
    """Establishes connection with CoppeliaSim."""
    sim.simxFinish(-1) # Just in case, close all opened connections
    client_id = sim.simxStart(COPPELIASIM_IP, COPPELIASIM_PORT, True, True, 5000, 5)
    if client_id != -1:
        print("Connected to CoppeliaSim")
    else:
        print("Failed to connect to CoppeliaSim")
        exit()
    return client_id

def get_object_handles(client_id):
    """Gets handles for robot joints, gripper, and target object."""
    handles = {'joints': [], 'gripper_joint': None, 'target_object': None, 'tcp': None}
    for name in ROBOT_JOINT_NAMES:
        err_code, handle = sim.simxGetObjectHandle(client_id, name, sim.simx_opmode_blocking)
        if err_code == sim.simx_return_ok:
            handles['joints'].append(handle)
        else:
            print(f"Error getting handle for joint: {name}")
            return None
    
    err_code, handles['gripper_joint'] = sim.simxGetObjectHandle(client_id, GRIPPER_JOINT_NAME, sim.simx_opmode_blocking)
    if err_code != sim.simx_return_ok:
        print(f"Error getting handle for gripper joint: {GRIPPER_JOINT_NAME}")
        # return None # Gripper might be optional for some tests

    err_code, handles['target_object'] = sim.simxGetObjectHandle(client_id, TARGET_OBJECT_NAME, sim.simx_opmode_blocking)
    if err_code != sim.simx_return_ok:
        print(f"Error getting handle for target object: {TARGET_OBJECT_NAME}")
        # return None

    err_code, handles['tcp'] = sim.simxGetObjectHandle(client_id, GRIPPER_TCP_NAME, sim.simx_opmode_blocking)
    if err_code != sim.simx_return_ok:
        print(f"Error getting handle for TCP: {GRIPPER_TCP_NAME}")
        # return None
        
    return handles

def set_joint_target_positions(client_id, joint_handles, target_q_deg, speed_factor=0.5):
    """Sets target positions for multiple joints and waits until movement is likely complete."""
    print(f"Moving to: {target_q_deg}")
    current_q_deg = []

    # Set max velocity (optional, for smoother movement)
    # for handle in joint_handles:
    #     sim.simxSetObjectFloatParameter(client_id, handle, sim.sim_jointfloatparam_max_velocity, speed_factor * math.pi/4, sim.simx_opmode_oneshot)

    for i, handle in enumerate(joint_handles):
        target_rad = math.radians(target_q_deg[i])
        sim.simxSetJointTargetPosition(client_id, handle, target_rad, sim.simx_opmode_oneshot)

    # Wait for movement to complete (simple heuristic)
    # A more robust way would involve checking if joints have reached target
    time.sleep(0.5) # Initial delay for movement to start
    while True:
        all_settled = True
        current_q_rad_temp = []
        for i, handle in enumerate(joint_handles):
            err_code, q_rad = sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_buffer) # Use _buffer or _streaming appropriately
            if err_code == sim.simx_return_ok:
                current_q_rad_temp.append(q_rad)
                if abs(q_rad - math.radians(target_q_deg[i])) > math.radians(2): # Tolerance of 2 degrees
                    all_settled = False
            else:
                # print(f"Warning: Could not read joint {i} position.")
                all_settled = False # Assume not settled if can't read
                break 
        
        if not current_q_rad_temp and len(joint_handles)>0 : # if list is empty, means first calls to getJointPosition haven't returned yet
             sim.simxGetPingTime(client_id) # Ensure commands are processed
             for i, handle in enumerate(joint_handles): # Start streaming if not already
                 sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_streaming)
             time.sleep(0.1)
             continue


        if all_settled:
            # print("Movement likely complete.")
            break
        time.sleep(0.1) # Check interval
    print("Movement complete.")


def control_gripper(client_id, gripper_joint_handle, action, open_pos_deg, closed_pos_deg):
    """Controls the gripper to open or close."""
    if gripper_joint_handle is None:
        print("Gripper not available.")
        return

    if action == "open":
        target_pos_rad = math.radians(open_pos_deg)
        print("Opening gripper...")
    elif action == "close":
        target_pos_rad = math.radians(closed_pos_deg)
        print("Closing gripper...")
    else:
        print(f"Invalid gripper action: {action}")
        return

    sim.simxSetJointTargetPosition(client_id, gripper_joint_handle, target_pos_rad, sim.simx_opmode_oneshot)
    time.sleep(1.0) # Allow time for gripper to actuate
    print(f"Gripper action '{action}' complete.")

def attach_object_to_gripper(client_id, tcp_handle, object_handle):
    """Attaches the object to the gripper's TCP (kinematically)."""
    if tcp_handle and object_handle:
        # Parent the object to the TCP
        # This makes the object follow the TCP.
        # Ensure the object is dynamic and respondable if physics is involved before parenting,
        # or make it non-static and non-respondable after parenting for pure kinematic attachment.
        # For simplicity, we'll just parent it.
        sim.simxSetObjectParent(client_id, object_handle, tcp_handle, True, sim.simx_opmode_oneshot_wait)
        print(f"Object attached to gripper TCP.")
    else:
        print("Could not attach object: TCP or object handle missing.")

def detach_object_from_gripper(client_id, object_handle, scene_object_handle=-1):
    """Detaches the object from the gripper, making it child of the scene."""
    if object_handle:
        # Parent the object to the world (or a specific base plate)
        sim.simxSetObjectParent(client_id, object_handle, scene_object_handle, True, sim.simx_opmode_oneshot_wait)
        print(f"Object detached from gripper.")
    else:
        print("Could not detach object: object handle missing.")

# --- Main Pick and Place Logic ---
def perform_pick_and_place(client_id, handles):
    """Executes the full pick and place sequence."""

    # 0. Ensure gripper is open and robot is at rest
    print("Initializing: Moving to REST and opening gripper.")
    control_gripper(client_id, handles['gripper_joint'], "open", GRIPPER_OPEN_POS_DEG, GRIPPER_CLOSED_POS_DEG)
    set_joint_target_positions(client_id, handles['joints'], Q_REST)
    
    # 1. Ir para frente e agarrar objeto (Ponto B)
    print("\nStep 1: Moving to pick object...")
    set_joint_target_positions(client_id, handles['joints'], Q_APPROACH_PICK)
    set_joint_target_positions(client_id, handles['joints'], Q_GRASP_OBJECT)
    control_gripper(client_id, handles['gripper_joint'], "close", GRIPPER_OPEN_POS_DEG, GRIPPER_CLOSED_POS_DEG)
    attach_object_to_gripper(client_id, handles['tcp'], handles['target_object']) # Attach object
    time.sleep(0.5) # Ensure grasp

    # 2. Voltar para trás (Ponto C)
    print("\nStep 2: Retreating with object...")
    set_joint_target_positions(client_id, handles['joints'], Q_RETREAT_BACK) # Could be Q_APPROACH_PICK

    # 3. Rotacionar base à direita (Ponto D)
    print("\nStep 3: Rotating base...")
    # Here, Q_ROTATE_BASE already has q1 modified.
    # If you wanted to calculate this dynamically based on current q2-q6,
    # you would read current joints, modify q1, then set.
    set_joint_target_positions(client_id, handles['joints'], Q_ROTATE_BASE)

    # 4. Avançar com o objeto (Ponto E)
    print("\nStep 4: Moving to release object...")
    set_joint_target_positions(client_id, handles['joints'], Q_APPROACH_RELEASE)
    set_joint_target_positions(client_id, handles['joints'], Q_RELEASE_OBJECT)
    
    # 5. Soltar objeto
    print("\nStep 5: Releasing object...")
    control_gripper(client_id, handles['gripper_joint'], "open", GRIPPER_OPEN_POS_DEG, GRIPPER_CLOSED_POS_DEG)
    detach_object_from_gripper(client_id, handles['target_object']) # Detach object
    time.sleep(0.5)

    # 6. Return to a safe/retreat position after release, then to rest
    print("\nStep 6: Returning to rest...")
    set_joint_target_positions(client_id, handles['joints'], Q_APPROACH_RELEASE) # Retreat slightly
    set_joint_target_positions(client_id, handles['joints'], Q_REST)

    print("\nPick and Place sequence completed!")

# --- Script Execution ---
if __name__ == "__main__":
    client_id = connect_to_coppeliasim()
    
    if client_id != -1:
        # Start simulation if not already running
        # sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)
        
        # Get handles
        object_handles = get_object_handles(client_id)

        if object_handles and all(object_handles['joints']):
            # Initialize streaming for joint positions for the wait logic
            for handle in object_handles['joints']:
                 sim.simxGetJointPosition(client_id, handle, sim.simx_opmode_streaming)
            time.sleep(0.1) # Allow streaming to start

            # Perform the task
            try:
                input("Press Enter to start the Pick and Place sequence...")
                sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot_wait) # Ensure simulation is running
                perform_pick_and_place(client_id, object_handles)
            except Exception as e:
                print(f"An error occurred: {e}")
            finally:
                # Stop simulation and disconnect
                sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot_wait)
                sim.simxFinish(client_id)
                print("Disconnected from CoppeliaSim.")
        else:
            print("Could not obtain all necessary object handles. Exiting.")
            sim.simxFinish(client_id)