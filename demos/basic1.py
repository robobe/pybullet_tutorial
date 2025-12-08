import tkinter as tk
import threading
import time
import pybullet as p
import pybullet_data



def print_joints(robot_id):
    num_joints = p.getNumJoints(robot_id)
    print("Total joints:", num_joints)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        print(i, info[1].decode("utf-8"), "| type =", info[2])


def pybullet_thread():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    
    robot = p.loadURDF("racecar/racecar.urdf")
    plane = p.loadURDF("plane.urdf")

    print_joints(robot)
    # Disable default wheel motors
    for wheel in [2,3,4,5]:
        p.changeDynamics(robot, wheel, lateralFriction=1.0)

    for j in [2, 3]:
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0)

    while True:
        pos, orn = p.getBasePositionAndOrientation(robot)
        # update tk variable safely
        txt.set(f"Position: {pos}")
        # keyboard handler
        keys = p.getKeyboardEvents()
        if ord('j') in keys:
            vel = 10
        elif ord('k') in keys:
            vel = -10
        else:
            vel = 0

        p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, targetVelocity=vel, force=20)
        p.setJointMotorControl2(robot, 2, p.VELOCITY_CONTROL, targetVelocity=vel, force=20)

        p.stepSimulation()
        time.sleep(1/240)

# ----------------- GUI (main thread) -----------------


root = tk.Tk()
root.title("Robot Debugger")

txt = tk.StringVar()
label = tk.Label(root, textvariable=txt, font=("Arial", 18))
label.pack()

# Start pybullet in separate thread
threading.Thread(target=pybullet_thread, daemon=True).start()

# Tkinter MUST be in the main thread
root.mainloop()
