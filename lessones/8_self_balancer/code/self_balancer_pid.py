# made by Ayush Agarwal , 20095021 , electronics
# ######################################IMPORTANT#############################################
# ###################### PLEASE USE ARROW KEYS to move the robot , and use f and b for flips
# ###########################################################################################
# importing libraries
import os
import sys
import pidcontrol as pid
import numpy as np
import pybullet as p
import math
import time
import pybullet_data

PITCH_SETPOINT = 0

POSITION_INDEX = 0
ORIENTATION_INDEX = 1

ROLL_INDEX = 0
PITCH_INDEX = 1
YAW_INDEX = 2


class SelfBalanceController:
    """
    Purpose:
    ---
    Class Describing the gains of LQR and various functions
    Functions:
    ---
        __init__ : Called by default to initilize the variables in PID
        callback : It takes the data from sensors/bots and accordingly predicts the next state and respecive action
        callback_Kp : It can be used to change the value of gains during execution
        callback_Ki : It can be used to change the value of gains during execution
        callback_Kd : It can be used to change the value of gains during execution
    Example initialization and function call:
    ---
    balance=SelfBalanceLQR()
    vel=balance.callback(data)
    """

    def __init__(self):
        self.xvelMin = -0.01
        self.xvelMax = 0
        self.yMin = -0.01
        self.yMax = -0.001
        self.yPrev = 0
        self.delY = 0
        # here are the values to which the PID controller is perfectly tuned
        self.Kp = 16  # 2.7
        self.Ki = 0.08  # 0.005
        self.Kd = 100  # 1
        self.controller = pid.PID_Controller(self.Kp, self.Ki, self.Kd)

    def update(self, current):
        """
        Run pid update try to keep 0 degree PITCH
        
        :param self: Description
        :param current: Description
        """

        xvel = -self.controller.update(
            PITCH_SETPOINT, current
        )  

        return xvel

    # region Callbacks for changing PID gains during execution
    def callback_Kp(self, data):
        self.Kp = data.data
        self.controller = pid.PID_Controller(self.Kp, self.Ki, self.Kd)

    def callback_Ki(self, data):
        self.Ki = data.data
        self.controller = pid.PID_Controller(self.Kp, self.Ki, self.Kd)

    def callback_Kd(self, data):
        self.Kd = data.data
        self.controller = pid.PID_Controller(self.Kp, self.Ki, self.Kd)

    # endregion


def read_pitch(robot):
    """
    return robot pitch angle like IMU sensor data
    """
    data = p.getEulerFromQuaternion(
        p.getBasePositionAndOrientation(robot, id)[ORIENTATION_INDEX]
    )[PITCH_INDEX]
    return data


# Main Function

if __name__ == "__main__":
    # region init pybullet environment
    id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # loading the plane URDF
    plane = p.loadURDF("plane.urdf")
    # setting the gravity for the environment
    p.setGravity(0, 0, -9.8)
    # loading the robot in the environment
    cwd = os.path.dirname(os.path.abspath(__file__))
    cwd = os.path.join(cwd, "urdf")
    p.setAdditionalSearchPath(cwd)
    robot = p.loadURDF("demos/urdf/self_balance.urdf", [0, 0, 0.2])

    text_id = p.addUserDebugText(
        "Pitch: 0.00", textPosition=[0, 0, 2], textColorRGB=[1, 0, 0]
    )
    # endregion

    # region disabling the built-in wheel motors:
    left_joint = 0
    right_joint = 1
    maxForce = 0
    mode = p.VELOCITY_CONTROL
    p.setJointMotorControl2(robot, left_joint, controlMode=mode, force=maxForce)
    p.setJointMotorControl2(robot, right_joint, controlMode=mode, force=maxForce)
    # endregion
    
    controller = SelfBalanceController()

    while True:
        pitch = read_pitch(robot)  # get robot state (pitch angle from IMU)
        vel = controller.update(pitch)  # calculating torque to be applied on wheels
        p.addUserDebugText(
            str("Pitch: %.2f" % (pitch * 180 / math.pi)),
            textPosition=[0, 0, 2],
            textColorRGB=[1, 0, 0],
            replaceItemUniqueId=text_id,
        )

        # Here the controller (PID) gives the torque force on the motors , thus balancing it
        p.setJointMotorControl2(robot, left_joint, p.TORQUE_CONTROL, force=vel)
        p.setJointMotorControl2(robot, right_joint, p.TORQUE_CONTROL, force=-vel)
        p.stepSimulation()
        time.sleep(0.01)

        # region keyboard events for moving the bot
        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            # to move forward , the wheels should make it move forward
            if k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN):
                trqfwd = 6
                p.stepSimulation()
                p.setJointMotorControl2(
                    robot, left_joint, p.TORQUE_CONTROL, force=-trqfwd
                )
                p.setJointMotorControl2(
                    robot, right_joint, p.TORQUE_CONTROL, force=trqfwd
                )
                p.stepSimulation()
            if k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN):
                trqfwd = 6
                p.stepSimulation()
                p.setJointMotorControl2(
                    robot, left_joint, p.TORQUE_CONTROL, force=trqfwd
                )
                p.setJointMotorControl2(
                    robot, right_joint, p.TORQUE_CONTROL, force=-trqfwd
                )
                p.stepSimulation()
            # to move left , we use the concept of differential drive
            if k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN):
                trqfwd = 1
                p.stepSimulation()
                p.setJointMotorControl2(
                    robot, left_joint, p.TORQUE_CONTROL, force=trqfwd
                )
                p.setJointMotorControl2(
                    robot, right_joint, p.TORQUE_CONTROL, force=trqfwd
                )
                p.stepSimulation()
            if k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN):
                trqfwd = 1
                p.stepSimulation()
                p.setJointMotorControl2(
                    robot, left_joint, p.TORQUE_CONTROL, force=-trqfwd
                )
                p.setJointMotorControl2(
                    robot, right_joint, p.TORQUE_CONTROL, force=-trqfwd
                )
                p.stepSimulation()
            ########FRONT FLIP ON PRESSING F
            # front flip can be achieved in pybullet environment by applying a huge torque
            if k == ord("f") and (v & p.KEY_IS_DOWN):
                trqfwd = 400
                p.stepSimulation()
                p.setJointMotorControl2(
                    robot, left_joint, p.TORQUE_CONTROL, force=-trqfwd
                )
                p.setJointMotorControl2(
                    robot, right_joint, p.TORQUE_CONTROL, force=trqfwd
                )
                p.stepSimulation()
            ########BACK FLIP ON PRESSING B
            # similarly huge torque in opposite direction for back flip
            if k == ord("b") and (v & p.KEY_IS_DOWN):
                trqfwd = -400
                p.stepSimulation()
                p.setJointMotorControl2(
                    robot, left_joint, p.TORQUE_CONTROL, force=-trqfwd
                )
                p.setJointMotorControl2(
                    robot, right_joint, p.TORQUE_CONTROL, force=trqfwd
                )
                p.stepSimulation()

        # endregion
