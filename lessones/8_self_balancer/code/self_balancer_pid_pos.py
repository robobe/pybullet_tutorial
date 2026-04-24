# made by Ayush Agarwal , 20095021 , electronics
# ######################################IMPORTANT#############################################
# ###################### PLEASE USE ARROW KEYS to move the robot , and use f and b for flips
# ###########################################################################################
# importing libraries
import os
import sys
import json
import pidcontrol as pid
import numpy as np
import pybullet as p
import math
import socket
import time
import pybullet_data

PITCH_SETPOINT = 0
KP = 16
KI = 0.08
KD = 100
POSITION_SETPOINT = 0
POSITION_KP = 0.2
POSITION_KI = 0
POSITION_KD = 0.8
MAX_PITCH_SETPOINT = 0.25
PLOTJUGGLER_HOST = "127.0.0.1"
PLOTJUGGLER_PORT = 9870

POSITION_INDEX = 0
ORIENTATION_INDEX = 1

ROLL_INDEX = 0
PITCH_INDEX = 1
YAW_INDEX = 2


class PlotJugglerUdpClient:
    """
    Send telemetry as JSON over UDP for PlotJuggler's UDP server.
    """

    def __init__(self, host=PLOTJUGGLER_HOST, port=PLOTJUGGLER_PORT):
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(
        self,
        timestamp,
        position_setpoint,
        position_feedback,
        pitch_setpoint,
        pitch_feedback,
    ):
        data = {
            "time": timestamp,
            "position/setpoint": position_setpoint,
            "position/feedback": position_feedback,
            "pitch/setpoint": pitch_setpoint,
            "pitch/feedback": pitch_feedback,
            "pitch/setpoint_deg": pitch_setpoint * 180 / math.pi,
            "pitch/feedback_deg": pitch_feedback * 180 / math.pi,
        }
        self.socket.sendto(json.dumps(data).encode("utf-8"), self.address)


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

    def __init__(
        self,
        Kp=KP,
        Ki=KI,
        Kd=KD,
        position_Kp=POSITION_KP,
        position_Ki=POSITION_KI,
        position_Kd=POSITION_KD,
    ):
        self.xvelMin = -0.01
        self.xvelMax = 0
        self.yMin = -0.01
        self.yMax = -0.001
        self.yPrev = 0
        self.delY = 0
        # here are the values to which the PID controller is perfectly tuned
        self.Kp = Kp  # 2.7
        self.Ki = Ki  # 0.005
        self.Kd = Kd  # 1
        self.controller = pid.PID_Controller(self.Kp, self.Ki, self.Kd)
        self.position_Kp = position_Kp
        self.position_Ki = position_Ki
        self.position_Kd = position_Kd
        self.position_controller = pid.PID_Controller(
            self.position_Kp, self.position_Ki, self.position_Kd
        )

    def set_gains(self, Kp, Ki, Kd):
        """
        Update PID gains while keeping the controller state.
        """
        if self.Kp == Kp and self.Ki == Ki and self.Kd == Kd:
            return

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.controller.tune(self.Kp, self.Ki, self.Kd)

    def set_position_gains(self, Kp, Ki, Kd):
        """
        Update position PID gains while keeping the controller state.
        """
        if (
            self.position_Kp == Kp
            and self.position_Ki == Ki
            and self.position_Kd == Kd
        ):
            return

        self.position_Kp = Kp
        self.position_Ki = Ki
        self.position_Kd = Kd
        self.position_controller.tune(
            self.position_Kp, self.position_Ki, self.position_Kd
        )

    def update_position(self, current_position):
        """
        Convert position error into a pitch setpoint for the balance PID.
        """
        pitch_setpoint = self.position_controller.update(
            POSITION_SETPOINT, current_position
        )
        return max(-MAX_PITCH_SETPOINT, min(MAX_PITCH_SETPOINT, pitch_setpoint))

    def update(self, current, pitch_setpoint=PITCH_SETPOINT):
        """
        Run pid update try to keep 0 degree PITCH
        
        :param self: Description
        :param current: Description
        """

        xvel = -self.controller.update(
            pitch_setpoint, current
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


def read_x_position(robot):
    """
    Return robot base x position in world coordinates.
    """
    return p.getBasePositionAndOrientation(robot, id)[POSITION_INDEX][0]


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
    
    plotjuggler = PlotJugglerUdpClient()
    controller = SelfBalanceController(
        KP, KI, KD, POSITION_KP, POSITION_KI, POSITION_KD
    )
    kp_param_id = p.addUserDebugParameter("Kp", 0, 100, KP, id)
    ki_param_id = p.addUserDebugParameter("Ki", 0, 100, KI, id)
    kd_param_id = p.addUserDebugParameter("Kd", 0, 100, KD, id)
    position_kp_param_id = p.addUserDebugParameter(
        "Position Kp", 0, 2, POSITION_KP, id
    )
    position_ki_param_id = p.addUserDebugParameter(
        "Position Ki", 0, 1, POSITION_KI, id
    )
    position_kd_param_id = p.addUserDebugParameter(
        "Position Kd", 0, 5, POSITION_KD, id
    )

    while True:
        timestamp = time.time()
        Kp = p.readUserDebugParameter(kp_param_id, id)
        Ki = p.readUserDebugParameter(ki_param_id, id)
        Kd = p.readUserDebugParameter(kd_param_id, id)
        controller.set_gains(Kp, Ki, Kd)
        position_Kp = p.readUserDebugParameter(position_kp_param_id, id)
        position_Ki = p.readUserDebugParameter(position_ki_param_id, id)
        position_Kd = p.readUserDebugParameter(position_kd_param_id, id)
        controller.set_position_gains(position_Kp, position_Ki, position_Kd)

        position = read_x_position(robot)  # get robot position in world frame
        pitch_setpoint = controller.update_position(position)
        pitch = read_pitch(robot)  # get robot state (pitch angle from IMU)
        vel = controller.update(
            pitch, pitch_setpoint
        )  # calculating torque to be applied on wheels
        plotjuggler.send(
            timestamp,
            POSITION_SETPOINT,
            position,
            pitch_setpoint,
            pitch,
        )
        p.addUserDebugText(
            "Pitch: %.2f | X: %.2f | Pitch setpoint: %.2f"
            % (
                pitch * 180 / math.pi,
                position,
                pitch_setpoint * 180 / math.pi,
            ),
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
