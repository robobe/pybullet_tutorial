import typeings.pybullet as p
import pybullet_data
import time
import threading
import queue
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import pi as PI


# Thread-safe queue
data_queue = queue.Queue()

# ------------------------------------
# BACKGROUND PYBULLET SIMULATION
# ------------------------------------
def bullet_thread():
    print("[Bullet] Thread started")
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.8)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane = p.loadURDF("plane.urdf")

    # Path to your URDF
    p.setAdditionalSearchPath("/workspaces/pybullet_tutorial/demos/urdf")

    robot = p.loadURDF("rrbot.urdf", useFixedBase=True)

    # Disable default motors
    for j in range(p.getNumJoints(robot)):
        p.setJointMotorControl2(robot, j, p.VELOCITY_CONTROL, force=0)

    joint_id = 0
    torque = 5.0
    t = 0.0

    while True:
        # Physics step
        p.setJointMotorControl2(robot, joint_id, p.TORQUE_CONTROL, force=torque)
        p.stepSimulation()
        time.sleep(1/240)
        t += 1/240

        # Get joint position
        pos, vel, _, _ = p.getJointState(robot, joint_id)
        pos = (pos + PI) % (2 * PI) - PI
        # Debug (comment out later)
        # print(f"[Bullet] t={t:.3f}, pos={pos:.3f}")

        # Send to plot thread
        data_queue.put((t, pos))


# Start background simulation
thread = threading.Thread(target=bullet_thread, daemon=True)
thread.start()


# ------------------------------------
# REAL-TIME MATPLOTLIB PLOT
# ------------------------------------
plt.style.use("ggplot")
fig, ax = plt.subplots()

times = []
positions = []

line, = ax.plot([], [], lw=2)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Joint Position (rad)")
ax.set_title("Real-Time Joint Position (Threaded PyBullet)")
ax.set_xlim(0, 5)
ax.set_ylim(-3, 3)


def update_plot(frame):
    updated = False

    # Fetch ALL available data quickly
    while not data_queue.empty():
        t, pos = data_queue.get()
        times.append(t)
        positions.append(pos)
        updated = True
        print(t, pos)

    if not updated:
        return line,

    # Sliding window
    t_last = times[-1]
    if t_last > 5:
        ax.set_xlim(t_last - 5, t_last)

    line.set_data(times, positions)
    return line,


# IMPORTANT: disable frame caching (fix warning)
ani = animation.FuncAnimation(
    fig,
    update_plot,
    interval=30,
    blit=True,
    cache_frame_data=False
)

plt.show()
