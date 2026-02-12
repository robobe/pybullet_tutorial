import tkinter as tk
import threading
import time
import typeings.pybullet as p
import pybullet_data

counter = 0
running = True


def pybullet_thread():
    global counter
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    robot = p.loadURDF("racecar/racecar.urdf")
    plane = p.loadURDF("plane.urdf")

    while running:
        # update tk variable safely
        txt.set(f"Position: {counter}")
        
        counter += 1
        p.stepSimulation()
        time.sleep(1 / 240)
    p.disconnect()


# ----------------- GUI (main thread) -----------------


def reset_counter():
    global counter
    counter = 0


def on_close():
    global running
    running = False
    time.sleep(0.5)
    print("Close")
    root.destroy()


root = tk.Tk()
root.title("Robot Debugger")
root.protocol("WM_DELETE_WINDOW", on_close)

txt = tk.StringVar()
label = tk.Label(root, textvariable=txt, font=("Arial", 18))
label.pack()
tk.Button(root, text="Reset", command=reset_counter).pack(pady=5)

# Start pybullet in separate thread
threading.Thread(target=pybullet_thread, daemon=True).start()

# Tkinter MUST be in the main thread
root.mainloop()
