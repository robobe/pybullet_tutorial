import pybullet as p
import pybullet_data
import tkinter as tk
import sys
import signal


client_id = p.connect(p.SHARED_MEMORY, 1234)
if client_id < 0:
    raise RuntimeError("Run: python3 -m pybullet --shared_memory_key=1234 first!")

def handle_sigint(sig, frame):
    print("Ctrl-C pressed — exiting gracefully.")
    root.destroy()
    sys.exit(0)

signal.signal(signal.SIGINT, handle_sigint)

root = tk.Tk()
root.title("PyBullet Joint Control")
# scale = tk.Scale(root, from_=-3.14, to=3.14, orient="horizontal", resolution=0.01, command=set_joint)
# scale.pack()
root.mainloop()
