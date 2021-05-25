import os
import multiprocessing as mp
from Experiment.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
import time

# Simple test to see if we can get a good stiffness and damping from the steering wheel

Bw = 0.4  # Max = 0.5
Kw = 2  # Max = 2.5
torque = 0
N = 10
delta = 4/N

if __name__ == "__main__":
    parent_conn, child_conn = mp.Pipe(True)
    senso_drive_process = SensoDriveModule(Bw, Kw, parent_conn, child_conn)
    senso_drive_process.start()
    send_dict = {"msg": "hey!",
                 "torque": 0,
                 "exit": False}
    # Send 10 messages
    for i in range(N):
        t0 = time.time()
        send_dict["torque"] = i * 0.2
        parent_conn.send(send_dict)  # Child is for s
        time.sleep(0.005)
        msg = parent_conn.recv()
        print(msg, "<-- Should be a hi back")
        t = time.time() - t0
        time.sleep(max(0, delta - t))

    # Finish process
    send_dict["exit"] = True
    parent_conn.send(send_dict)  # Child is for s

    senso_drive_process.join()
