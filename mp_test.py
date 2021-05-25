import os
import multiprocessing as mp
from Experiment.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
import time

# Simple test to see if we can get a good stiffness and damping from the steering wheel

Bw = 0.4  # Max = 0.5
Kw = 2  # Max = 2.5
torque = 0


class TesterClass:
    def __init__(self, parent, child, Bw, Kw):
        super().__init__()
        self.parent = parent
        self.child = child
        self.send_dict = {"msg": "hey!",
                 "torque": 0,
                 "damping": Bw,
                 "stiffness": Kw,
                 "exit": False}

    def do(self):
        N = 10
        delta = 4 / N
        # Send 10 messages
        for i in range(N):
            t0 = time.time()
            self.send_dict["torque"] = i * 0.2
            parent = self.parent
            parent.send(self.send_dict)  # Child is for sending
            msg = self.parent.recv()  # Receive from child
            print(msg["steering_angle"], "<-- angle")
            t = time.time() - t0
            time.sleep(max(0, delta - t))

        # Finish process
        self.send_dict["exit"] = True
        parent_conn.send(self.send_dict)  # Child is for s


if __name__ == "__main__":
    parent_conn, child_conn = mp.Pipe(True)
    senso_drive_process = SensoDriveModule(Bw, Kw, parent_conn, child_conn)
    senso_drive_process.start()
    print("made it so far")

    tester = TesterClass(parent_conn, child_conn, Bw, Kw)
    tester.do()
    senso_drive_process.join()
