import platform
import multiprocessing as mp
import time
from SensoDriveClean import SensoDriveModule

if __name__ == "__main__":
    if platform.system() == 'Windows':
        import wres

        # Start the senso drive parallel process
        parent_conn1, child_conn1 = mp.Pipe(True)
        parent_conn2, child_conn2 = mp.Pipe(True)
        senso_dict1 = {
            "stiffness": 0.8,
            "damping": 0.4,
            "parent_conn": parent_conn1,
            "child_conn": child_conn1,
        }
        senso_dict2 = {
            "stiffness": 0.8,
            "damping": 0.4,
            "parent_conn": parent_conn2,
            "child_conn": child_conn2,
        }

        # Automatically find the correct sensodrive module
        senso_drive_process1 = SensoDriveModule(senso_dict1, 0)
        senso_drive_process2 = SensoDriveModule(senso_dict2, 1)
        senso_drive_process1.start()
        senso_drive_process2.start()

        send_dict1 = {
            "difference": 0
        }
        send_dict2 = {
            "difference": 0
        }
        reference1 = 0
        reference2 = 0
        t_start = time.time()
        while time.time() - t_start < 20:
            # Read out sensodrive1
            parent_conn1.send(send_dict1)  # Child is for sending
            new_states1 = parent_conn1.recv()  # Receive from child
            reference1 = new_states1["steering_angle"]
            difference1 = reference2 - reference1
            send_dict2["difference"] = difference1

            # Hand as reference to sensodrive2

            # Read out sensodrive 2
            parent_conn2.send(send_dict2)  # Child is for sending
            new_states2 = parent_conn2.recv()  # Receive from child
            reference2 = new_states2["steering_angle"]
            difference2 = reference1 - reference2
            send_dict1["difference"] = difference2

            # hand as reference to sensodrive1

            # Repeat

        # senso_drive_process = SensoDriveModule(senso_dict, PCAN_USBBUS1)