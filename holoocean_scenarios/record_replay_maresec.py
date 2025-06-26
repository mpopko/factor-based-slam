import holoocean
import numpy as np
import keyboard  # MIT licensed, replacement for pynput
from datetime import datetime
import os
import threading
import time

cfg = {
    "name": "record_replay",
    "world": "MyNewWorld",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 60,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {"sensor_type": "IMUSensor",
                 "socket": "IMUSocket",
                 "configuration": {
                     "AccelCov": (np.eye(3)*(0.023**2)).tolist(),
                     "AngVelCov": (np.eye(3)*(0.00674**2)).tolist(),
                     "AccelBiasCov": (np.eye(3)*(0.00082**2)).tolist(),
                     "AngVelBiasCov": (np.eye(3)*((4.85e-5)**2)).tolist(),
                     "ReturnBias": True
                 }},
                {"sensor_type": "PoseSensor", "socket": "IMUSocket", "configuration": []}
            ],
            "control_scheme": 0,
            "location": [0, 0, -18],
            "rotation": [0, 0, 90]
        }
    ]
}

pressed_keys = set()
force = 25 / 2

def update_pressed_keys():
    global pressed_keys
    while True:
        for key in ['i', 'k', 'j', 'l', 'w', 's', 'a', 'd', 'q']:
            if keyboard.is_pressed(key):
                pressed_keys.add(key)
            else:
                pressed_keys.discard(key)
        time.sleep(0.01)

def parse_keys(keys, val):
    command = np.zeros(8)
    if 'i' in keys:
        command[0:4] += val
    if 'k' in keys:
        command[0:4] -= val
    if 'j' in keys:
        command[[4, 7]] += val / 2
        command[[5, 6]] -= val / 2
    if 'l' in keys:
        command[[4, 7]] -= val / 2
        command[[5, 6]] += val / 2
    if 'w' in keys:
        command[4:8] += val
    if 's' in keys:
        command[4:8] -= val
    if 'a' in keys:
        command[[4, 6]] += val
        command[[5, 7]] -= val
    if 'd' in keys:
        command[[4, 6]] -= val
        command[[5, 7]] += val
    return command

manual = True

if __name__ == "__main__":
    imu = []
    commands = []
    poses = []
    i = 0

    # Start key detection in separate thread
    key_thread = threading.Thread(target=update_pressed_keys, daemon=True)
    key_thread.start()

    with holoocean.make(scenario_cfg=cfg) as env:
        env.spawn_prop("box", [0.0, -15.0, -17.0], [0.0, 0.0, 0.0], 0.1, False, "white", "box")
        env.spawn_prop("box", [15.0, -15.0, -17.0], [0.0, 0.0, 0.0], 0.1, False, "white", "box")
        env.spawn_prop("box", [15.0, 0.0, -17.0], [0.0, 0.0, 0.0], 0.1, False, "white", "box")

        if manual:
            while True:
                if 'q' in pressed_keys:
                    break
                command = parse_keys(pressed_keys, force)
                env.act("auv0", command)
                state = env.tick()
                imu.append(state["IMUSensor"])
                poses.append(state["PoseSensor"])
                commands.append(command)
                if i == 0 or i == 1:
                    print(state)
                i += 1
        else:
            d = np.load("missions\\search1\\data.npz")
            commands = d["commands"]
            for command in commands:
                env.act("auv0", command)
                state = env.tick()
                imu.append(state["IMUSensor"])
                poses.append(state["PoseSensor"])
                if i == 0 or i == 1:
                    print(state)
                i += 1

    current_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    os.makedirs(f"missions/{current_datetime}", exist_ok=True)
    np.savez_compressed(f"missions/{current_datetime}/data", imu=np.array(imu), commands=np.array(commands), poses=np.array(poses))
