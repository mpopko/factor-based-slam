import holoocean
import numpy as np
import keyboard  
from datetime import datetime
import os
import threading
import time
import json
import argparse

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
    parser = argparse.ArgumentParser(description="Play or replay a holoocean scenario.")
    parser.add_argument(
        '--config',
        type=argparse.FileType('r'),
        default='scenes/example.json',
        help='Path to the JSON configuration file (default: scenes/example.json)'
    )
    
    # Boolean flag for manual mode
    manual_group = parser.add_mutually_exclusive_group()
    manual_group.add_argument('--manual', dest='manual', action='store_true', help='Enable manual control (default)')
    manual_group.add_argument('--no-manual', dest='manual', action='store_false', help='Disable manual control and replay commands')
    parser.set_defaults(manual=True)

    # Optional replay file
    parser.add_argument(
        '--replay',
        type=argparse.FileType('r'),
        help='Path to a replay file containing commands (commands are saved in the npz file)'
    )

    args = parser.parse_args()

    if not args.manual and not args.replay:
        parser.error("--replay is required when --no-manual is specified.")
    
    imu = []
    commands = []
    poses = []
    times = []

    # Start key detection in separate thread
    key_thread = threading.Thread(target=update_pressed_keys, daemon=True)
    key_thread.start()

    cfg = json.load(args.config)
    with holoocean.make(scenario_cfg=cfg) as env:

        if args.manual:
            print("Press Q to exit and save. W/A/S/D is Forward/Left/Backwards/Right and I/J/K/L is Upwards/Turn-Left/Downwards/Turn-Right.")
            while True:
                if 'q' in pressed_keys:
                    break
                command = parse_keys(pressed_keys, force)
                env.act("auv0", command)
                state = env.tick()
                imu.append(state["IMUSensor"])
                poses.append(state["PoseSensor"])
                commands.append(command)
                times.append(state["t"])
        else:
            d = np.load(args.replay.name)
            commands = d["commands"]
            for command in commands:
                env.act("auv0", command)
                state = env.tick()
                imu.append(state["IMUSensor"])
                poses.append(state["PoseSensor"])
                times.append(state["t"])


    current_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    os.makedirs(f"data/{current_datetime}", exist_ok=True)
    np.savez_compressed(f"data/{current_datetime}/data", imu=np.array(imu), commands=np.array(commands), poses=np.array(poses), t=np.array(times))
    print("Data saved: " + f"data/{current_datetime}/data")
