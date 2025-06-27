# Factor Graph Based SLAM fusing IMU, BearingRange and USBL(GPS) Factors
This repository implements a factor graph and solves it incrementally with the iSAM2 algorithm from the GTSAM library https://gtsam.org/. This code corresponds to the paper: 

With HoloOcean (https://byu-holoocean.github.io/holoocean-docs/), this code implements a simple interface for generating a trajectory and the interial measurements, by either manually driving or replay an already driven trajectory. This allows for evaluation under different noise levels of the IMU. 

---

## Installation

Using HoloOcean is only supported under Windows and Linux distributions and thorugh directly cloning from Github. GTSAM Python library can be installed via conda-forge.
Note that, HoloOcean is not needed for running the factor-based SLAM functionalities.

We recommand using a Conda environment.

Create a new environment and activate it:

```sh
$ conda create -n factorslam python=3.10
$ conda activate factorslam
```

Then install the needed packages for running the SLAM code and the example jupyter notebook:

```sh
$ (factorslam) conda install numpy pandas matplotlib scipy ipykernel notebook
$ (factorslam) conda install gtsam -c conda-forge
```

Test usage by running the example jupyter notebook:
```sh
$ (factorslam) jupyter notebook
```

In the jupyter notebook, open and run factorslam/example.ipynb .

---

### HoloOcean installation (Not supported under MacOS):
For inputs:
```sh
$ (factorslam) pip install keyboard
```
To install HoloOcean, we refer to their documentation and installation guide: 
https://byu-holoocean.github.io/holoocean-docs/v2.0.0/usage/installation.html

(HoloOcean is based on UnrealEngine and can therefore not be distributed via e.g. pypi. See the refered installation guide for more information.)

Follow the instruction by the creators of HoloOcean and make sure ```python -c `import holoocean; holoocean.install("Ocean")``` has been executed. This installs some sample levels, which are used in our examples.

Make sure everything is installed in the ```(factorslam)``` environment. As example, run:
```sh
$ (factorslam) cd factorslam/holoocean_scenarios
$ (factorslam) python record_replay.py
```
This opens an HoloOcean environment with an BlueROV, which can be controlled via keyboard.

 (Press Q to exit and save. W/A/S/D is Forward/Left/Backwards/Right and I/J/K/L is Upwards/Turn-Left/Downwards/Turn-Right.)

HoloOcean allows for creating different scenarios specified in an scenario JSON-format (see, e.g. `factorslam\holoocean_scenarios\scenes\example.json`).

--- 
### Output data

After running the scenario—whether in **manual mode** or **replay mode**—the script collects sensor data and command history, then saves the results in a compressed format.

#### Output Directory

A new directory is created under the `data/` folder, named with the current date and time:

```
data/YYYY-MM-DD_HH-MM-SS/
```

For example:

```
data/2025-06-27_14-30-45/
```

#### Saved File

Inside this directory, the following file is generated:

```
data.npz
```

This file contains the following arrays:

| Key        | Description                                                                              |
| ---------- | ---------------------------------------------------------------------------------------- |
| `imu`      | Data from the `IMUSensor` collected at each tick.                                        |
| `commands` | The list of commands issued to the agent.                                       |
| `poses`    | Data from the `PoseSensor`, representing the agent's position and orientation over time. |
| `t`        | Timestamps associated with each tick of the environment.                                 |

---

### Command-Line Arguments for record_replay.py

This script allows you to play or replay a Holoocean scenario using various command-line options.

#### Usage

```bash
python your_script.py [--config CONFIG] [--manual | --no-manual] [--replay REPLAY]
```

#### Arguments

##### `--config`

* **Type**: HoloOcean scenario file
* **Default**: `scenes/example.json`
* **Description**:
  Path to the JSON configuration file that defines the Holoocean scenario settings.

  **Example:**

  ```bash
  --config scenes/example.json
  ```

##### `--manual` / `--no-manual`

* **Type**: Boolean flag (mutually exclusive)
* **Default**: `--manual` (manual mode enabled by default)
* **Description**:
  Controls whether the scenario runs in manual mode or not.

  * `--manual`: Enable manual control (default)
  * `--no-manual`: Disable manual control and replay pre-recorded commands

  **Examples:**

  ```bash
  --manual        # Run with manual control
  --no-manual     # Run in replay mode
  ```

##### `--replay`

* **Type**: A before saved data file which contains commands 
* **Description**:
  Path to a replay file containing saved commands (typically in `.npz` format) to be used for replaying a session. This is only used when manual mode is disabled.

  **Example:**

  ```bash
  --replay data/example.npz
  ```


