# Factor Graph Based SLAM fusing IMU, BearingRange and USBL(GPS) Factors
This repository implements a factor graph and solves it incrementally with the iSAM2 algorithm from the GTSAM library https://gtsam.org/. This code corresponds to the paper: 

With HoloOcean (https://byu-holoocean.github.io/holoocean-docs/), this code implements a simple interface for generating a trajectory and the interial measurements, by either manually driving or replay an already driven trajectory. This allows for evaluation under different noise levels of the IMU. 

## Installation

Using HoloOcean is only supported under Windows and Linux distributions. GTSAM Python library can be installed via conda-forge.

We recommand using a Conda enviornment.

Create a new envrionment and activate it:

```sh
$ conda create -n factorslam python=3.9
$ conda activate factorslam
```

Then install the needed packages for running the SLAM code and the example jupyter notebook:

```sh
$ (factorslam) conda install numpy pandas matplotlib scipy ipykernal notebook
$ (factorslam) conda install gtsam -c conda-forge
```

If HoloOcean should be installed (Not supported under MacOS):
```sh
$ (factorslam) pip install holoocean
```

Test usage by running the example jupyter notebook:
```sh
$ (factorslam) jupyter notebook
```

In the jupyter notebook, open and run factorslam/example.ipynb .
