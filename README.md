# Factor Graph Based SLAM fusing IMU, BearingRange and USBL(GPS) Factors
This repository implements a factor graph and solves it incrementally with the iSAM2 algorithm from the GTSAM library https://gtsam.org/. This code corresponds to the paper: 

With HoloOcean (https://byu-holoocean.github.io/holoocean-docs/), this code implements a simple interface for generating a trajectory and the interial measurements, by either manually driving or replay an already driven trajectory. This allows for evaluation under different noise levels of the IMU. 

## Installation

Using HoloOcean is only supported under Windows and Linux distributions. GTSAM Python library can be installed via pip, however this only works under MacOS and Linux (Windows has to be build manually.)
We therefore provide installation guides for Linux which includes HoloOcean and the Factor-Based-SLAM methods, and on installation guide for only installing the Factor-Based-SLAM. To be able to run the Factor-Based-SLAM methods on must install and build GTSAM under Windows (https://github.com/borglab/gtsam/blob/develop/INSTALL.md).

### Installation Factor-Based-SLAM + Holoocean


### Installation Factor-Based-SLAM
